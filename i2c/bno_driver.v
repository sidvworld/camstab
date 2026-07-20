/*
 * Module: bno055_i2c_fsm
 **********************************************************************************************************************************************************
 * Purpose: Drive the i2c_master module (100MHz clock, 400KHz SCL) through the Bosch-recommended BNO055 init sequence, then continuously read the
 *          16-bit Euler Roll angle and 16-bit Gyroscope X-axis rate.
 *
 *          Init sequence:
 *            1) Wait for power-on (POR_DELAY_CYCLES).
 *            2) Read CHIP_ID (register 0x00), confirm it reads back 0xA0. Retry up to CHIP_ID_RETRY_LIMIT times if it doesn't.
 *            3) Write CONFIGMODE (0x00) to OPR_MODE (register 0x3D).
 *            4) Wait ~20ms (MODE_SWITCH_DELAY_CYCLES).
 *            5) (Axis/unit configuration would go here if needed - not required for this application.)
 *            6) Write NDOF (0x0C) to OPR_MODE (register 0x3D).
 *            7) Wait ~20ms (MODE_SWITCH_DELAY_CYCLES).
 *
 *          Steady-state loop (forever): read EUL_Roll (LSB+MSB, registers 0x1C/0x1D) -> read GYR_DATA_X (LSB+MSB, registers 0x14/0x15) -> repeat.
 **********************************************************************************************************************************************************
 * Notes:  * Assumes the BNO055 ADR pin is tied low, giving a 7-bit I2C address of 0x28 (change BNO055_I2C_ADDR if ADR is tied high -> 0x29).
 *         * i2c_master's i_sub_addr/i_sub_len scheme: for an 8-bit register address, put the address in i_sub_addr[7:0] and set i_sub_len = 0.
 *           Internally i2c_master left-shifts it into sub_addr[15:8] before sending it, since it always transmits the upper byte first.
 *         * Multi-byte reads: BNO055 registers auto-increment on sequential reads, and the LSB register always sits at the lower address, so
 *           a byte_len = 2 read starting at the LSB address returns LSB first, then MSB. That ordering is relied on below when assembling the
 *           16-bit roll/gyr    o values (byte_idx tracks which byte of the pair just arrived on valid_out).
 *         * A transaction is considered complete when i2c_master's `busy` signal falls (1 -> 0). This is detected here with a one-cycle-delayed
 *           copy of busy (busy_d) rather than polling busy==0 directly, since busy is still 0 for one cycle right before a new transaction issues.
 *         * o_nack_error is a sticky/latched flag - once any transaction is NACKed it stays high. Clear it externally via reset_n if desired.
 *         * Delay values assume i_clk = 100MHz, matching the i2c_master module's expected input clock.
 **********************************************************************************************************************************************************
 */

`timescale 1fs/1fs
module bno_driver #(
    parameter [6:0]  BNO055_I2C_ADDR          = 7'h28,          // 7-bit slave address (ADR pin low = 0x28, ADR pin high = 0x29)
    parameter [7:0]  CHIP_ID_REG              = 8'h00,          // Chip ID register
    parameter [7:0]  CHIP_ID_VAL              = 8'hA0,          // Expected chip ID value
    parameter [7:0]  OPR_MODE_REG             = 8'h3D,          // Operation mode register
    parameter [7:0]  CONFIG_MODE              = 8'h00,          // CONFIGMODE value
    parameter [7:0]  NDOF_MODE                = 8'h0C,          // NDOF (9-DOF fusion) mode value
    parameter [7:0]  EUL_ROLL_LSB_REG         = 8'h1C,          // Euler roll angle, LSB (MSB is 0x1D, auto-read next)
    parameter [7:0]  GYR_DATA_X_LSB_REG       = 8'h14,          // Gyroscope X axis data, LSB (MSB is 0x15, auto-read next)
    parameter [31:0] POR_DELAY_CYCLES         = 32'd10_200_000, // ~850ms @ 100MHz, worst-case BNO055 power-on-reset time before it will ACK
    parameter [31:0] MODE_SWITCH_DELAY_CYCLES = 32'd240_000,  // ~20ms  @ 100MHz, Bosch-recommended delay after each OPR_MODE write
    parameter [31:0] CHIP_ID_RETRY_DELAY_CYCLES = 32'd120_000,// ~10ms  @ 100MHz, delay between CHIP_ID read retries
    parameter [7:0]  CHIP_ID_RETRY_LIMIT      = 8'd10           // give up confirming CHIP_ID after this many attempts (still proceeds, flags error)
)(
    input             i_clk,          // 100MHz system clock, fed straight into i2c_master
    input             reset_n,        // active low async reset

    inout             scl_o,          // I2C clock line to BNO055 (needs external pull-up resistor)
    inout             sda_o,          // I2C data line to BNO055 (needs external pull-up resistor)

    output wire [1:0] o_roll_msb,         // most recently read 16-bit Euler roll angle {MSB, LSB}
    output reg        o_roll_valid,   // pulses high for 1 cycle when o_roll updates
    // output reg [15:0] o_gyro_x,       // most recently read 16-bit Gyroscope X-axis rate {MSB, LSB}
    output reg        o_gyro_x_valid, // pulses high for 1 cycle when o_gyro_x updates

    output reg        o_init_done,    // goes high once the full init sequence (chip id -> configmode -> ndof) has completed
    output reg        o_chip_id_error,// latched high if CHIP_ID never matched CHIP_ID_VAL within CHIP_ID_RETRY_LIMIT attempts
    output reg        o_nack_error    // sticky flag: set if any transaction was NACKed by the slave
);

//======================================================================
// i2c_master instantiation
//======================================================================
reg  [7:0]  addr_w_rw;
reg  [15:0] sub_addr;
reg         sub_len;
reg  [23:0] byte_len;
reg  [7:0]  data_write;
reg         req_trans;

wire [7:0]  data_out;
wire        valid_out;
wire        req_data_chunk;   // unused here - only relevant for multi-byte writes
wire        busy;
wire        nack;

// FOR TESTING:
reg [15:0] o_roll;
reg [15:0] o_gyro_x;

assign o_roll_msb = o_roll[9:8];

i2c_master u_i2c_master (
    .i_clk          (i_clk),
    .reset_n        (reset_n),
    .i_addr_w_rw    (addr_w_rw),
    .i_sub_addr     (sub_addr),
    .i_sub_len      (sub_len),
    .i_byte_len     (byte_len),
    .i_data_write   (data_write),
    .req_trans      (req_trans),

    .data_out       (data_out),
    .valid_out      (valid_out),

    .scl_o          (scl_o),
    .sda_o          (sda_o),

    .req_data_chunk (req_data_chunk),
    .busy           (busy),
    .nack           (nack)
);

//======================================================================
// Top-level FSM
//======================================================================
localparam [3:0] S_RESET_WAIT     = 4'd0,   // hold off after reset so BNO055 can finish power-on-reset
                 S_CHIPID_ISSUE   = 4'd1,   // load read(CHIP_ID, 1 byte) and pulse req_trans
                 S_CHIPID_WAIT    = 4'd2,   // wait for the read to finish, capture/compare data_out
                 S_CHIPID_RETRY_DELAY = 4'd3, // short delay before re-attempting CHIP_ID read
                 S_CFGMODE_ISSUE  = 4'd4,   // load write(OPR_MODE, CONFIGMODE) and pulse req_trans
                 S_CFGMODE_WAIT   = 4'd5,   // wait for the write to finish
                 S_CFGMODE_SETTLE = 4'd6,   // ~20ms settle time
                 S_NDOF_ISSUE     = 4'd7,   // load write(OPR_MODE, NDOF) and pulse req_trans
                 S_NDOF_WAIT      = 4'd8,   // wait for the write to finish
                 S_NDOF_SETTLE    = 4'd9,   // ~20ms settle time
                 S_ROLL_ISSUE     = 4'd10,  // load read(EUL_ROLL_LSB, 2 bytes) and pulse req_trans
                 S_ROLL_WAIT      = 4'd11,  // wait for the read to finish, assembling {MSB,LSB} along the way
                 S_GYRO_ISSUE     = 4'd12,  // load read(GYR_DATA_X_LSB, 2 bytes) and pulse req_trans
                 S_GYRO_WAIT      = 4'd13;  // wait for the read to finish, assembling {MSB,LSB} along the way

reg [3:0]  state;
reg        busy_d;                    // busy delayed by 1 cycle, used to catch the 1->0 edge (transaction complete)
wire       trans_done = busy_d & ~busy;

reg [31:0] delay_cntr;                // reused for POR wait, mode-switch settle waits, and chip-id retry delay
reg [7:0]  chip_id_retry_cntr;        // counts CHIP_ID read attempts
reg        byte_idx;                  // 0 = expecting LSB (1st byte of a 2-byte read), 1 = expecting MSB (2nd byte)

always@(posedge i_clk or negedge reset_n) begin
    if(!reset_n) begin
        state              <= S_RESET_WAIT;
        busy_d             <= 1'b0;
        req_trans          <= 1'b0;
        addr_w_rw          <= 8'b0;
        sub_addr           <= 16'b0;
        sub_len            <= 1'b0;
        byte_len           <= 24'b0;
        data_write         <= 8'b0;
        delay_cntr         <= 32'b0;
        chip_id_retry_cntr <= 8'b0;
        byte_idx           <= 1'b0;
        o_roll             <= 16'b0;
        o_roll_valid       <= 1'b0;
        o_gyro_x           <= 16'b0;
        o_gyro_x_valid     <= 1'b0;
        o_init_done        <= 1'b0;
        o_chip_id_error    <= 1'b0;
        o_nack_error       <= 1'b0;
    end
    else begin
        // Defaults - these are only ever asserted for a single cycle below
        req_trans      <= 1'b0;
        o_roll_valid   <= 1'b0;
        o_gyro_x_valid <= 1'b0;
        busy_d         <= busy;

        case(state)

            //--------------------------------------------------------
            // 1) Wait for power-on. BNO055 datasheet worst-case POR
            //    time is ~850ms before it will reliably respond.
            //--------------------------------------------------------
            S_RESET_WAIT: begin
                if(delay_cntr == POR_DELAY_CYCLES) begin
                    delay_cntr <= 32'b0;
                    state      <= S_CHIPID_ISSUE;
                end
                else
                    delay_cntr <= delay_cntr + 1'b1;
            end

            //--------------------------------------------------------
            // 2) Read CHIP_ID (0x00), confirm it equals 0xA0.
            //--------------------------------------------------------
            S_CHIPID_ISSUE: begin
                if(!busy) begin
                    addr_w_rw <= {BNO055_I2C_ADDR, 1'b1};    // read
                    sub_addr  <= {8'b0, CHIP_ID_REG};
                    sub_len   <= 1'b0;
                    byte_len  <= 24'd1;
                    req_trans <= 1'b1;
                    state     <= S_CHIPID_WAIT;
                end
            end

            S_CHIPID_WAIT: begin
                if(trans_done) begin
                    o_nack_error <= o_nack_error | nack;
                    if(data_out == CHIP_ID_VAL) begin
                        chip_id_retry_cntr <= 8'b0;
                        state              <= S_CFGMODE_ISSUE;
                    end
                    else if(chip_id_retry_cntr == CHIP_ID_RETRY_LIMIT) begin
                        // Give up waiting for a clean CHIP_ID match, flag it, and proceed anyway
                        o_chip_id_error <= 1'b1;
                        state           <= S_CFGMODE_ISSUE;
                    end
                    else begin
                        chip_id_retry_cntr <= chip_id_retry_cntr + 1'b1;
                        state              <= S_CHIPID_RETRY_DELAY;
                    end
                end
            end

            S_CHIPID_RETRY_DELAY: begin
                if(delay_cntr == CHIP_ID_RETRY_DELAY_CYCLES) begin
                    delay_cntr <= 32'b0;
                    state      <= S_CHIPID_ISSUE;
                end
                else
                    delay_cntr <= delay_cntr + 1'b1;
            end

            //--------------------------------------------------------
            // 3) Write CONFIGMODE (0x00) to OPR_MODE (0x3D).
            //--------------------------------------------------------
            S_CFGMODE_ISSUE: begin
                if(!busy) begin
                    addr_w_rw  <= {BNO055_I2C_ADDR, 1'b0};   // write
                    sub_addr   <= {8'b0, OPR_MODE_REG};
                    sub_len    <= 1'b0;
                    byte_len   <= 24'd1;
                    data_write <= CONFIG_MODE;
                    req_trans  <= 1'b1;
                    state      <= S_CFGMODE_WAIT;
                end
            end

            S_CFGMODE_WAIT: begin
                if(trans_done) begin
                    o_nack_error <= o_nack_error | nack;
                    state        <= S_CFGMODE_SETTLE;
                end
            end

            //--------------------------------------------------------
            // 4) Wait ~20ms.
            //    (5) Axis/unit configuration writes would slot in here.
            //--------------------------------------------------------
            S_CFGMODE_SETTLE: begin
                if(delay_cntr == MODE_SWITCH_DELAY_CYCLES) begin
                    delay_cntr <= 32'b0;
                    state      <= S_NDOF_ISSUE;
                end
                else
                    delay_cntr <= delay_cntr + 1'b1;
            end

            //--------------------------------------------------------
            // 6) Write NDOF (0x0C) to OPR_MODE (0x3D).
            //--------------------------------------------------------
            S_NDOF_ISSUE: begin
                if(!busy) begin
                    addr_w_rw  <= {BNO055_I2C_ADDR, 1'b0};   // write
                    sub_addr   <= {8'b0, OPR_MODE_REG};
                    sub_len    <= 1'b0;
                    byte_len   <= 24'd1;
                    data_write <= NDOF_MODE;
                    req_trans  <= 1'b1;
                    state      <= S_NDOF_WAIT;
                end
            end

            S_NDOF_WAIT: begin
                if(trans_done) begin
                    o_nack_error <= o_nack_error | nack;
                    state        <= S_NDOF_SETTLE;
                end
            end

            //--------------------------------------------------------
            // 7) Wait ~20ms, then init is complete.
            //--------------------------------------------------------
            S_NDOF_SETTLE: begin
                if(delay_cntr == MODE_SWITCH_DELAY_CYCLES) begin
                    delay_cntr  <= 32'b0;
                    o_init_done <= 1'b1;
                    state       <= S_ROLL_ISSUE;
                end
                else
                    delay_cntr <= delay_cntr + 1'b1;
            end

            //--------------------------------------------------------
            // Steady state: read 2 bytes starting at EUL_Roll_LSB
            // (0x1C). BNO055 auto-increments, so byte order returned
            // is LSB then MSB.
            //--------------------------------------------------------
            S_ROLL_ISSUE: begin
                if(!busy) begin
                    addr_w_rw <= {BNO055_I2C_ADDR, 1'b1};    // read
                    sub_addr  <= {8'b0, EUL_ROLL_LSB_REG};
                    sub_len   <= 1'b0;
                    byte_len  <= 24'd2;
                    byte_idx  <= 1'b0;
                    req_trans <= 1'b1;
                    state     <= S_ROLL_WAIT;
                end
            end

            S_ROLL_WAIT: begin
                if(valid_out) begin
                    if(!byte_idx)
                        o_roll[7:0]  <= data_out;   // 1st byte = LSB
                    else
                        o_roll[15:8] <= data_out;   // 2nd byte = MSB
                    byte_idx <= 1'b1;
                end
                if(trans_done) begin
                    o_nack_error <= o_nack_error | nack;
                    o_roll_valid <= 1'b1;
                    state        <= S_GYRO_ISSUE;
                end
            end

            //--------------------------------------------------------
            // Steady state: read 2 bytes starting at GYR_DATA_X_LSB
            // (0x14). Same LSB-then-MSB ordering as above.
            //--------------------------------------------------------
            S_GYRO_ISSUE: begin
                if(!busy) begin
                    addr_w_rw <= {BNO055_I2C_ADDR, 1'b1};    // read
                    sub_addr  <= {8'b0, GYR_DATA_X_LSB_REG};
                    sub_len   <= 1'b0;
                    byte_len  <= 24'd2;
                    byte_idx  <= 1'b0;
                    req_trans <= 1'b1;
                    state     <= S_GYRO_WAIT;
                end
            end

            S_GYRO_WAIT: begin
                if(valid_out) begin
                    if(!byte_idx)
                        o_gyro_x[7:0]  <= data_out;   // 1st byte = LSB
                    else
                        o_gyro_x[15:8] <= data_out;   // 2nd byte = MSB
                    byte_idx <= 1'b1;
                end
                if(trans_done) begin
                    o_nack_error   <= o_nack_error | nack;
                    o_gyro_x_valid <= 1'b1;
                    state          <= S_ROLL_ISSUE;   // loop forever: roll -> gyro -> roll -> ...
                end
            end

            default: state <= S_RESET_WAIT;

        endcase
    end
end

endmodule