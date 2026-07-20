
`timescale 1ns / 1ps
module wt901_depacketizer (
    input  wire clk,
    input  wire rst,

    input  wire [7:0] s_axis_tdata,
    input  wire s_axis_tvalid,
    output wire s_axis_tready,

    output reg signed [15:0] roll_raw,
    output reg signed [15:0] pitch_raw,
    output reg signed [15:0] yaw_raw,       // need this
    output reg signed [15:0] angle_temp_raw,
    output reg angle_valid,

    output reg signed [15:0] gyro_x_raw,
    output reg signed [15:0] gyro_y_raw,
    output reg signed [15:0] gyro_z_raw,
    output reg signed [15:0] gyro_temp_raw,
    output reg gyro_valid,

    output reg checksum_error
);
    assign s_axis_tready = 1'b1;

    localparam [1:0]
        ST_HEADER = 2'd0,  // waiting for 0x55
        ST_TYPE = 2'd1,  // waiting for 0x53 (angle) or 0x52 (gyro)
        ST_DATA = 2'd2,  // collecting 8 data bytes
        ST_CHECKSUM = 2'd3;  // checking SUM byte

    localparam [7:0]
        TYPE_GYRO  = 8'h52,
        TYPE_ANGLE = 8'h53;

    reg [1:0]  state = ST_HEADER;
    reg [7:0]  pkt_type = 8'd0;
    reg [3:0]  byte_cnt = 4'd0;
    reg [7:0]  sum = 8'd0;
    reg [7:0]  data_bytes [0:7]; // generic 8-byte payload for either packet type

    integer i;

    always @(posedge clk) begin
        if (rst) begin
            state          <= ST_HEADER;
            pkt_type       <= 8'd0;
            byte_cnt       <= 4'd0;
            sum            <= 8'd0;
            angle_valid    <= 1'b0;
            gyro_valid     <= 1'b0;
            checksum_error <= 1'b0;
            roll_raw       <= 16'sd0;
            pitch_raw      <= 16'sd0;
            yaw_raw        <= 16'sd0;
            angle_temp_raw <= 16'sd0;
            gyro_x_raw     <= 16'sd0;
            gyro_y_raw     <= 16'sd0;
            gyro_z_raw     <= 16'sd0;
            gyro_temp_raw  <= 16'sd0;
            for (i = 0; i < 8; i = i + 1) begin
                data_bytes[i] <= 8'd0;
            end
        end else begin
            // these are 1-cycle strobes
            angle_valid    <= 1'b0;
            gyro_valid     <= 1'b0;
            checksum_error <= 1'b0;

            if (s_axis_tvalid) begin
                case (state)

                    ST_HEADER: begin
                        if (s_axis_tdata == 8'h55) begin
                            sum   <= 8'h55;
                            state <= ST_TYPE;
                        end
                        // else: garbage byte, keep waiting in ST_HEADER
                    end

                    ST_TYPE: begin
                        if (s_axis_tdata == TYPE_ANGLE || s_axis_tdata == TYPE_GYRO) begin
                            // recognized packet type - start collecting data bytes
                            pkt_type <= s_axis_tdata;
                            sum      <= sum + s_axis_tdata;
                            byte_cnt <= 4'd0;
                            state    <= ST_DATA;
                        end else if (s_axis_tdata == 8'h55) begin
                            // could be the header of the *next* packet -
                            // resync without losing a byte
                            sum   <= 8'h55;
                            state <= ST_TYPE;
                        end else begin
                            // some other packet type (accel/mag/quaternion/etc) -
                            // we don't decode it, drop back to header search
                            state <= ST_HEADER;
                        end
                    end

                    ST_DATA: begin
                        data_bytes[byte_cnt] <= s_axis_tdata;
                        sum                  <= sum + s_axis_tdata;
                        if (byte_cnt == 4'd7) begin
                            state <= ST_CHECKSUM;
                        end else begin
                            byte_cnt <= byte_cnt + 4'd1;
                        end
                    end

                    ST_CHECKSUM: begin
                        if (s_axis_tdata == sum) begin
                            // good packet - latch outputs based on type
                            if (pkt_type == TYPE_ANGLE) begin
                                roll_raw       <= {data_bytes[1], data_bytes[0]};
                                pitch_raw      <= {data_bytes[3], data_bytes[2]};
                                yaw_raw        <= {data_bytes[5], data_bytes[4]};
                                angle_temp_raw <= {data_bytes[7], data_bytes[6]};
                                angle_valid    <= 1'b1;
                            end else if (pkt_type == TYPE_GYRO) begin
                                gyro_x_raw     <= {data_bytes[1], data_bytes[0]};
                                gyro_y_raw     <= {data_bytes[3], data_bytes[2]};
                                gyro_z_raw     <= {data_bytes[5], data_bytes[4]};
                                gyro_temp_raw  <= {data_bytes[7], data_bytes[6]};
                                gyro_valid     <= 1'b1;
                            end
                        end else begin
                            checksum_error <= 1'b1;
                        end
                        state <= ST_HEADER;
                    end

                    default: state <= ST_HEADER;

                endcase
            end
        end
    end

endmodule