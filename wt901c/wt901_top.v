// Language: Verilog 2001
`timescale 1ns / 1ps
/*
 * Example top-level: uart_rx -> wt901_depacketizer
 *
 * uart_rx here is the AXI-Stream UART receiver you provided
 * (Alex Forencich, uart.v). prescale sets the baud rate:
 *
 *   prescale = clk_freq / (baud_rate * 8)
 *
 * e.g. for a 50 MHz clk and 9600 baud:
 *   prescale = 50_000_000 / (9600 * 8) = 651  (approx)
 *
 * Adjust CLK_FREQ below to match your actual system clock.
 */
module wt901_top_example #(
    parameter CLK_FREQ  = 12_000_000,
    parameter BAUD_RATE = 9600
) (
    input  wire clk,
    input  wire rst,
    input  wire rxd,

    output wire signed [15:0] roll_raw,
    output wire signed [15:0] pitch_raw,
    output wire signed [15:0] yaw_raw,        // Z-axis angle
    output wire signed [15:0] angle_temp_raw,
    output wire                angle_valid,

    output wire signed [15:0] gyro_x_raw,
    output wire signed [15:0] gyro_y_raw,
    output wire signed [15:0] gyro_z_raw,     // Z-axis angular rate
    output wire signed [15:0] gyro_temp_raw,
    output wire                gyro_valid,

    output wire                checksum_error,

    output wire uart_busy,
    output wire uart_overrun_error,
    output wire uart_frame_error
);

    localparam [15:0] PRESCALE = CLK_FREQ / (BAUD_RATE * 8);

    wire [7:0] uart_tdata;
    wire       uart_tvalid;
    wire       uart_tready;

    uart_rx uart_rx_inst (
        .clk            (clk),
        .rst            (rst),
        .m_axis_tdata   (uart_tdata),
        .m_axis_tvalid  (uart_tvalid),
        .m_axis_tready  (uart_tready),
        .rxd            (rxd),
        .busy           (uart_busy),
        .overrun_error  (uart_overrun_error),
        .frame_error    (uart_frame_error),
        .prescale       (PRESCALE)
    );

    wt901_depacketizer wt901_depacketizer_inst (
        .clk             (clk),
        .rst             (rst),
        .s_axis_tdata    (uart_tdata),
        .s_axis_tvalid   (uart_tvalid),
        .s_axis_tready   (uart_tready),

        .roll_raw        (roll_raw),
        .pitch_raw       (pitch_raw),
        .yaw_raw         (yaw_raw),
        .angle_temp_raw  (angle_temp_raw),
        .angle_valid     (angle_valid),

        .gyro_x_raw      (gyro_x_raw),
        .gyro_y_raw      (gyro_y_raw),
        .gyro_z_raw      (gyro_z_raw),
        .gyro_temp_raw   (gyro_temp_raw),
        .gyro_valid      (gyro_valid),

        .checksum_error  (checksum_error)
    );

endmodule