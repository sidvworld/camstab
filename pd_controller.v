module pd_controller
    #(parameter [15:0] KP = 10,
    parameter [15:0] KD = 2)
    (
    input clk,
    input rst_i,

    input [15:0] err_x_i,
    input [15:0] err_y_i,
    input [15:0] err_z_i,

    input [15:0] gyro_x_i,
    input [15:0] gyro_y_i,
    input [15:0] gyro_z_i,

    output [15:0] out_x_o,
    output [15:0] out_y_o,
    output [15:0] out_z_o
);

    wire [31:0] kp_term_x;
    wire [31:0] kp_term_y;
    wire [31:0] kp_term_z;

    wire [31:0] kd_term_x;
    wire [31:0] kd_term_y;
    wire [31:0] kd_term_z;
    
    out_x_r reg [15:0];
    out_y_r reg [15:0];
    out_z_r reg [15:0];

    multiplier_16x16 mult_kpx(.M(KP), .Q(err_x_i), .product(kp_term_x));
    multiplier_16x16 mult_kpy(.M(KP), .Q(err_y_i), .product(kp_term_y));
    multiplier_16x16 mult_kpz(.M(KP), .Q(err_z_i), .product(kp_term_z));

    multiplier_16x16 mult_kdx(.M(KD), .Q(gyro_x_i), .product(kd_term_x));
    multiplier_16x16 mult_kdy(.M(KD), .Q(gyro_y_i), .product(kd_term_y));
    multiplier_16x16 mult_kdz(.M(KD), .Q(gyro_z_i), .product(kd_term_z));

    always @(posedge clk)
    begin
    if (rst_i)
        out_x_r <= 0;
        out_y_r <= 0;
        out_z_r <= 0;

    else
        out_x_r <= (kp_term_x) - (kd_term_x);
        out_y_r <= (kp_term_y) - (kd_term_y);
        out_z_r <= (kp_term_z) - (kd_term_z);
    end

    assign out_x_o = out_x_r;
    assign out_y_o = out_y_r;
    assign out_z_o = out_z_r;

endmodule