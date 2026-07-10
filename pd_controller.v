module pd_controller
    #(parameter signed [7:0] KP = 10,
    parameter signed [7:0] KD = 2)
    (input clk,
    input rst_i,

    input signed [7:0] err_x_i,
    input signed [7:0] err_y_i,
    input signed [7:0] err_z_i,

    input signed [7:0] gyro_x_i,
    input signed [7:0] gyro_y_i,
    input signed [7:0] gyro_z_i,

    output signed [15:0] out_x_o,
    output signed [15:0] out_y_o,
    output signed [15:0] out_z_o
    );

    wire signed [15:0] kp_term_x;
    wire signed [15:0] kp_term_y;
    wire signed [15:0] kp_term_z;

    wire signed [15:0] kd_term_x;
    wire signed [15:0] kd_term_y;
    wire signed [15:0] kd_term_z;

    reg signed [15:0] out_x_r;
    reg signed [15:0] out_y_r;
    reg signed [15:0] out_z_r;

    mult8bs mult_kpx (.a(KP), .b(err_x_i), .out(kp_term_x));
    mult8bs mult_kpy (.a(KP), .b(err_y_i), .out(kp_term_y));
    mult8bs mult_kpz (.a(KP), .b(err_z_i), .out(kp_term_z));

    mult8bs mult_kdx (.a(KD), .b(gyro_x_i), .out(kd_term_x));
    mult8bs mult_kdy (.a(KD), .b(gyro_y_i), .out(kd_term_y));
    mult8bs mult_kdz (.a(KD), .b(gyro_z_i), .out(kd_term_z));

    always @(posedge clk) begin
        if (rst_i) begin
            out_x_r <= 16'sd0;
            out_y_r <= 16'sd0;
            out_z_r <= 16'sd0;
        end
        else begin
            out_x_r <= kp_term_x - kd_term_x;
            out_y_r <= kp_term_y - kd_term_y;
            out_z_r <= kp_term_z - kd_term_z;
        end
    end

    assign out_x_o = out_x_r;
    assign out_y_o = out_y_r;
    assign out_z_o = out_z_r;

    `ifdef COCOTB_SIM
        initial begin
            $dumpfile("pd_controller.vcd");
            $dumpvars(0, pd_controller);
            #1;
        end
    `endif

endmodule