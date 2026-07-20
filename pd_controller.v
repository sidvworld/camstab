module pd_controller (
    input wire clk,
    input wire rst_i,

    input wire signed [15:0] roll_err_i,
    input wire signed [15:0] gyro_roll_i,

    output reg signed [15:0] pd_o
    );

    parameter signed [15:0] KP = 16'sd10;
    parameter signed [15:0] KD = 16'sd2;

    wire signed [31:0] kp_term;
    wire signed [31:0] kd_term;
    wire signed [31:0] pd_term;

    assign kp_term = roll_err_i * KP;
    assign kd_term = gyro_roll_i * KD;
    assign pd_term = kp_term - kd_term;

    always @(posedge clk) begin
        if (rst_i) begin
            pd_o <= 16'sd0;
        end else begin
            pd_o <= pd_term[15:0];
        end
    end

    `ifdef COCOTB_SIM
        initial begin
            $dumpfile("pd_controller.vcd");
            $dumpvars(0, pd_controller);
            #1;
        end
    `endif


endmodule