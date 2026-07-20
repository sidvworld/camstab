module error_calculator
    (
    input wire clk,
    input wire rst_i,

    input wire signed [15:0] roll_target_i,

    input wire signed [15:0] roll_i,

    output reg signed [15:0] roll_error_o
    );


    always @(posedge clk) begin
        if (rst_i) begin
            roll_error_o <= 16'sd0;
        end
        else begin
            roll_error_o <= roll_target_i - roll_i;
        end
    end


    `ifdef COCOTB_SIM
        initial begin
            $dumpfile("error_calculator.vcd");
            $dumpvars(0, error_calculator);
            #1;
        end
    `endif

endmodule