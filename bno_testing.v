module bno_testing(
    input wire clk,
    input wire SDA_i,
    output wire LED_RED_o,
    output wire SDA_o
);

reg SDA_prev = 1'b0;
reg RED_r = 1'b0;
reg [23:0] timer = 24'd0;

assign LED_RED_o = RED_r;
assign SDA_o = SDA_i;

always @(posedge clk) begin
    SDA_prev <= SDA_i;

    if (SDA_i != SDA_prev)
        timer <= 24'd6000000;
    else if (timer > 0)
        timer <= timer - 1;

    if (timer > 0)
        RED_r <= 1'b1;
    else
        RED_r <= 1'b0;
end

endmodule