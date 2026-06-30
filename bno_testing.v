module bno_testing(
    input wire clk,
    input wire SDA_i,
    output wire LED_RED_o
);

reg SDA_prev = 1'b0;
reg RED_r = 1'b0;

reg [23:0] timer = 24'd0;

assign LED_RED_o = RED_r;

always @(posedge clk)
begin
    if (SDA_i != SDA_prev)
        timer <= 24'd6000000;

    SDA_prev <= SDA_i;

    if (timer > 0) begin
        timer <= timer - 1;
        RED_r <= 1'b1;
    end
    else begin
        RED_r <= 1'b0;
    end

end

endmodule