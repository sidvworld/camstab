module bno_testing(
    input wire clk,
    input wire SDA_i,
    input wire SCL_i,
    input wire RST_i,
    output wire LED_RED_o,
    output wire SDA_o,
    output wire SCL_o,
    output wire RST_o
);

reg SDA_prev = 1'b0;
reg RED_r = 1'b0;
reg [23:0] timer = 24'd0;
reg [3:0] edge_count = 4'd0;

assign LED_RED_o = RED_r;
assign SDA_o = SDA_i;
assign SCL_o = SCL_i;
assign RST_o = RST_i;

// SDA testing
always @(posedge clk) begin
    if (SDA_i != SDA_prev) begin
        SDA_prev <= SDA_i;

        if (edge_count < 8)
            edge_count <= edge_count + 1;

        if (edge_count == 7) begin
            timer <= 24'd6000000;
            edge_count <= 4'd0;
        end
    end

    if (timer > 0)
        timer <= timer - 1;

    RED_r <= (timer > 0);
end

endmodule