module mult8bs(
    input signed [7:0] a,
    input signed [7:0] b,
    output signed [15:0] out
    );

    wire [15:0] tempout;
    assign tempout = a*b;
    assign out = tempout[15:0];
    
endmodule