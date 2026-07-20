module pwm_compiler(
    input wire clk,
    input wire rst_i,

    // PD controller output
    input wire signed [15:0] pd_i,

    output wire [15:0] pwm_cmd_o
    );


    parameter CENTER_PWM = 16'd1500;
    parameter MIN_PWM = 16'd1000;
    parameter MAX_PWM = 16'd2000;


    reg signed [16:0] pwm_calc_r;


    always @(posedge clk) begin
        if (rst_i) begin
            pwm_cmd_o <= CENTER_PWM;
        end

        else begin
            pwm_calc_r = $signed(CENTER_PWM) + pd_i;

            // clamping
            if (pwm_calc_r > MAX_PWM) begin
                pwm_cmd_o <= MAX_PWM;
            end

            else if (pwm_calc_r < MIN_PWM) begin
                pwm_cmd_o <= MIN_PWM;
            end

            else begin
                pwm_cmd_o <= pwm_calc_r[15:0];
            end

        end

    end


    `ifdef COCOTB_SIM
        initial begin
            $dumpfile("pwm_compiler.vcd");
            $dumpvars(0, pwm_compiler);
            #1;
        end
    `endif


endmodule