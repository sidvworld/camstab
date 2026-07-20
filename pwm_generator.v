module pwm_generator
    (
    input wire clk,
    input wire rst_i,

    // pulse width command in us
    input wire [15:0] pwm_cmd_i,

    output reg servo_pwm_o
    );


    localparam CLK_FREQ = 12_000_000,
    localparam PWM_FREQ = 50
    
    // 1us = 12 clk cycles at 12MHz
    localparam integer CYCLES_PER_US = CLK_FREQ / 1_000_000;

    // PWM period = 20ms
    localparam integer PWM_PERIOD = CLK_FREQ / PWM_FREQ;


    reg [31:0] counter;

    reg [31:0] pulse_cycles;


    always @(posedge clk) begin

        if (rst_i) begin
            counter <= 0;
            servo_pwm_o <= 1'b0;
        end


        else begin

            // convert microseconds to clock cycles
            pulse_cycles <= pwm_cmd_i * CYCLES_PER_US;


            // this to reset PWM period
            if (counter >= PWM_PERIOD-1) begin
                counter <= 0;
            end

            else begin
                counter <= counter + 1;
            end


            // generate PWM pulse
            if (counter < pulse_cycles)
                servo_pwm_o <= 1'b1;
            else
                servo_pwm_o <= 1'b0;

        end

    end


    `ifdef COCOTB_SIM
        initial begin
            $dumpfile("pwm_generator.vcd");
            $dumpvars(0, pwm_generator);
            #1;
        end
    `endif


endmodule