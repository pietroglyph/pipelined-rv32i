`timescale 1ns/1ps
`default_nettype none

module pwm(clk, rst, ena, step, duty, out);

parameter N = 8;

input wire clk, rst;
input wire ena; // Enables the output.
input wire step; // Enables the internal counter. You should only increment when this signal is high (this is how we slow down the PWM to reasonable speeds).
input wire [N-1:0] duty; // The "duty cycle" input.
output logic out;

logic [N-1:0] counter;

logic comparator;

always_comb begin
  comparator = counter < duty;
  out = ena & ( comparator | &counter );
end

always_ff @(posedge clk) begin
  if(rst) begin
    counter <=0;
  end
  else if (step) begin
    counter <= counter + 1;
  end
end

// SOLUTION END

endmodule
