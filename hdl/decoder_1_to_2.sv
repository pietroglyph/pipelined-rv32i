`timescale 1ns/1ps
`default_nettype none

module decoder_1_to_2(ena, in, out);

input wire ena;
input wire in;
output logic [1:0] out;

always_comb begin
  out[1] = ena & in;
  out[0] = ena & ~in;
end

endmodule