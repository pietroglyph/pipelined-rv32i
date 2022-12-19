`timescale 1ns/1ps
`default_nettype none

module decoder_5_to_32(ena, in, out);

input wire ena;
input wire [4:0] in;
output logic [31:0] out;

// SOLUTION CODE START

wire [1:0] enas;
decoder_1_to_2 DECODER_ENA(ena, in[4], enas);
decoder_4_to_16 DECODER_0(enas[0], in[3:0], out[15:0]);
decoder_4_to_16 DECODER_1(enas[1], in[3:0], out[31:16]);

// SOLUTION CODE END

endmodule