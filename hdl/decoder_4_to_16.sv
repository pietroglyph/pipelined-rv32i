`timescale 1ns/1ps
`default_nettype none

module decoder_4_to_16(ena, in, out);

input wire ena;
input wire [3:0] in;
output logic [15:0] out;

// SOLUTION CODE START

wire [1:0] enas;
decoder_1_to_2 DECODER_ENA(ena, in[3], enas);
decoder_3_to_8 DECODER_0(enas[0], in[2:0], out[7:0]);
decoder_3_to_8 DECODER_1(enas[1], in[2:0], out[15:8]);

// SOLUTION CODE END

endmodule