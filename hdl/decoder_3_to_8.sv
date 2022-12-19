`timescale 1ns/1ps
`default_nettype none

module decoder_3_to_8(ena, in, out);

  input wire ena;
  input wire [2:0] in;
  output logic [7:0] out;

  // SOLUTION CODE START

  // Modular Solution is far more succinct here.
  wire [1:0] enas;
  decoder_1_to_2 DECODER_ENA(ena, in[2], enas);
  decoder_2_to_4 DECODER_0(enas[0], in[1:0], out[3:0]);
  decoder_2_to_4 DECODER_1(enas[1], in[1:0], out[7:4]);

  // SOLUTION CODE END

endmodule