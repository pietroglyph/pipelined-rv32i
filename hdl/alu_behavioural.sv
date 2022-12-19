`timescale 1ns/1ps
`default_nettype none

`include "alu_types.sv"

module alu_behavioural(a, b, control, result, overflow, zero, equal);
parameter N = 32;

input wire signed [N-1:0] a, b;
input alu_control_t control;

output logic signed [N-1:0] result; // Result of the selected operation.

output logic overflow; // Is high if the result of an ADD or SUB wraps around the 32 bit boundary.
output logic zero;  // Is high if the result is ever all zeros.
output logic equal; // is high if a == b.

logic unsigned [N-1:0] unsigned_a, unsigned_b; // behavioural logic doesn't work right without setting signed or unsigned. Pretty much just for SLTU
logic carry_out;
logic [N-1:0] sum, difference;

always_comb begin : behavioural_alu_logic
  unsigned_a = a;
  unsigned_b = b;
  {carry_out, sum} = a + b;
  difference = a - b;
  case (control) // This is how you make  a MUX.
    ALU_AND: result = a & b;
    ALU_OR : result = a | b;
    ALU_XOR : result = a ^ b;
    ALU_SLL : result = a << b;
    ALU_SRL : result = a >>  b;
    ALU_SRA : result = (unsigned_b > 32'd32) ? 0 : a >>> unsigned_b;
    ALU_ADD : result = sum;
    ALU_SUB : result = difference;
    ALU_SLT : result = { {(N-1){1'b0}}, a < b };
    ALU_SLTU : result = { {(N-1){1'b0}}, unsigned_a < unsigned_b};
    default : result = 0;
  endcase

  equal = (a == b);
  zero = (result == {{N{1'b0}}});
  case (control) 
    ALU_SLTU, ALU_SLT, ALU_SUB: begin
      overflow = (a[N-1] != b[N-1]) && (a[N-1] != difference[N-1]); 
    end
    ALU_ADD : begin
      overflow = (a[N-1] == b[N-1]) && (a[N-1] != sum[N-1]);
    end
    default: overflow = 0;
  endcase
end

endmodule
