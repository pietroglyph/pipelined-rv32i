// Based on UG901 - The Vivado Synthesis Guide

`timescale 1ns/1ps
`default_nettype none

module block_rom(clk, addr, data);

parameter W = 8; // Width of each row of  the memory
parameter L = 32; // Length fo the memory
parameter INIT = "zeros.memh";

input wire clk;
input wire [$clog2(L)-1:0] addr;
output logic [W-1:0] data;

logic [W-1:0] rom [0:L-1];
initial begin
  $display("Initializing block rom from file %s.", INIT);
  $readmemh(INIT, rom); // Initializes the ROM with the values in the init file.
end

always_ff @(posedge clk) begin : synthesizable_rom
  data <= rom[addr];
end

task dump_memory(string file);
  $writememh(file, rom);
endtask


endmodule
