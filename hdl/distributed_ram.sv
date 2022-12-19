`timescale 1ns/1ps
`default_nettype none

// Based on UG901 - Vivado Synthesis Guide

// Single-Port RAM with Asynchronous Read (Distributed RAM)
/* Avi's N.B:
Instead of using real RAMs built into the FPGA you can also
leverage the LUTs as RAM. It's not always the best option since
it uses up some of your logic area, but it does allow for
asynchronous (combinational) reading. This makes it execellent
for two (or more) modules accessing the same memory space, or
for any memory accesses that you want to be quick.
*/

module distributed_ram(clk, wr_ena, addr, wr_data, rd_data);

parameter W = 32;
parameter L = 128;
parameter INIT = "zeros.memh";
input wire clk;
input wire wr_ena;
input wire [$clog2(L)-1:0] addr;
input wire [W-1:0] wr_data;
output wire [W-1:0] rd_data;

logic [W-1:0] ram [0:L-1];

initial begin
  $display("###########################################");
  $display("Initializing distributed ram from file %s.", INIT);
  $display("###########################################");
  $readmemh(INIT, ram);
end

always_ff @(posedge clk) begin
  if(wr_ena) ram[addr] <= wr_data;
end

// icarus verilog does not like always_comb in inferred memories
assign rd_data = ram[addr];


task dump_memory(string file);
  $writememh(file, ram);
endtask

endmodule