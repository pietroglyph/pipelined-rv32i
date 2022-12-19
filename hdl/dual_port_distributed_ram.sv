`timescale 1ns/1ps
`default_nettype none

// Based on UG901 - Vivado Synthesis Guide

// Dual-Port RAM with Asynchronous Read (Distributed RAM)
/* Avi's N.B:
Instead of using real RAMs built into the FPGA you can also
leverage the LUTs as RAM. It's not always the best option since
it uses up some of your logic area, but it does allow for
asynchronous (combinational) reading. This makes it execellent
for two (or more) modules accessing the same memory space.
*/

module dual_port_distributed_ram(
  clk, 
  wr_ena0, addr0, wr_data0, rd_data0,
  wr_ena1, addr1, wr_data1, rd_data1
);
parameter W = 32;
parameter L = 128;
parameter INIT = "zeros.memh";
input wire clk;
input wire wr_ena0, wr_ena1;
input wire [$clog2(L)-1:0] addr0, addr1;
input wire [W-1:0] wr_data0, wr_data1;
output wire [W-1:0] rd_data0, rd_data1;

logic [W-1:0] ram [0:L-1];

initial begin
  $display("Initializing distributed ram from file %s.", INIT);
  $readmemh(INIT, ram);
end

always_ff @(posedge clk) begin
  if(wr_ena0) ram[addr0] <= wr_data0;
  if(wr_ena1) ram[addr0] <= wr_data1;
end

// icarus verilog doesn't like always_comb here.
assign rd_data0 = ram[addr0];
assign rd_data1 = ram[addr1];

task dump_memory(string file);
  $writememh(file, ram);
endtask

endmodule