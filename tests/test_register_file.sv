`default_nettype none
`timescale 1ns/1ps

module test_register_file;

logic clk;

// Write channel
logic wr_ena;
logic [4:0] wr_addr;
logic [31:0] wr_data;

// Two read channels
logic [4:0] rd_addr0, rd_addr1;
wire [31:0] rd_data0, rd_data1;

register_file UUT(
  .clk(clk), .wr_ena(wr_ena), .wr_addr(wr_addr), .wr_data(wr_data),
  .rd_addr0(rd_addr0), .rd_addr1(rd_addr1),
  .rd_data0(rd_data0), .rd_data1(rd_data1)
);

initial begin
  // for all inputs: set to default value
  clk = 0;
  wr_ena = 0;
  wr_addr = 0;
  wr_data = 0;
  rd_addr0 = 0;
  rd_addr1 = 0;

  $dumpfile("register_file.fst");
  $dumpvars(0, UUT);

  /*
  Test methodology: write unique values to x01 to x31, then read them back out.
  Trying to pick unique values that exercise a lot of bits, so trying negative numbers is good.
  Specifically trying to write to the zero register to make sure that doesn't change.
  */
  for(int i = 0; i < 32; i = i + 1) begin
    @(negedge clk);
    wr_ena = 1;
    wr_addr = i[4:0];
    wr_data = -1*(i + 1);
    @(posedge clk);
  end
  wr_ena = 0;

  /*
  Then we'll read out the values, and to prove that both read channels work we'll 
  read them out in opposite orders
  */

  for(int i = 0; i < 32; i = i + 1) begin
    @(negedge clk);
    rd_addr0 = i;
    rd_addr1 = 31-i;
    @(posedge clk);
    $display("@%t: read0[%02d] = %x, read1[%02d] = %x", $time, rd_addr0, rd_data0, rd_addr1, rd_data1);
  end


  // Zero check
  @(negedge clk);
  rd_addr0 = 0; rd_addr1 = 0; // have both read channels read from x00, which should be zero!
  @(posedge clk);
  $display("@%t: read0[%02d] = %x, read1[%02d] = %x", $time, rd_addr0, rd_data0, rd_addr1, rd_data1);
  if(rd_data0 != 32'd0 || rd_data1 !=32'd0) begin
    $display("Crical error, reading zero didn't result in zero! Quitting");
    $finish;
  end

  $finish;

end

always #5 clk = ~clk; // clock generator

endmodule