`default_nettype none
`timescale 1ns/1ps

module register_file(
  clk, // wtf, no rst?  // avi - makes debugging register writes easier
  wr_ena, wr_addr, wr_data,
  rd_addr0, rd_data0,
  rd_addr1, rd_data1
);
// Not parametrizing, these widths are defined by the RISC-V Spec!

input wire clk;

// Write channel
input wire wr_ena;
input wire [4:0] wr_addr;
input wire [31:0] wr_data;

// Two read channels
input wire [4:0] rd_addr0, rd_addr1;
output logic [31:0] rd_data0, rd_data1;

logic [31:0] x00; 
always_comb begin 
  x00 = 32'd0; // ties x00 to ground
end

// DON'T DO THIS:
// logic [31:0] register_file_registers [31:0]
// CAN'T: because that's a RAM. Works in simulation, not synthesis.

// Use python to instantiate a bunch of register modules with the right connections.

// python: print(",".join(["x%02d"%i for i in range(0,32)]))
wire signed [31:0] x01,x02,x03,x04,x05,x06,x07,x08,x09,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31; // Keep these signed for pretty printing when debugging.
logic [31:0] wr_enas; // separate enable for each register


// Need 2x read muxes

// behavioural option:
always_comb begin : read_mux0
  case(rd_addr0)
    // python:  print("\n".join(["5'd%02d : rd_data0 = x%02d;"%(i,i) for i in range(0,32)])
    5'd00 : rd_data0 = x00;
    5'd01 : rd_data0 = x01;
    5'd02 : rd_data0 = x02;
    5'd03 : rd_data0 = x03;
    5'd04 : rd_data0 = x04;
    5'd05 : rd_data0 = x05;
    5'd06 : rd_data0 = x06;
    5'd07 : rd_data0 = x07;
    5'd08 : rd_data0 = x08;
    5'd09 : rd_data0 = x09;
    5'd10 : rd_data0 = x10;
    5'd11 : rd_data0 = x11;
    5'd12 : rd_data0 = x12;
    5'd13 : rd_data0 = x13;
    5'd14 : rd_data0 = x14;
    5'd15 : rd_data0 = x15;
    5'd16 : rd_data0 = x16;
    5'd17 : rd_data0 = x17;
    5'd18 : rd_data0 = x18;
    5'd19 : rd_data0 = x19;
    5'd20 : rd_data0 = x20;
    5'd21 : rd_data0 = x21;
    5'd22 : rd_data0 = x22;
    5'd23 : rd_data0 = x23;
    5'd24 : rd_data0 = x24;
    5'd25 : rd_data0 = x25;
    5'd26 : rd_data0 = x26;
    5'd27 : rd_data0 = x27;
    5'd28 : rd_data0 = x28;
    5'd29 : rd_data0 = x29;
    5'd30 : rd_data0 = x30;
    5'd31 : rd_data0 = x31;
  endcase
end

always_comb begin : read_mux1
  case(rd_addr1)
     // python: print("\n".join(["5'd%02d : rd_data1 = x%02d;"%(i,i) for i in range(0,32)]))
    5'd00 : rd_data1 = x00;
    5'd01 : rd_data1 = x01;
    5'd02 : rd_data1 = x02;
    5'd03 : rd_data1 = x03;
    5'd04 : rd_data1 = x04;
    5'd05 : rd_data1 = x05;
    5'd06 : rd_data1 = x06;
    5'd07 : rd_data1 = x07;
    5'd08 : rd_data1 = x08;
    5'd09 : rd_data1 = x09;
    5'd10 : rd_data1 = x10;
    5'd11 : rd_data1 = x11;
    5'd12 : rd_data1 = x12;
    5'd13 : rd_data1 = x13;
    5'd14 : rd_data1 = x14;
    5'd15 : rd_data1 = x15;
    5'd16 : rd_data1 = x16;
    5'd17 : rd_data1 = x17;
    5'd18 : rd_data1 = x18;
    5'd19 : rd_data1 = x19;
    5'd20 : rd_data1 = x20;
    5'd21 : rd_data1 = x21;
    5'd22 : rd_data1 = x22;
    5'd23 : rd_data1 = x23;
    5'd24 : rd_data1 = x24;
    5'd25 : rd_data1 = x25;
    5'd26 : rd_data1 = x26;
    5'd27 : rd_data1 = x27;
    5'd28 : rd_data1 = x28;
    5'd29 : rd_data1 = x29;
    5'd30 : rd_data1 = x30;
    5'd31 : rd_data1 = x31;
  endcase
end

// Need 1x write enable decoder 

`ifdef TRUTH_TABLE_DECODER

// Here's a truth table approach
always_comb begin : write_enable_decoder_truth_table
  if(wr_ena) begin
    case (wr_addr)
      // Very lazy pythonic way of making a decoder truth table
      // python: print("\n".join(["5'd%02d : wr_enas = 32'd%d;"%(i, 1 << i) for i in range(0,32)]))
      5'd00 : wr_enas = 32'd1;
      5'd01 : wr_enas = 32'd2;
      5'd02 : wr_enas = 32'd4;
      5'd03 : wr_enas = 32'd8;
      5'd04 : wr_enas = 32'd16;
      5'd05 : wr_enas = 32'd32;
      5'd06 : wr_enas = 32'd64;
      5'd07 : wr_enas = 32'd128;
      5'd08 : wr_enas = 32'd256;
      5'd09 : wr_enas = 32'd512;
      5'd10 : wr_enas = 32'd1024;
      5'd11 : wr_enas = 32'd2048;
      5'd12 : wr_enas = 32'd4096;
      5'd13 : wr_enas = 32'd8192;
      5'd14 : wr_enas = 32'd16384;
      5'd15 : wr_enas = 32'd32768;
      5'd16 : wr_enas = 32'd65536;
      5'd17 : wr_enas = 32'd131072;
      5'd18 : wr_enas = 32'd262144;
      5'd19 : wr_enas = 32'd524288;
      5'd20 : wr_enas = 32'd1048576;
      5'd21 : wr_enas = 32'd2097152;
      5'd22 : wr_enas = 32'd4194304;
      5'd23 : wr_enas = 32'd8388608;
      5'd24 : wr_enas = 32'd16777216;
      5'd25 : wr_enas = 32'd33554432;
      5'd26 : wr_enas = 32'd67108864;
      5'd27 : wr_enas = 32'd134217728;
      5'd28 : wr_enas = 32'd268435456;
      5'd29 : wr_enas = 32'd536870912;
      5'd30 : wr_enas = 32'd1073741824;
      5'd31 : wr_enas = 32'd2147483648;
    endcase
  end
  else begin 
    wr_enas = 32'b0;
  end
end
`endif // TRUTH_TABLE_DECODER

`ifdef BEHAVIOURAL_SHIFTER_DECODER
// This is a more typical behavioural decoder implementation, but the shift operator makes it non-obvious which way it will be synthesized (a full shifter is usually bigger than a 5:32 decoder)
always_comb begin : write_enable_decoder_shifter
  wr_enas = wr_ena ? (32'b1 << wr_addr) : 32'b0;
end
`endif // BEHAVIOURAL_SHIFTER_DECODER

`define STRUCTURAL_DECODER
`ifdef STRUCTURAL_DECODER
decoder_5_to_32 WR_ENA_DECODER(.ena(wr_ena), .in(wr_addr), .out(wr_enas));
`endif //STRUCTURAL_DECODER




// instantiate registers:
// python: print("\n".join(["register #(.N(32)) r_x%02d(.clk(clk), .rst(1'b0), .q(x%02d), .d(wr_data), .ena(wr_enas[%02d]));"%(i,i,i) for i in range(1,32)]))
register #(.N(32)) r_x01(.clk(clk), .rst(1'b0), .q(x01), .d(wr_data), .ena(wr_enas[01]));
register #(.N(32)) r_x02(.clk(clk), .rst(1'b0), .q(x02), .d(wr_data), .ena(wr_enas[02]));
register #(.N(32)) r_x03(.clk(clk), .rst(1'b0), .q(x03), .d(wr_data), .ena(wr_enas[03]));
register #(.N(32)) r_x04(.clk(clk), .rst(1'b0), .q(x04), .d(wr_data), .ena(wr_enas[04]));
register #(.N(32)) r_x05(.clk(clk), .rst(1'b0), .q(x05), .d(wr_data), .ena(wr_enas[05]));
register #(.N(32)) r_x06(.clk(clk), .rst(1'b0), .q(x06), .d(wr_data), .ena(wr_enas[06]));
register #(.N(32)) r_x07(.clk(clk), .rst(1'b0), .q(x07), .d(wr_data), .ena(wr_enas[07]));
register #(.N(32)) r_x08(.clk(clk), .rst(1'b0), .q(x08), .d(wr_data), .ena(wr_enas[08]));
register #(.N(32)) r_x09(.clk(clk), .rst(1'b0), .q(x09), .d(wr_data), .ena(wr_enas[09]));
register #(.N(32)) r_x10(.clk(clk), .rst(1'b0), .q(x10), .d(wr_data), .ena(wr_enas[10]));
register #(.N(32)) r_x11(.clk(clk), .rst(1'b0), .q(x11), .d(wr_data), .ena(wr_enas[11]));
register #(.N(32)) r_x12(.clk(clk), .rst(1'b0), .q(x12), .d(wr_data), .ena(wr_enas[12]));
register #(.N(32)) r_x13(.clk(clk), .rst(1'b0), .q(x13), .d(wr_data), .ena(wr_enas[13]));
register #(.N(32)) r_x14(.clk(clk), .rst(1'b0), .q(x14), .d(wr_data), .ena(wr_enas[14]));
register #(.N(32)) r_x15(.clk(clk), .rst(1'b0), .q(x15), .d(wr_data), .ena(wr_enas[15]));
register #(.N(32)) r_x16(.clk(clk), .rst(1'b0), .q(x16), .d(wr_data), .ena(wr_enas[16]));
register #(.N(32)) r_x17(.clk(clk), .rst(1'b0), .q(x17), .d(wr_data), .ena(wr_enas[17]));
register #(.N(32)) r_x18(.clk(clk), .rst(1'b0), .q(x18), .d(wr_data), .ena(wr_enas[18]));
register #(.N(32)) r_x19(.clk(clk), .rst(1'b0), .q(x19), .d(wr_data), .ena(wr_enas[19]));
register #(.N(32)) r_x20(.clk(clk), .rst(1'b0), .q(x20), .d(wr_data), .ena(wr_enas[20]));
register #(.N(32)) r_x21(.clk(clk), .rst(1'b0), .q(x21), .d(wr_data), .ena(wr_enas[21]));
register #(.N(32)) r_x22(.clk(clk), .rst(1'b0), .q(x22), .d(wr_data), .ena(wr_enas[22]));
register #(.N(32)) r_x23(.clk(clk), .rst(1'b0), .q(x23), .d(wr_data), .ena(wr_enas[23]));
register #(.N(32)) r_x24(.clk(clk), .rst(1'b0), .q(x24), .d(wr_data), .ena(wr_enas[24]));
register #(.N(32)) r_x25(.clk(clk), .rst(1'b0), .q(x25), .d(wr_data), .ena(wr_enas[25]));
register #(.N(32)) r_x26(.clk(clk), .rst(1'b0), .q(x26), .d(wr_data), .ena(wr_enas[26]));
register #(.N(32)) r_x27(.clk(clk), .rst(1'b0), .q(x27), .d(wr_data), .ena(wr_enas[27]));
register #(.N(32)) r_x28(.clk(clk), .rst(1'b0), .q(x28), .d(wr_data), .ena(wr_enas[28]));
register #(.N(32)) r_x29(.clk(clk), .rst(1'b0), .q(x29), .d(wr_data), .ena(wr_enas[29]));
register #(.N(32)) r_x30(.clk(clk), .rst(1'b0), .q(x30), .d(wr_data), .ena(wr_enas[30]));
register #(.N(32)) r_x31(.clk(clk), .rst(1'b0), .q(x31), .d(wr_data), .ena(wr_enas[31]));

// Aliases (helpful for debugging assembly);
`ifdef SIMULATION
logic [31:0] ra, sp, gp, tp, t0, t1, t2, s0, fp, s1, a0, a1, a2, a3, a4, a5, 
  a6, a7, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, t3, t4, t5, t6;
always_comb begin : REGISTER_ALIASES
  ra = x01; // Return Address
  sp = x02; // Stack Pointer
  gp = x03; // Global Pointer
  tp = x04; // Thread Pointer
  fp = x08; // Frame Pointer
  s0 = x08; // Saved Registers - must be preserved by called functions.
  s1 = x09; 
  s2 = x18;
  s3 = x19;
  s4 = x20;
  s5 = x21;
  s6 = x22;
  s7 = x23;
  s8 = x24;
  s9 = x25;
  s10 = x26;
  s11 = x27;
  t0 = x05; // Temporary values (can be changed by called functions).
  t1 = x06;
  t2 = x07;
  t3 = x28;
  t4 = x29;
  t5 = x30;
  t6 = x31;
  a0 = x10;
  a1 = x11;
  a2 = x12;
  a3 = x13;
  a4 = x14;
  a5 = x15;
  a6 = x16;
  a7 = x17;
end

task print_state;
  $display("|---------------------------------------|");
  $display("| Register File State                   |");
  $display("|---------------------------------------|");
  $display("| %12s = 0x%8h (%10d)|", "x00, zero", x00, x00);
  $display("| %12s = 0x%8h (%10d)|", "x01, ra", x01, x01);
  $display("| %12s = 0x%8h (%10d)|", "x02, sp", x02, x02);
  $display("| %12s = 0x%8h (%10d)|", "x03, gp", x03, x03);
  $display("| %12s = 0x%8h (%10d)|", "x04, tp", x04, x04);
  $display("| %12s = 0x%8h (%10d)|", "x05, t0", x05, x05);
  $display("| %12s = 0x%8h (%10d)|", "x06, t1", x06, x06);
  $display("| %12s = 0x%8h (%10d)|", "x07, t2", x07, x07);
  $display("| %12s = 0x%8h (%10d)|", "x08, s0", x08, x08);
  $display("| %12s = 0x%8h (%10d)|", "x09, s1", x09, x09);
  $display("| %12s = 0x%8h (%10d)|", "x10, a0", x10, x10);
  $display("| %12s = 0x%8h (%10d)|", "x11, a1", x11, x11);
  $display("| %12s = 0x%8h (%10d)|", "x12, a2", x12, x12);
  $display("| %12s = 0x%8h (%10d)|", "x13, a3", x13, x13);
  $display("| %12s = 0x%8h (%10d)|", "x14, a4", x14, x14);
  $display("| %12s = 0x%8h (%10d)|", "x15, a5", x15, x15);
  $display("| %12s = 0x%8h (%10d)|", "x16, a6", x16, x16);
  $display("| %12s = 0x%8h (%10d)|", "x17, a7", x17, x17);
  $display("| %12s = 0x%8h (%10d)|", "x18, s2", x18, x18); 
  $display("| %12s = 0x%8h (%10d)|", "x19, s3", x19, x19); 
  $display("| %12s = 0x%8h (%10d)|", "x20, s4", x20, x20); 
  $display("| %12s = 0x%8h (%10d)|", "x21, s5", x21, x21); 
  $display("| %12s = 0x%8h (%10d)|", "x22, s6", x22, x22); 
  $display("| %12s = 0x%8h (%10d)|", "x23, s7", x23, x23); 
  $display("| %12s = 0x%8h (%10d)|", "x24, s8", x24, x24); 
  $display("| %12s = 0x%8h (%10d)|", "x25, s9", x25, x25); 
  $display("| %12s = 0x%8h (%10d)|", "x26, s10", x26, x26); 
  $display("| %12s = 0x%8h (%10d)|", "x27, s11", x27, x27); 
  $display("| %12s = 0x%8h (%10d)|", "x28, t3", x28, x28); 
  $display("| %12s = 0x%8h (%10d)|", "x29, t4", x29, x29); 
  $display("| %12s = 0x%8h (%10d)|", "x30, t5", x30, x30); 
  $display("| %12s = 0x%8h (%10d)|", "x31, t6", x31, x31); 
  $display("|---------------------------------------|");
endtask

`endif // SIMULATION
endmodule