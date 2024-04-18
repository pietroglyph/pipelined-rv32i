`timescale 1ns/1ps
`default_nettype none

module rv32i_system(
  sysclk, buttons, leds, rgb,
  interface_mode, backlight, display_rstb, data_commandb,
  display_csb, spi_mosi, spi_miso, spi_clk,
  gpio
);


input wire sysclk;
input wire [1:0] buttons;
output wire [1:0] leds;
output wire [2:0] rgb;
wire rst; assign rst = buttons[0];
wire clk;
parameter SYS_CLK_HZ = 12_000_000.0; // aka ticks per second
parameter SYS_CLK_PERIOD_NS = (1_000_000_000.0/SYS_CLK_HZ);
parameter CLK_HZ = 10*SYS_CLK_HZ; // aka ticks per second
parameter CLK_PERIOD_NS = (1_000_000_000.0/CLK_HZ); // Approximation.
parameter GPIO_PINS = 32;

// Display driver signals
output wire [3:0] interface_mode;
output wire backlight, display_rstb, data_commandb;
output wire display_csb, spi_clk, spi_mosi;
input wire spi_miso;

// GPIO
inout wire [7:0] gpio;

// Vivado can simulate MMCME2_BASE but Icarus can't
`ifdef __ICARUS__
assign clk = sysclk;
`else 
wire clk_feedback;

MMCME2_BASE #(
  .BANDWIDTH("OPTIMIZED"), .CLKIN1_PERIOD(SYS_CLK_PERIOD_NS), .CLKOUT0_DUTY_CYCLE(0.5),.CLKOUT0_PHASE(0.0), .STARTUP_WAIT("FALSE"), // Delays DONE until MMCM is locked (FALSE, TRUE) // Don't mess with anything on  this line!
  .CLKFBOUT_MULT_F(64.0), //2.0 to 64.0 in increments of 0.125
  .CLKOUT0_DIVIDE_F(12.5), // Divide amount for CLKOUT0 (1.000-128.000).
  .DIVCLK_DIVIDE(1) // Master division value (1-106)
)
MMCME2_BASE_inst (
  .CLKOUT0(clk),
  .CLKIN1(sysclk),
  .PWRDWN(0),
  .RST(buttons[1]),
  .CLKFBOUT(clk_feedback),
  .CLKFBIN(clk_feedback)
);

`endif // SIMULATION

wire core_mem_wr_ena, is_inst_addr;
wire [31:0] core_mem_addr, core_mem_wr_data, core_mem_rd_data, inst_mem_addr, inst_mem_rd_data, core_pc;
wire [31:0] instructions_completed;

rv32i_pipelined_core CORE (
  .clk(clk), .rst(rst), .ena(1'b1),
  .inst_mem_addr(inst_mem_addr), .inst_mem_rd_data(inst_mem_rd_data),
  .data_mem_addr(core_mem_addr), .data_mem_wr_data(core_mem_wr_data), .data_mem_rd_data(core_mem_rd_data), .data_mem_wr_ena(core_mem_wr_ena),
  .is_inst_addr(is_inst_addr),
  .PC(core_pc), .instructions_completed(instructions_completed)
);

// Memory Management Unit
`ifndef INITIAL_INST_MEM
initial begin 
  $display("Initial instruction memory not defined, not running simulation.");
  $finish;
end
`define INITIAL_INST_MEM "mem/zeros.memh"
`endif // INITIAL_INST_MEM

mmu #(
  .INIT_INST(`INITIAL_INST_MEM),
  .GPIO_PINS(8)
) MMU (
  .clk(clk), .rst(rst), .core_addr(core_mem_addr),
  .core_wr_ena(core_mem_wr_ena), .core_wr_data(core_mem_wr_data),
  .core_rd_data(core_mem_rd_data),
  .inst_addr(inst_mem_addr), .inst_rd_data(inst_mem_rd_data),
  .bank_test_addr(core_mem_addr), .is_inst_addr(is_inst_addr),
  .leds(leds), .rgb(rgb),
  .display_rstb(display_rstb), .interface_mode(interface_mode), .backlight(backlight),
  .display_csb(display_csb), .spi_clk(spi_clk), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
  .data_commandb(data_commandb),
  .gpio_pins(gpio)
);

endmodule
