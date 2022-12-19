`timescale 1ns/1ps
`default_nettype none

`include "memmap.sv"

module mmu(
  clk, rst, 
  core_addr, core_rd_data, core_wr_ena, core_wr_data,
  leds, rgb,
  interface_mode, backlight, display_rstb, data_commandb,
  display_csb, spi_mosi, spi_miso, spi_clk,
  gpio_pins
);

parameter INST_L = `INST_L_WORDS;
parameter DATA_L = `DATA_L_WORDS;
parameter VRAM_UPSCALE = 1;
localparam VRAM_L = 320*240/VRAM_UPSCALE/VRAM_UPSCALE;

parameter INIT_INST = "mem/zeros.memh";
parameter INIT_DATA = "mem/zeros.memh";
parameter INIT_VRAM = "mem/zeros.memh";
parameter GPIO_PINS = 32;
initial if (GPIO_PINS > 32) begin 
  $display("Error: GPIO_PINS must be less than 32, is %d.", GPIO_PINS);
  $finish;
end

parameter CLK_HZ=12_000_000;

genvar i;

input wire  clk, rst;
input wire core_wr_ena;
input wire [31:0] core_wr_data, core_addr;
output logic [31:0] core_rd_data;
// LEDs
output wire [1:0] leds;
output wire [2:0] rgb;
// Display driver signals
output wire [3:0] interface_mode;
output wire backlight, display_rstb, data_commandb;
output wire display_csb, spi_clk, spi_mosi;
input wire spi_miso;
inout wire [GPIO_PINS-1:0] gpio_pins;
// GPIO Pins
wire [GPIO_PINS-1:0] mmr_gpio_mode, mmr_gpio_state;

// Muxing/Decoding between different memory banks.
wire [31:0] inst_rd_data, data_rd_data;
logic [31:0] mmrs_rd_data;
wire [31:0] periph_vram_addr;
wire [15:0] core_vram_rd_data, periph_vram_rd_data;

// Decoder that enables the different memory banks.
logic [MMU_BANK_DECODER_SIZE-1:0] bank_access, bank_wr_decoder;
generate for (i=0; i < MMU_BANK_DECODER_SIZE; i++) begin : MMU_BANK_WR_DECODER
  always_comb begin
      bank_access[i] = (core_addr[31:28] == i[3:0]);
      bank_wr_decoder[i] =  bank_access[i] & core_wr_ena;
  end
end endgenerate

// Mux that pulls data from the correct memory bank.
// Mux that pulls data from the correct memory bank.
always_comb begin: MMU_BANK_DATA_MUX
  case(core_addr[31:28])
    MMU_BANK_INST: core_rd_data = inst_rd_data;
    MMU_BANK_MMRS: core_rd_data = mmrs_rd_data;
    MMU_BANK_VRAM: core_rd_data = {16'd0, core_vram_rd_data};
    MMU_BANK_DATA: core_rd_data = data_rd_data;
    default      : core_rd_data = 0;
  endcase
end

// Instruction Memory
distributed_ram #(.W(32), .L(INST_L), .INIT(INIT_INST)) INST_RAM(
  .clk(clk), .addr(core_addr[$clog2(INST_L)+1:2]), .rd_data(inst_rd_data), 
  .wr_ena(bank_wr_decoder[MMU_BANK_INST]), .wr_data(core_wr_data)
);

// Data Memory
distributed_ram #(.W(32), .L(DATA_L), .INIT(INIT_DATA)) DATA_RAM(
  .clk(clk), .addr(core_addr[$clog2(DATA_L)+1:2]), .rd_data(data_rd_data),
  .wr_ena(bank_wr_decoder[MMU_BANK_DATA]), .wr_data(core_wr_data)
);

// Video Memory - DMA (Direct Memory Access) Example
// Note that this breaks spec a little by not being byte aligned. 
// TODO(avinash) - check to see if there's a way to better byte align this.
// dual_port_distributed_ram 
dual_port_ram // TODO(avinash) - double check that I don't need this to fit on a 15T!
#(.W(16), .L(VRAM_L), .INIT(INIT_VRAM)) VRAM(
  .clk(clk),
  .addr0(core_addr[$clog2(VRAM_L)-1:0]), 
  .wr_ena0(bank_wr_decoder[MMU_BANK_VRAM]), 
  .wr_data0(core_wr_data[15:0]), .rd_data0(core_vram_rd_data[15:0]),
  .wr_ena1(1'b0), .addr1(periph_vram_addr[$clog2(VRAM_L)-1:0]),
  .wr_data1(16'd0), .rd_data1(periph_vram_rd_data)
);

// Display Peripheral
assign backlight = 1;
ili9341_display_peripheral #(
  .DISPLAY_WIDTH(240),
  .DISPLAY_HEIGHT(320),
  .VRAM_START_ADDRESS(MMU_BANK_VRAM),
  .VRAM_UPSCALE(VRAM_UPSCALE)
) ILI9341 (
  .clk(clk), .rst(rst), .ena(1'b1), 
  .display_rstb(display_rstb), .interface_mode(interface_mode),
  .spi_csb(display_csb), .spi_clk(spi_clk), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
  .data_commandb(data_commandb),
  .vram_rd_addr(periph_vram_addr),
  .vram_rd_data(periph_vram_rd_data[15:0]),
  .enable_test_pattern(1'b0) // TODO(avinash) have an MMR control this.
);

// Pulse generators for timing signals.
wire pulse_1kHz, pulse_10kHz;
pulse_generator #(.N($clog2(CLK_HZ/10000))) PULSE_10KHZ (
  .clk(clk), .rst(rst), .ena(1'b1), .out(pulse_10kHz),
  .ticks(CLK_HZ/10000)
);
pulse_generator #(.N($clog2(CLK_HZ/1000))) PULSE_1KHZ (
  .clk(clk), .rst(rst), .ena(1'b1), .out(pulse_1kHz),
  .ticks(CLK_HZ/1000)
);

//////////////////////////////////////////////////////////////////////////////
// MMR Specific Logic                                                       //
//////////////////////////////////////////////////////////////////////////////
wire [31:0] mmr_led;
logic [3:0] led_pwm0,led_pwm1;
logic [8:0] led_r_pwm, led_g_pwm, led_b_pwm;
logic [31:0] timer_1kHz, timer_10kHz;
logic [MMR_MAX_INDEX-1:0] mmr_access, mmr_wr_decoder;
logic [3:0] mmr_index;
always_comb mmr_index = core_addr[$clog2(MMR_MAX_INDEX)+2:2];
generate for (i = 0; i < MMR_MAX_INDEX; i++) begin : MMR_bank_wr_decoder
  always_comb begin
    mmr_access[i] = mmr_index == i[3:0] & bank_access[MMU_BANK_MMRS];
    mmr_wr_decoder[i] = mmr_access[i] & core_wr_ena;
  end
end endgenerate

// MMR Specific Encode/Decode

always_comb begin : mmr_read_data_mux
  case(mmr_index)
    MMR_INDEX_LEDS: mmrs_rd_data = mmr_led;
    MMR_INDEX_GPIO_MODE: mmrs_rd_data = {{(32-GPIO_PINS) {1'b0}}, mmr_gpio_mode};
    MMR_INDEX_GPIO_STATE: mmrs_rd_data = {{(32-GPIO_PINS) {1'b0}}, gpio_pins};
    MMR_INDEX_TIMER_1kHZ: mmrs_rd_data = timer_1kHz;
    MMR_INDEX_TIMER_10kHz: mmrs_rd_data = timer_10kHz;
    default: mmrs_rd_data = 0;
  endcase
end

// An LED MMR that lets us control duty cycle.
register #(.N(32)) MMR_LED(.clk(clk), .rst(rst), 
  .ena(mmr_wr_decoder[MMR_INDEX_LEDS]), .d(core_wr_data), .q(mmr_led));

pwm #(.N(4)) PWM_LED0 (
  .clk(clk), .rst(rst), .ena(1'b1), .step(1'b1), .duty(led_pwm0),
  .out(leds[0])
);
pwm #(.N(4)) PWM_LED1 (
  .clk(clk), .rst(rst), .ena(1'b1), .step(1'b1), .duty(led_pwm1),
  .out(leds[1])
);
pwm #(.N(9)) PWM_LED_R (
  .clk(clk), .rst(rst), .ena(1'b1), .step(1'b1), .duty(led_r_pwm),
  .out(rgb[0])
);
pwm #(.N(9)) PWM_LED_G (
  .clk(clk), .rst(rst), .ena(1'b1), .step(1'b1), .duty(led_g_pwm),
  .out(rgb[1])
);
pwm #(.N(9)) PWM_LED_B (
  .clk(clk), .rst(rst), .ena(1'b1), .step(1'b1), .duty(led_b_pwm),
  .out(rgb[2])
);

// GPIO Peripheral MMRs
register #(.N(GPIO_PINS)) MMR_GPIO_STATE(
  .clk(clk), .rst(rst), .ena(mmr_wr_decoder[MMR_INDEX_GPIO_STATE]),
  .d(core_wr_data[GPIO_PINS-1:0]), .q(mmr_gpio_state)
);
register #(.N(GPIO_PINS)) MMR_GPIO_MODE(
  .clk(clk), .rst(rst), .ena(mmr_wr_decoder[MMR_INDEX_GPIO_MODE]),
  .d(core_wr_data[GPIO_PINS-1:0]), .q(mmr_gpio_mode)
);

generate for(i = 0; i < GPIO_PINS; i++) begin : GPIO_TRISTATES
  assign gpio_pins[i] = mmr_gpio_mode[i] ? mmr_gpio_state[i] : 1'bz;
end endgenerate


always_comb begin : led_mmr_decode
  led_pwm0 = mmr_led[31:28];
  led_pwm1 = mmr_led[27:24];
  // Hidden bonus question - why did I do this for the RGB LED?
  led_r_pwm = 9'h1ff - mmr_led[23:16];
  led_g_pwm = 9'h1ff - mmr_led[15:8];
  led_b_pwm = 9'h1ff - mmr_led[7:0];
end

// A read-only timer MMR peripheral.
always_ff @(posedge clk) begin
  if(rst) begin
    timer_1kHz <= 0;
    timer_10kHz <= 0;
  end else begin
`ifdef SIMULATION
    // Run timers much faster if simulating to make our code run in finite time.
    timer_10kHz <= timer_10kHz + 1; // Increment every clock.
    timer_1kHz <= (timer_10kHz[0]) ? timer_1kHz + 1 : timer_1kHz; // Increment every other clock.
`else 
    if(pulse_1kHz) timer_1kHz <= timer_1kHz + 1;
    if(pulse_10kHz) timer_10kHz <= timer_10kHz + 1;
`endif // SIMULATION
  end
end

task dump_memory(string prefix);
  INST_RAM.dump_memory({prefix, "_inst.out"});
  DATA_RAM.dump_memory({prefix, "_data.out"});
  VRAM.dump_memory({prefix, "_vram.out"});
endtask

endmodule
