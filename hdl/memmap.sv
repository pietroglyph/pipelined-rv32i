`ifndef MEMMAP_H
`define MEMMAP_H
`define INST_L_WORDS 1024
`define DATA_L_WORDS 1024
`define MMU_DECODER_SIZE 4

// Use the top 4-bits of the address space to decode to different memory
// banks. This allows for simple (low delay) comb. logic to decide which
// memory subsystem the address applies to (see mmu.sv).

typedef enum logic [3:0] {
  MMU_BANK_INST = 4'b0000, 
  MMU_BANK_MMRS = 4'b0001, 
  MMU_BANK_VRAM = 4'b0010, 
  MMU_BANK_DATA = 4'b0011,
  MMU_BANK_DECODER_SIZE /* Adding this at the end of the enum makes sizing busses easier. */
} mmu_bank_t;


// Memory Mapped Registers
typedef enum logic [3:0]  {
  MMR_INDEX_LEDS,          // offset: 0x00 // TODO(avinash) - test
  MMR_INDEX_GPIO_MODE,     // offset: 0x04 // TODO(avinash) - test
  MMR_INDEX_GPIO_STATE,    // offset: 0x08 // TODO(avinash) - test
  MMR_INDEX_TIMER_1kHZ,    // offset: 0x0c // TODO(avinash) - test
  MMR_INDEX_TIMER_10kHz,   // offset: 0x10 // TODO(avinash) - test
  MMR_MAX_INDEX
} mmr_index_t;


function logic [31:0] mmr_address_from_index(mmr_index_t index);
  mmr_address_from_index = {MMU_BANK_MMRS, 22'b0, index, 2'b0};
endfunction

`endif // MEMMAP_H
