`ifndef RV32I_DEFINES
`define RV32I_DEFINES

// addi x0, x0, 0 aka NOP (no operation)
`define RV32_NOP 32'h0000_0013 

typedef enum logic [6:0] {
  OP_LTYPE = 7'b0000011,
  OP_ITYPE = 7'b0010011,
  OP_AUIPC = 7'b0010111,
  OP_STYPE = 7'b0100011,
  OP_RTYPE = 7'b0110011,
  OP_LUI   = 7'b0110111,
  OP_BTYPE = 7'b1100011,
  OP_JALR  = 7'b1100111,
  OP_JAL   = 7'b1101111
} op_type_t;

typedef enum logic [2:0] {
  FUNCT3_LOAD_LB  = 3'b000,
  FUNCT3_LOAD_LH  = 3'b001,
  FUNCT3_LOAD_LW  = 3'b010,
  FUNCT3_LOAD_LBU = 3'b100,
  FUNCT3_LOAD_LHU = 3'b101
} funct3_load_t;

typedef enum logic [2:0] {
  FUNCT3_ADD = 3'b000,
  FUNCT3_SLL = 3'b001,
  FUNCT3_SLT = 3'b010,
  FUNCT3_SLTU = 3'b011,
  FUNCT3_XOR = 3'b100,
  FUNCT3_SHIFT_RIGHT = 3'b101, // Needs a funct7 bit to determine!
  FUNCT3_OR = 3'b110,
  FUNCT3_AND = 3'b111
} funct3_ritype_t;

typedef enum logic [2:0] {
  FUNCT3_BEQ  = 3'b000,
  FUNCT3_BNE  = 3'b001,
  FUNCT3_BLT  = 3'b100,
  FUNCT3_BGE  = 3'b101,
  FUNCT3_BLTU = 3'b110,
  FUNCT3_BGEU = 3'b111
} funct3_btype_t;

function string op_name(logic [6:0] op);
  case(op)
    OP_ITYPE  : op_name = " I-type ";
    OP_LTYPE  : op_name = " L-type ";
    OP_AUIPC  : op_name = " AUIPC  ";
    OP_STYPE  : op_name = " S-type ";
    OP_RTYPE  : op_name = " R-type ";
    OP_LUI    : op_name = " LUI    ";
    OP_BTYPE  : op_name = " B-type ";
    OP_JALR   : op_name = " JALR   ";
    OP_JAL    : op_name = " JAL    ";
    default   : op_name = " UNDEF  ";
  endcase
endfunction


`endif // RV32I_DEFINES
