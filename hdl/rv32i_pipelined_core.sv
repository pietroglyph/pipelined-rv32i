`timescale 1ns/1ps
`default_nettype none

`include "alu_types.sv"
`include "rv32i_defines.sv"

module rv32i_pipelined_core(
  clk, rst, ena,
  inst_mem_addr, inst_mem_rd_data, data_mem_addr, data_mem_wr_data, data_mem_rd_data, data_mem_wr_ena,
  is_inst_addr,
  PC, instructions_completed
);

parameter PC_START_ADDRESS=0;

typedef enum logic [1:0] {
	RESULT_ALU = 0,
	RESULT_MEM_RD,
	RESULT_PC_PLUS_FOUR
} result_src_t;
typedef enum logic {
	ALU_SRC_REG = 0,
	ALU_SRC_IMM
} alu_src_t;
typedef enum logic [1:0] {
	FWD_SRC_REG = 0,
	FWD_SRC_ALU_FORWARDED,
	FWD_SRC_MEM_FORWARDED
} alu_forwarding_src_t;
typedef enum logic {
	PC_SRC_PLUS_FOUR = 0,
	PC_SRC_JUMP_TARGET
} pc_src_t;
typedef enum logic [2:0] {
	BRANCH_CTRL_NO_BRANCH,
	BRANCH_CTRL_EQ,
	BRANCH_CTRL_NE,
	BRANCH_CTRL_LT,
	BRANCH_CTRL_GE
} branch_control_t;

// Standard control signals.
input  wire clk, rst, ena; // <- worry about implementing the ena signal last.
output logic [31:0] instructions_completed;

// Memory interface.
output logic [31:0] inst_mem_addr, data_mem_addr, data_mem_wr_data;
input wire [31:0] inst_mem_rd_data, data_mem_rd_data;
input wire is_inst_addr;
output logic data_mem_wr_ena;

// Hazard unit
logic lw_stall;
always_comb begin : hazard_unit
	// (Data hazards)

	// We would read from a register being written by the previous
	// instruction; forward from the memory stage to fix this
	if (((rs1_e === rd_addr_m) & reg_write_m) & (rs1_e != 0)) forward_src_a_e = FWD_SRC_ALU_FORWARDED;
	// We would read from a register being written by the instruction
	// before the previous instruction; forward from the writeback stage
	// to fix this
	else if (((rs1_e === rd_addr_w) & reg_write_w) & (rs1_e != 0)) forward_src_a_e = FWD_SRC_MEM_FORWARDED;
	else forward_src_a_e = FWD_SRC_REG;
	// Same cases as above, but with the second register being read by the
	// instruction
	if (((rs2_e === rd_addr_m) & reg_write_m) & (rs2_e != 0)) forward_src_b_e = FWD_SRC_ALU_FORWARDED;
	else if (((rs2_e === rd_addr_w) & reg_write_w) & (rs2_e != 0)) forward_src_b_e = FWD_SRC_MEM_FORWARDED;
	else forward_src_b_e = FWD_SRC_REG;

	// We use a register that will be written to by a load word
	// instruction in the execute stage (i.e. one that hasn't yet actually
	// loaded the word from memory); we have to stall to fix this
	lw_stall = result_src_e === RESULT_MEM_RD & ((rs1_d === rd_addr_e) | (rs2_d === rd_addr_e));
	// We also stall the fetch stage if we're trying to write or read
	// instruction memory; we need this because the instruction memory is
	// not dual-ported, so we can't fetch from instruction memory and
	// read/write to it in the memory stage at the same time
	stall_f = lw_stall | (is_inst_addr & (result_src_m == RESULT_MEM_RD | mem_write_m));
	stall_d = lw_stall;

	// (Control hazards)	

	// A branch was taken or a load put us in a bubble
	flush_d = pc_src_e === PC_SRC_JUMP_TARGET;
	flush_e = lw_stall | (pc_src_e === PC_SRC_JUMP_TARGET);
end

// [Fetch]
// Control registers
logic stall_f;

// Data registers
output logic [31:0] PC;
wire [31:0] pc_f;
logic [31:0] pc_next_f, pc_plus_4_f, instr_f;

always_ff @(posedge clk) begin : fetch_to_decode_reg
	if (flush_d | rst) begin
		instr_d <= 0;
		pc_d <= 0;
		pc_plus_4_d <= 0;
	end else begin
		if (~stall_d) begin
			instr_d <= instr_f;
			pc_d <= pc_f;
			pc_plus_4_d <= pc_plus_4_f;
		end
	end
end

// Combinational logic
register #(.N(32), .RESET(PC_START_ADDRESS)) PC_REGISTER (
  .clk(clk), .rst(rst), .ena(~stall_f), .d(pc_next_f), .q(pc_f)
);

always_comb begin : fetch_datapath
	PC = pc_f; // Alias

	pc_plus_4_f = pc_f + 4;

	case (pc_src_e)
		PC_SRC_PLUS_FOUR: pc_next_f = pc_plus_4_f;
		PC_SRC_JUMP_TARGET: pc_next_f = pc_target_e;
	endcase

	inst_mem_addr = stall_f ? 0 : pc_f;
	instr_f = inst_mem_rd_data;
end

// [Decode]
// Data registers
wire [31:0] rd1_d, rd2_d;
logic [4:0] rs1_d, rs2_d, rd_addr_d;
logic [31:0] instr_d, pc_d, imm_ext_d, pc_plus_4_d;

// Control registers
logic reg_write_d, mem_write_d, jump_d;
branch_control_t branch_control_d;
result_src_t result_src_d;
alu_control_t alu_control_d;
alu_src_t alu_src_d;
logic stall_d, flush_d;

always_ff @(posedge clk) begin : decode_to_execute_reg
	if (flush_e | rst) begin
		rd1_e <= 0;
		rd2_e <= 0;
		pc_e <= 0;
		rs1_e <= 0;
		rs2_e <= 0;
		rd_addr_e <= 0;
		imm_ext_e <= 0;
		pc_plus_4_e <= 0;
		`ifdef SIMULATION
		instr_e <= 0;
		`endif

		reg_write_e <= 0;
		result_src_e <= RESULT_ALU;
		mem_write_e <= 0;
		jump_e <= 0;
		branch_control_e <= BRANCH_CTRL_NO_BRANCH;
		alu_control_e <= ALU_INVALID;
		alu_src_e <= ALU_SRC_REG;
	end else begin
		rd1_e <= rd1_d;
		rd2_e <= rd2_d;
		pc_e <= pc_d;
		rs1_e <= rs1_d;
		rs2_e <= rs2_d;
		rd_addr_e <= rd_addr_d;
		imm_ext_e <= imm_ext_d;
		pc_plus_4_e <= pc_plus_4_d;
		`ifdef SIMULATION
		instr_e <= instr_d;
		`endif

		reg_write_e <= reg_write_d;
		result_src_e <= result_src_d;
		mem_write_e <= mem_write_d;
		jump_e <= jump_d;
		branch_control_e <= branch_control_d;
		alu_control_e <= alu_control_d;
		alu_src_e <= alu_src_d;
	end
end

// Combinational logic
register_file REGISTER_FILE(
  .clk(clk), 
  .wr_ena(reg_write_w), .wr_addr(rd_addr_w), .wr_data(result_w),
  .rd_addr0(rs1_d), .rd_addr1(rs2_d),
  .rd_data0(rd1_d), .rd_data1(rd2_d)
);

logic [6:0] op;
logic [2:0] funct3;
logic [31:25] funct7;
enum logic [2:0] {IMM_EXT_SRC_I_TYPE, IMM_EXT_SRC_S_TYPE, IMM_EXT_SRC_B_TYPE, IMM_EXT_SRC_J_TYPE, IMM_EXT_SRC_U_TYPE, IMM_EXT_INVALID} imm_ext_src;
always_comb begin : decode_unit
	op = instr_d[6:0];
	rd_addr_d = instr_d[11:7];
	funct3 = instr_d[14:12];
	rs1_d = instr_d[19:15];
	rs2_d = instr_d[24:20];
	funct7 = instr_d[31:25];

	case (op)
		OP_LTYPE, OP_ITYPE, OP_JALR: imm_ext_src = IMM_EXT_SRC_I_TYPE;
		OP_AUIPC, OP_LUI: imm_ext_src = IMM_EXT_SRC_U_TYPE;
		OP_STYPE: imm_ext_src = IMM_EXT_SRC_S_TYPE;
		OP_BTYPE: imm_ext_src = IMM_EXT_SRC_B_TYPE;	
		OP_JAL: imm_ext_src = IMM_EXT_SRC_J_TYPE;
		default: imm_ext_src = IMM_EXT_INVALID;
	endcase

	case (imm_ext_src)
		// We sign-extend (i.e. take MSB for top bytes) for everything
		// but U-type.
		IMM_EXT_SRC_I_TYPE: case (funct3)
			// sll and sra/srl have an effective funct7 in the
			// immediate that we don't want.
			FUNCT3_SLL, FUNCT3_SHIFT_RIGHT: imm_ext_d = {27'b0, instr_d[24:20]};
			default: imm_ext_d = {{20{instr_d[31]}}, instr_d[31:20]};
		endcase
		IMM_EXT_SRC_S_TYPE: imm_ext_d = {{20{instr_d[31]}}, instr_d[31:25], instr_d[11:7]};
		IMM_EXT_SRC_B_TYPE: imm_ext_d = {{20{instr_d[31]}}, instr_d[7], instr_d[30:25], instr_d[11:8], 1'b0};
		IMM_EXT_SRC_J_TYPE: imm_ext_d = {{12{instr_d[31]}}, instr_d[19:12], instr_d[20], instr_d[30:21], 1'b0};
		IMM_EXT_SRC_U_TYPE: imm_ext_d = {instr_d[31:12], 12'b0};
		default: imm_ext_d = 32'b0;
	endcase
end

logic magic_funct7, zero_funct7;
always_comb begin : control_unit
	// Controls the data memory write enable
	mem_write_d = op == OP_STYPE;

	// Controls the register write enable
	case (op)
		OP_RTYPE, OP_ITYPE, OP_JAL, OP_JALR, OP_LTYPE: reg_write_d = 1'b1;
		default: reg_write_d = 1'b0;
	endcase

	// Controls the mux directly before the ALU's src_b input
	case (op)
		OP_JALR, OP_ITYPE, OP_STYPE, OP_LTYPE: alu_src_d = ALU_SRC_IMM;
		OP_RTYPE, OP_BTYPE: alu_src_d = ALU_SRC_REG;
		default: alu_src_d = ALU_SRC_REG; // Maybe have a third option that zeros things out?
	endcase

	// Controls the mux before the program counter
	case (op)
		OP_JAL, OP_JALR: jump_d = 1'b1;
		default: jump_d = 1'b0;
	endcase
	if (op === OP_BTYPE) begin
		case (funct3)
			FUNCT3_BEQ: branch_control_d = BRANCH_CTRL_EQ;
			FUNCT3_BNE: branch_control_d = BRANCH_CTRL_NE;
			FUNCT3_BLT, FUNCT3_BLTU: branch_control_d = BRANCH_CTRL_LT;
			FUNCT3_BGE, FUNCT3_BGEU: branch_control_d = BRANCH_CTRL_GE;
			default: branch_control_d = BRANCH_CTRL_NO_BRANCH;
		endcase
	end else branch_control_d = BRANCH_CTRL_NO_BRANCH;

	// Controls the result mux
	case (op)
		OP_JAL, OP_JALR: result_src_d = RESULT_PC_PLUS_FOUR;
		OP_LTYPE: result_src_d = RESULT_MEM_RD;
		default: result_src_d = RESULT_ALU;
	endcase

	// Controls the ALU control input
	// XXX: The instructions that use funct7 to
	// differentiate themselves will break if there's
	// a non sub/sll/sra instruction with a nonzero
	// funct7 code.
	magic_funct7 = funct7 === 7'b0100000;
	zero_funct7 = funct7 === 7'b0;
	case (op)
		OP_STYPE, OP_LTYPE, OP_JAL, OP_JALR: alu_control_d = ALU_ADD;
		OP_RTYPE, OP_ITYPE: case (funct3)
			FUNCT3_ADD: begin
				if (op === OP_ITYPE | zero_funct7) alu_control_d = ALU_ADD;
				else if (op === OP_RTYPE & magic_funct7) alu_control_d = ALU_SUB;
				else alu_control_d = ALU_INVALID;
			end
			FUNCT3_SLL: begin
				if (zero_funct7) alu_control_d = ALU_SLL;
				else alu_control_d = ALU_INVALID;
			end
			FUNCT3_SLT: alu_control_d = ALU_SLT;
			FUNCT3_SLTU: alu_control_d = ALU_SLTU;
			FUNCT3_XOR: alu_control_d = ALU_XOR;
			FUNCT3_SHIFT_RIGHT: begin
				if (zero_funct7) alu_control_d = ALU_SRL;
				else if (magic_funct7) alu_control_d = ALU_SRA;
				else alu_control_d = ALU_INVALID;
			end
			FUNCT3_OR: alu_control_d = ALU_OR;
			FUNCT3_AND: alu_control_d = ALU_AND;
			default: alu_control_d = ALU_INVALID;
		endcase
		OP_BTYPE: case (funct3)
			FUNCT3_BLTU, FUNCT3_BGEU: alu_control_d = ALU_SLTU;	
			default: alu_control_d = ALU_SLT;
		endcase
		default: alu_control_d = ALU_INVALID;
	endcase
end


// [Execute]
// Data registers
logic [31:0] rd1_e, rd2_e, pc_e, src_a, src_b;
logic [4:0] rs1_e, rs2_e, rd_addr_e;
logic [31:0] alu_result_e, imm_ext_e, pc_target_e, pc_plus_4_e, write_data_e;
`ifdef SIMULATION
logic [31:0] instr_e; // useful for debugging
`endif

// Control registers
wire overflow_e, zero_e, equal_e;
logic reg_write_e, mem_write_e, jump_e;
branch_control_t branch_control_e;
result_src_t result_src_e;
alu_control_t alu_control_e;
alu_src_t alu_src_e;
alu_forwarding_src_t forward_src_a_e, forward_src_b_e;
pc_src_t pc_src_e;
logic flush_e;

always_ff @(posedge clk) begin : execute_to_memory_reg
	if (rst) begin
		alu_result_m <= 0;
		write_data_m <= 0;
		rd_addr_m <= 0;
		`ifdef SIMULATION
		instr_m <= 0;
		`endif

		reg_write_m <= 0;
		result_src_e <= RESULT_ALU;
		mem_write_m <= 0;
	end else begin
		alu_result_m <= alu_result_e;
		write_data_m <= write_data_e;
		rd_addr_m <= rd_addr_e;
		pc_plus_4_m <= pc_plus_4_e;
		`ifdef SIMULATION
		instr_m <= instr_e;
		`endif

		reg_write_m <= reg_write_e;
		result_src_m <= result_src_e;
		mem_write_m <= mem_write_e;
	end
end

// Combinational logic
always_comb begin : execute_datapath
	case (forward_src_a_e)
		FWD_SRC_REG: src_a = rd1_e;
		FWD_SRC_ALU_FORWARDED: src_a = alu_result_m;
		FWD_SRC_MEM_FORWARDED: src_a = result_w;
		default: src_a = 32'b0;
	endcase

	case (forward_src_b_e)
		FWD_SRC_REG: write_data_e = rd2_e;
		FWD_SRC_ALU_FORWARDED: write_data_e = alu_result_m;
		FWD_SRC_MEM_FORWARDED: write_data_e = result_w;
		default: write_data_e = 32'b0;
	endcase

	case (alu_src_e)
		ALU_SRC_REG: src_b = write_data_e;
		ALU_SRC_IMM: src_b = imm_ext_e;
		default: src_b = 32'b0;
	endcase

	// I regret making this an enum...
	`define pc_src_t_cast(val) (val) === 0 ? PC_SRC_PLUS_FOUR : PC_SRC_JUMP_TARGET
	case (branch_control_e)
		BRANCH_CTRL_NO_BRANCH: pc_src_e = `pc_src_t_cast(jump_e);
		BRANCH_CTRL_EQ: pc_src_e = `pc_src_t_cast(equal_e);
		BRANCH_CTRL_NE: pc_src_e = `pc_src_t_cast(~equal_e);
		BRANCH_CTRL_LT: pc_src_e = `pc_src_t_cast(alu_result_e[0]);
		BRANCH_CTRL_GE: pc_src_e = `pc_src_t_cast(equal_e | ~alu_result_e[0]);
		default: pc_src_e = PC_SRC_PLUS_FOUR;
	endcase

	pc_target_e = pc_e + imm_ext_e;	
end

alu_behavioural ALU (
  .a(src_a), .b(src_b), .result(alu_result_e),
  .control(alu_control_e),
  .overflow(overflow_e), .zero(zero_e), .equal(equal_e)
);

// [Memory]
// Control registers
logic reg_write_m, mem_write_m;
result_src_t result_src_m;

// Data registers
logic [31:0] alu_result_m, write_data_m, read_result_m, pc_plus_4_m;
`ifdef SIMULATION
logic [31:0] instr_m;
`endif
logic [4:0] rd_addr_m;

always_ff @(posedge clk) begin : memory_to_writeback_reg
	if (rst) begin
		alu_result_w <= 0;
		read_result_w <= 0;
		rd_addr_w <= 0;
		pc_plus_4_w <= 0;

		reg_write_w <= 0;
		result_src_w <= RESULT_ALU;
	end else begin
		alu_result_w <= alu_result_m;
		read_result_w <= read_result_m;
		rd_addr_w <= rd_addr_m;
		pc_plus_4_w <= pc_plus_4_m;

		reg_write_w <= reg_write_m;
		result_src_w <= result_src_m;
	end
end

// Combinational logic
always_comb begin : memory_datapath
	data_mem_addr = alu_result_m;
	data_mem_wr_data = write_data_m;
	data_mem_wr_ena = mem_write_m;
	read_result_m = data_mem_rd_data;
end

// [Writeback]
// Control registers
logic reg_write_w;
result_src_t result_src_w;

// Data registers
logic [31:0] alu_result_w, read_result_w, pc_plus_4_w, result_w;
logic [4:0] rd_addr_w;

// Combinational logic
always_comb begin : writeback_datapath
	case (result_src_w)
		RESULT_ALU: result_w = alu_result_w;
		RESULT_MEM_RD: result_w = read_result_w;
		RESULT_PC_PLUS_FOUR: result_w = pc_plus_4_w;
		default: result_w = 32'b0;
	endcase
end

endmodule
