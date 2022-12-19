# Assumes addi and slli are working.
# Loads some values into the different memory banks.
# Assuming DATA_L is 256 words, or 1024 bytes. Subtract 4 to get the highest address in the system. Using gp as the start of data memory, sp as the end (stack pointer).
addi gp, zero, 3
slli gp, gp, 28
addi sp, gp, 512
addi t0, zero, 17
# Store some values onto the stack
sw t0, -4(sp)
addi t0, t0, 17
sw t0, -8(sp)
addi t0, t0, 17
sw t0, -12(sp)
addi t0, t0, 17
sw t0, -16(sp)
# Then write them back into the global memory and instruction memory.
lw t0, -16(sp)
sw t0, 0(gp)
sw t0, 120(zero)
lw t1, -12(sp)
sw t1, 4(gp)
sw t1, 124(zero)
lw t2, -8(sp)
sw t2, 8(gp)
sw t2, 128(zero)
lw t3, -4(sp)
sw t3, 12(gp)
sw t3, 132(zero)

# |---------------------------------------|
# | Final Register File State             |
# |---------------------------------------|
# |    x00, zero = 0x00000000 (         0)|
# |      x01, ra = 0xxxxxxxxx (         x)|
# |      x02, sp = 0x30000200 ( 805306880)|
# |      x03, gp = 0x30000000 ( 805306368)|
# |      x04, tp = 0xxxxxxxxx (         x)|
# |      x05, t0 = 0x00000044 (        68)|
# |      x06, t1 = 0x00000033 (        51)|
# |      x07, t2 = 0x00000022 (        34)|
# |      x08, s0 = 0xxxxxxxxx (         x)|
# |      x09, s1 = 0xxxxxxxxx (         x)|
# |      x10, a0 = 0xxxxxxxxx (         x)|
# |      x11, a1 = 0xxxxxxxxx (         x)|
# |      x12, a2 = 0xxxxxxxxx (         x)|
# |      x13, a3 = 0xxxxxxxxx (         x)|
# |      x14, a4 = 0xxxxxxxxx (         x)|
# |      x15, a5 = 0xxxxxxxxx (         x)|
# |      x16, a6 = 0xxxxxxxxx (         x)|
# |      x17, a7 = 0xxxxxxxxx (         x)|
# |      x18, s2 = 0xxxxxxxxx (         x)|
# |      x19, s3 = 0xxxxxxxxx (         x)|
# |      x20, s4 = 0xxxxxxxxx (         x)|
# |      x21, s5 = 0xxxxxxxxx (         x)|
# |      x22, s6 = 0xxxxxxxxx (         x)|
# |      x23, s7 = 0xxxxxxxxx (         x)|
# |      x24, s8 = 0xxxxxxxxx (         x)|
# |      x25, s9 = 0xxxxxxxxx (         x)|
# |     x26, s10 = 0xxxxxxxxx (         x)|
# |     x27, s11 = 0xxxxxxxxx (         x)|
# |      x28, t3 = 0x00000011 (        17)|
# |      x29, t4 = 0xxxxxxxxx (         x)|
# |      x30, t5 = 0xxxxxxxxx (         x)|
# |      x31, t6 = 0xxxxxxxxx (         x)|
# |---------------------------------------|
#
# mmu_data.out
# // 0x00000000
# 00000044
# 00000033
# 00000022
# 00000011
# ...
# xxxxxxxx
# xxxxxxxx
# 00000044
# 00000033
# 00000022
# 00000011
# // 0x00000080
#
# mmu_inst.out
# // 0x00000000
# 00300193
# 01c19193
# 20018113
# 01100293
# fe512e23
# 01128293
# fe512c23
# 01128293
# fe512a23
# 01128293
# fe512823
# ff012283
# 0051a023
# 06502c23
# ff412303
# 0061a223
# // 0x00000010
# 06602e23
# ff812383
# 0071a423
# 08702023
# ffc12e03
# 01c1a623
# 09c02223
# xxxxxxxx
# xxxxxxxx
# xxxxxxxx
# xxxxxxxx
# xxxxxxxx
# xxxxxxxx
# xxxxxxxx
# 00000044
# 00000033
# // 0x00000020
# 00000022
# 00000011
# xxxxxxxx