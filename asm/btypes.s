# Assumes addi is working.
# Runs a few branches to skip over incrementing x31.
# At the end of the program x10-x15 should be 1,
# and x31 should still be zero.

addi x1, x0, 1
addi x2, x0, -1
addi x31, x0, 0
beq x0, x0, BEQ_WORKS
  addi x31, x31, 1 # Should never run!
BEQ_WORKS: addi x10, x0, 1
bne x1, x2, BNE_WORKS
  addi x31, x31, 1 # Should never run!
BNE_WORKS: addi x11, x0, 1
blt x2, x1, BLT_WORKS
  addi x31, x31, 1 # Should never run!
BLT_WORKS: addi x12, x0, 1
bge x0, x0, BGE_WORKS
  addi x31, x31, 1 # Should never run!
BGE_WORKS: addi x13, x0, 1
bltu x1, x2, BLTU_WORKS
  addi x31, x31, 1 # Should never run!
BLTU_WORKS: addi x14, x0, 1
bgeu x2, x1, BGEU_WORKS
  addi x31, x31, 1 # Should never run!
BGEU_WORKS: addi x15, x0, 1
INFINITE_LOOP: beq x0, x0, INFINITE_LOOP

# |---------------------------|
# | Final Register File State |
# |---------------------------|
# |    x00, zero = 0x00000000 |
# |      x01, ra = 0x00000001 |
# |      x02, sp = 0xffffffff |
# |      x03, gp = 0xxxxxxxxx |
# |      x04, tp = 0xxxxxxxxx |
# |      x05, t0 = 0xxxxxxxxx |
# |      x06, t1 = 0xxxxxxxxx |
# |      x07, t2 = 0xxxxxxxxx |
# |      x08, s0 = 0xxxxxxxxx |
# |      x09, s1 = 0xxxxxxxxx |
# |      x10, a0 = 0x00000001 |
# |      x11, a1 = 0x00000001 |
# |      x12, a2 = 0x00000001 |
# |      x13, a3 = 0x00000001 |
# |      x14, a4 = 0x00000001 |
# |      x15, a5 = 0x00000001 |
# |      x16, a6 = 0xxxxxxxxx |
# |      x17, a7 = 0xxxxxxxxx |
# |      x18, s2 = 0xxxxxxxxx |
# |      x19, s3 = 0xxxxxxxxx |
# |      x20, s4 = 0xxxxxxxxx |
# |      x21, s5 = 0xxxxxxxxx |
# |      x22, s6 = 0xxxxxxxxx |
# |      x23, s7 = 0xxxxxxxxx |
# |      x24, s8 = 0xxxxxxxxx |
# |      x25, s9 = 0xxxxxxxxx |
# |     x26, s10 = 0xxxxxxxxx |
# |     x27, s11 = 0xxxxxxxxx |
# |      x28, t3 = 0xxxxxxxxx |
# |      x29, t4 = 0xxxxxxxxx |
# |      x30, t5 = 0xxxxxxxxx |
# |      x31, t6 = 0x00000000 |
# |---------------------------|