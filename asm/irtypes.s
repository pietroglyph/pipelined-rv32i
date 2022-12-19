addi  x1, x0, -1
addi  x2, x0, 35
xori  x3, x2, 68
ori   x4, x2, 17
andi  x5, x2, 17
slli  x6, x1, 8
srli  x7, x3, 2
srai  x8, x6, 6
slti  x9, x1, 35
sltiu x10, x1, 35
add   x11, x1, x1
sub   x12, x1, x1
xor   x13, x1, x12
or    x14, x2, x3
and   x15, x2, x3
addi  x16, x0, 4
sll   x17, x2, x16
srl   x18, x3, x16
sra   x19, x6, x16
slt   x20, x1, x2
sltu  x21, x1, x2
# |---------------------------------------|
# | Final Register File State             |
# |---------------------------------------|
# |    x00, zero = 0x00000000 (         0)|
# |      x01, ra = 0xffffffff (        -1)|
# |      x02, sp = 0x00000023 (        35)|
# |      x03, gp = 0x00000067 (       103)|
# |      x04, tp = 0x00000033 (        51)|
# |      x05, t0 = 0x00000001 (         1)|
# |      x06, t1 = 0xffffff00 (      -256)|
# |      x07, t2 = 0x00000019 (        25)|
# |      x08, s0 = 0xfffffffc (        -4)|
# |      x09, s1 = 0x00000001 (         1)|
# |      x10, a0 = 0x00000000 (         0)|
# |      x11, a1 = 0xfffffffe (        -2)|
# |      x12, a2 = 0x00000000 (         0)|
# |      x13, a3 = 0xffffffff (        -1)|
# |      x14, a4 = 0x00000067 (       103)|
# |      x15, a5 = 0x00000023 (        35)|
# |      x16, a6 = 0x00000004 (         4)|
# |      x17, a7 = 0x00000230 (       560)|
# |      x18, s2 = 0x00000006 (         6)|
# |      x19, s3 = 0x0ffffff0 ( 268435440)|
# |      x20, s4 = 0x00000001 (         1)|
# |      x21, s5 = 0x00000000 (         0)|
# |      x22, s6 = 0xxxxxxxxx (         x)|
# |      x23, s7 = 0xxxxxxxxx (         x)|
# |      x24, s8 = 0xxxxxxxxx (         x)|
# |      x25, s9 = 0xxxxxxxxx (         x)|
# |     x26, s10 = 0xxxxxxxxx (         x)|
# |     x27, s11 = 0xxxxxxxxx (         x)|
# |      x28, t3 = 0xxxxxxxxx (         x)|
# |      x29, t4 = 0xxxxxxxxx (         x)|
# |      x30, t5 = 0xxxxxxxxx (         x)|
# |      x31, t6 = 0xxxxxxxxx (         x)|
# |---------------------------------------|