# Testing if our core can execute the jumps, loads, and stores for complex
# function calls.
# Note - call = jal ra, LABEL.
# Note - ret =  jr ra = jalr x0, ra, 0.

# Use our data memory bank for our stack pointer. 
# Note - need to update this based on the size of DATA_MEM.
# Assuming DATA_L is 256 words, or 1024 bytes. Subtract 4 to get the highest address in the system.
# 
addi sp, zero, 3
slli sp, sp, 28
addi sp, sp, 1020 

# Test sum_4
addi a0, zero, 1
addi a1, zero, 2
addi a2, zero, 3
addi a3, zero, 4
call SUM_4

addi t0, zero, 10
bne a0, t0, DONE # End the program if the result isn't 10.

# Test DIFF_OF_SUMS
addi a0, zero, 4
addi a1, zero, 3
addi a2, zero, 2
addi a3, zero, 1
call DIFF_OF_SUMS
addi t0, zero, 4
bne a0, t0, DONE # End the program if the result isn't 4.

# Test Multiply.
addi a0, zero, 0
addi a1, zero, 17
call MUL
bne a0, zero, DONE # End program on wrong result.

addi a0, zero, 17
addi a1, zero, 0
call MUL
bne a0, zero, DONE # End program on wrong result.

addi a0, zero, 1
addi a1, zero, 17
call MUL
addi t0, zero, 17
bne a0, t0, DONE # End program on wrong result.

addi a0, zero, 17
addi a1, zero, 1
call MUL
addi t0, zero, 17
bne a0, t0, DONE # End program on wrong result.

# Test Fibonnaci(2)
addi a1, zero, 100  # Stop if we have more than 100 calls
# 1 1 2 3 5 8 13 21 34 55
add s11, zero, zero # Reset stack counter
addi a0, zero, 2
call FIBONACCI
addi t0, zero, 1
bne a0, t0, DONE # End program on wrong result.

# Test Fibonnaci(3)
addi a1, zero, 100  # Stop if we have more than 100 calls
# 1 1 2 3 5 8 13 21 34 55
add s11, zero, zero # Reset stack counter
addi a0, zero, 3
call FIBONACCI
addi t0, zero, 2
bne a0, t0, DONE # End program on wrong result.

# Test Fibonnaci(7)
addi a1, zero, 100  # Stop if we have more than 100 calls
# 1 1 2 3 5 8 13 21 34 55
add s11, zero, zero # Reset stack counter
addi a0, zero, 7
call FIBONACCI
addi t0, zero, 13
bne a0, t0, DONE # End program on wrong result.

# Infinite loop: makes sure we don't accidentally start executing defined functions.
DONE: beq zero, zero, DONE

# Create a few "leaf" functions - functions that don't call any other functions inside them.
# return a0 + a1 + a2 + a3 
SUM_4:
  add t0, a0, a1 # SUM_4: use temporaries, no need to preserve regs.
  add t1, a2, a3
  add a0, t0, t1
  ret

# return (a0 + a1) - (a2 + a3)
DIFF_OF_SUMS:
  add t0, a0, a1
  add t1, a2, a3
  sub a0, t0, t1
  ret

# Multiply a0 and a1, ignoring overflows (assumes a0 and a1 are only 16-bit)
MUL:
  # Check if either argument is zero, then return zero quickly.
  beq a0, zero, MUL_PRODUCT_IS_ZERO
  beq a1, zero, MUL_PRODUCT_IS_ZERO
  addi t0, zero, 1
  beq a0, t0, MUL_A0_IS_ONE
  beq a1, t1, MUL_A1_IS_ONE
  add t0, zero, a1 # loop index start
  add t1, zero, zero # product accumulator
  MUL_FOR_BEGIN: # for(t0 = a1-1; t0 > 0; t0--) 
    beq t0, zero, MUL_FOR_END
    add t1, t1, a0
    addi t0, t0, -1 # t0--
  MUL_FOR_END:
    add a0, x0, t1
    ret
  MUL_PRODUCT_IS_ZERO:
    add a0, zero, zero # Early return if product is zero.
    ret
  MUL_A0_IS_ONE:
    add a0, a1, zero # Early return if product is a1.
    ret
  MUL_A1_IS_ONE:
    ret # Early return if product is a0.

# Computes the fibonacci sequence of a0, with a maximum recursion limit of a1.
# Since this is recursive (non-leaf), each call needs to preserve a0 and ra in the stack.
# Assumes that s11 is zero'd prior to the first Fibonacci call and uses that 
# to track the number of recursive calls.
FIBONACCI:
  bge s11, a1, FIB_STACK_OVERFLOW # Check if we've exceeded a1 calls, if so return -1. 
  addi s11, s11, 1  # Keep track of number of iterations in s11.
  FIB_STACK_PUSH:
    addi sp, sp, -12  # Make space for variables that need to be preserved call to call.
    sw a0,  0(sp)     # Push a0 onto the stack.
    sw ra,  4(sp)     # Push ra onto the stack.
    sw s0,  8(sp)     # Use s0 to store the extra call result.
  # Handle special cases - fib(1) -> 1, fib(2) -> 1.
  addi t0, zero, 1    
  beq a0, t0, FIB_ONE
  addi t0, zero, 2
  beq  a0, t0, FIB_ONE
  addi a0, a0, -1
  call FIBONACCI # fib(n-1)
  add s0, a0, zero # save the result.
  lw a0, 0(sp) # Recall original a0. 
  addi a0, a0, -2
  call FIBONACCI # fib(n-2)
  add a0, s0, a0
  FIB_STACK_POP_AND_RETURN:
    # Note, do not pop a0 because that is also the return value location.
    lw ra,  4(sp)     # Pop ra from the stack.
    lw s0,  8(sp)     # Pop s0 from the stack.
    addi sp, sp, 12   # Deallocate the stack variables.
    ret
  FIB_ONE:
    addi a0, zero, 1
    j FIB_STACK_POP_AND_RETURN
  FIB_STACK_OVERFLOW:
    addi a0, zero, -1
    j FIB_STACK_POP_AND_RETURN
# |---------------------------------------|
# | Final Register File State             |
# |---------------------------------------|
# |    x00, zero = 0x00000000 (         0)|
# |      x01, ra = 0x000000cc (       204)|
# |      x02, sp = 0x300003fc ( 805307388)|
# |      x03, gp = 0xxxxxxxxx (         x)|
# |      x04, tp = 0xxxxxxxxx (         x)|
# |      x05, t0 = 0x0000000d (        13)|
# |      x06, t1 = 0x00000011 (        17)|
# |      x07, t2 = 0xxxxxxxxx (         x)|
# |      x08, s0 = 0xxxxxxxxx (         x)|
# |      x09, s1 = 0xxxxxxxxx (         x)|
# |      x10, a0 = 0x0000000d (        13)|
# |      x11, a1 = 0x00000064 (       100)|
# |      x12, a2 = 0x00000002 (         2)|
# |      x13, a3 = 0x00000001 (         1)|
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
# |     x27, s11 = 0x00000019 (        25)|
# |      x28, t3 = 0xxxxxxxxx (         x)|
# |      x29, t4 = 0xxxxxxxxx (         x)|
# |      x30, t5 = 0xxxxxxxxx (         x)|
# |      x31, t6 = 0xxxxxxxxx (         x)|
# |---------------------------------------|