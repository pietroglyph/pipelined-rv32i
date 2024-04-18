# Test the timers and RGB LED by setting a sequence of colors.
INIT_GLOBALS:
  lui gp, 0x30000
  addi s0, zero, 3
  slli s0, s0, 6
  add s0, s0, gp 
  add s1, zero, zero

MAIN:
  addi a0, zero, 1
  call DELAY
  addi s1, s1, 1
  beq zero, zero, MAIN

# Delay function - loops for a0 milliseconds. Returns nothing.
DELAY: addi t0, x0, 1 
  slli t0, t0, 28 # Get MMR base address into register t0.
  # Get the timer (offset 12 - check memmap.sv to make sure that's accurate.)
  lw t1, 12(t0)
  # Add a0 to the timer result to know what value to wait for.
  add t2, t1, a0
  DELAY_FOR:  lw t1, 12(t0)
  bge t1, t2, DELAY_DONE
  j DELAY_FOR
DELAY_DONE: ret # shorthand for `jr ra`
