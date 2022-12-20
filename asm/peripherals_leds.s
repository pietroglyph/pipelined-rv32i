# Test the timers and RGB LED by setting a sequence of colors.
INIT_GLOBALS:
  lui gp, 0x30000
  addi s0, zero, 3
  slli s0, s0, 6
  add s0, s0, gp 

MAIN:
  # Red
  addi a0, zero, 255
  addi a1, zero, 0
  addi a2, zero, 0
  call SET_RGB
  addi a0, zero, 1000
  call DELAY
  # Purple
  addi a0, zero, 127
  addi a1, zero, 0
  addi a2, zero, 127
  call SET_RGB
  addi a0, zero, 1000
  call DELAY
  # Blue
  addi a0, zero, 0
  addi a1, zero, 0
  addi a2, zero, 255
  call SET_RGB
  addi a0, zero, 1000
  call DELAY
  # Aqua
  addi a0, zero, 0
  addi a1, zero, 127
  addi a2, zero, 127
  call SET_RGB
  addi a0, zero, 1000
  call DELAY
  # Green
  addi a0, zero, 0
  addi a1, zero, 255
  addi a2, zero, 0
  call SET_RGB
  addi a0, zero, 1000
  call DELAY
  # Orange
  addi a0, zero, 127
  addi a1, zero, 127
  addi a2, zero, 0
  call SET_RGB
  addi a0, zero, 1000
  call DELAY
  
  # Toggle LED0
  call SET_LED0
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

# Control LED. If a0 is nonzero, turn on LED0, else off.
SET_LED0:  
  addi t0, x0, 1 # Get MMR base address into register t0.
  slli t0, t0, 28
  # Get current LED register into t1
  lw t1, 0(t0)
  # Generate the bit mask for the LED being on or off
  add t2, zero, zero
  beq a0, zero, LED0_OFF # if LED 0 is off, skip next instruction.
  # Set t2 to the bitmask for LED0 being fully on.
  addi t2, zero, 15 # else case
  LED0_OFF: slli t2, t2, 28
  # Set the new new LED register state.
  xor t1, t1, t2
  # Store the result back to MMR
  sw t1, 0(t0)
  ret # shorthand for `jr ra`


# RGB control. Takes a0, a1, a2 as r, g, and b between 0 and 255.
# MMR is {led0[3:0], led1[3:0], red[7:0], green[7:0], blue[7:0]
# LED peripheral knows that the RGB led is active low.
# Can set low bits with immediate instructions, then shift left.
# Common technique (bit masking)
SET_RGB: # void SET_RGB(red, green, blue)
  addi t0, zero, 1
  slli t0, t0, 28 # MMR Base Address
  or t1, zero, a0
  slli t1, t1, 8
  or t1, t1, a1
  slli t1, t1, 8
  or t1, t1, a2
  sw t1, 0(t0)
  ret