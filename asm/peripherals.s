# Write an interesting pattern into LED MMR. 
# Set a variable as the MMR Address, then set the bit fields one by one (shifting in between)
addi s1, x0, 1
slli s1, s1, 28 # s1 = 32'h1000_0000, the MMR Address
# MMR is {led0[3:0], led1[3:0], red[7:0], green[7:0], blue[7:0]
# LED peripheral knows that the RGB led is active low.
# Can set low bits with immediate instructions, then shift left.
# Common technique (bit masking)
ori  t0, x0, 8 # led0
slli t0, t0, 4 
ori  t0, t0, 4 # led1
slli t0, t0, 4
ori  t0, t0, 127 # half red
slli t0, t0, 8
ori  t0, t0, 0 # no green
slli t0, t0, 8
ori  t0, t0, 255 # full blue
sw t0, 0(s1)

# Test if we can "call" a simple function that doesn't modify the stack.
call VRAM_SWEEP # equivalent to JAL ra, VRAM_SWEEP

# Test the timers by blinking LED0 a few times.
# t0: holds address to timer MMR.
# s1: counts down
# Change either the number of blinks or the off/on time once you have a 
# synthesized design - that will validate that your code is actually working.
addi s1, zero, 12
BLINK_FOR:  beq s1, zero, DONE
            addi s1, s1, -1
            # LED on
            addi a0, zero, 1
            jal ra, SET_LED0
            # Wait 250ms
            addi a0, zero, 250
            jal ra, DELAY
            # LED off
            add a0, zero, zero
            jal ra, SET_LED0
            # Wait 500ms
            addi a0, zero, 500
            jal ra, DELAY
            j BLINK_FOR

DONE: beq x0, x0, DONE # infinite loop at end

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
SET_LED0:  addi t0, x0, 1 # Get MMR base address into register t0.
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

# Write an interesting pattern into VRAM (DMA example)
# Set a variable as the start of VRAM
# Set loop from 0 to 320*240 of VRAM addresses
# t1 = 320*240. Too big for an immediate! Use a shift to get the right value
# for (t0 = 0; t0 < t1; t0 = t0 + 1)
#   t2 = s1 + t0
#   vram[t2] = t2
VRAM_SWEEP: addi t3, x0, 2
  slli t3, t3, 28 # t3 =  32'h2000_0000, the base address for VRAM
  addi t1, x0, 75 # t1 = x6 = 75
  slli t1, t1, 10 # # t1 = 320*240
  addi t0, x0, 0  # t0 : 'color' to write into the pixel
  LOOP_START: add t2, t3, t0 # t2 = x7 = x8 + x5
    addi t0, t0, 17
    sw t0, 0(t2)  
    blt t0, t1, LOOP_START
  ret