# Xilinx Vivado Gui-less build script
# Based on this tutorial https://www.xilinx.com/video/hardware/using-the-non-project-batch-flow.html
# read_edif and read_ip commands are useful for using other IP.
# Note - you can add `start_gui` between any of these commands to interject into the Vivado environment. This lets you use any tools (and you can open any checkpoint too!).

# Set this to the memh file you'd like to use to synthesize. Be sure to update the Makefile dependency too.
set_property verilog_define INITIAL_INST_MEM="asm/peripherals.memh" [current_fileset]
read_verilog [ glob hdl/*.sv ]
read_xdc ./rv32i_system.xdc

# Sythesis & Optimization
# synth_design -top rv32i_system -part xc7a15tcpg236-1
synth_design -top rv32i_system -part xc7a35tcpg236-1
report_drc -file drc.log -verbose
write_checkpoint -force synthesis.checkpoint
opt_design
# power_opt_design # optional till later.

report_timing_summary -file timing_summary.log
report_timing -sort_by group -max_paths 100 -path_type summary -file timing.log -verbose
report_utilization -file usage.log -verbose
report_utilization -hierarchical -file usage_by_module.log -verbose

place_design
# phys_opt_design # optional till later.
write_checkpoint -force place.checkpoint
route_design
write_checkpoint -force route.checkpoint

report_clocks -file clocks.log

# write bitstream
write_bitstream -force ./rv32i_system.bit