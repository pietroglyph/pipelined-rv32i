open_hw_manager
connect_hw_server
open_hw_target
set device [lindex [get_hw_devices] 0]
set_property PROGRAM.FILE { rv32i_system.bit } [ current_hw_device ] 
current_hw_device $device
program_hw_devices $device
refresh_hw_device [lindex [get_hw_devices] 0]
close_hw_target
close_hw_manager