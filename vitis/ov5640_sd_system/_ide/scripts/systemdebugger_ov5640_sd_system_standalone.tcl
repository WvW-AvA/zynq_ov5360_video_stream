# Usage with Vitis IDE:
# In Vitis IDE create a Single Application Debug launch configuration,
# change the debug type to 'Attach to running target' and provide this 
# tcl script in 'Execute Script' option.
# Path of this script: D:\verilog\AXU2CGB\course_s2\course_s2\25_an5642_sd\vitis\ov5640_sd_system\_ide\scripts\systemdebugger_ov5640_sd_system_standalone.tcl
# 
# 
# Usage with xsct:
# To debug using xsct, launch xsct and run below command
# source D:\verilog\AXU2CGB\course_s2\course_s2\25_an5642_sd\vitis\ov5640_sd_system\_ide\scripts\systemdebugger_ov5640_sd_system_standalone.tcl
# 
connect -url tcp:127.0.0.1:3121
source D:/Xilinx/Vitis/2022.2/scripts/vitis/util/zynqmp_utils.tcl
targets -set -nocase -filter {name =~"APU*"}
rst -system
after 3000
targets -set -filter {jtag_cable_name =~ "Digilent JTAG-HS1 210512180081" && level==0 && jtag_device_ctx=="jsn-JTAG-HS1-210512180081-14711093-0"}
fpga -file D:/verilog/AXU2CGB/course_s2/course_s2/25_an5642_sd/vitis/ov5640_sd/_ide/bitstream/design_1_wrapper.bit
targets -set -nocase -filter {name =~"APU*"}
loadhw -hw D:/verilog/AXU2CGB/course_s2/course_s2/25_an5642_sd/vitis/design_1_wrapper/export/design_1_wrapper/hw/design_1_wrapper.xsa -mem-ranges [list {0x80000000 0xbfffffff} {0x400000000 0x5ffffffff} {0x1000000000 0x7fffffffff}] -regs
configparams force-mem-access 1
targets -set -nocase -filter {name =~"APU*"}
set mode [expr [mrd -value 0xFF5E0200] & 0xf]
targets -set -nocase -filter {name =~ "*A53*#0"}
rst -processor
dow D:/verilog/AXU2CGB/course_s2/course_s2/25_an5642_sd/vitis/design_1_wrapper/export/design_1_wrapper/sw/design_1_wrapper/boot/fsbl.elf
set bp_53_25_fsbl_bp [bpadd -addr &XFsbl_Exit]
con -block -timeout 60
bpremove $bp_53_25_fsbl_bp
targets -set -nocase -filter {name =~ "*A53*#0"}
rst -processor
dow D:/verilog/AXU2CGB/course_s2/course_s2/25_an5642_sd/vitis/ov5640_sd/Debug/ov5640_sd.elf
configparams force-mem-access 0
targets -set -nocase -filter {name =~ "*A53*#0"}
con
