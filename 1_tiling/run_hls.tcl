#===================================
# run_hls.tcl for GEMM
#===================================
# open the HLS project fir.prj
open_project hotspot3D.prj -reset
# set the top-level function of the design to be fir
set_top kernel_3D_hw
# add design files
add_files hotspot3D.cpp
# add the testbench files
add_files -tb hotspot3D_test.cpp
add_files -tb ../../data/hotspot3D/power_512x8
add_files -tb ../../data/hotspot3D/temp_512x8
add_files -tb output_512x8.out
# open HLS solution solution1
open_solution "solution1"
# set target FPGA device: Alveo U50 in this example
set_part {xcu50-fsvh2104-2-e}
# target clock period is 5 ns, i.e., 200MHz
create_clock -period 3.33

# do a c simulation
csim_design -ldflags {-z stack-size=10485760} 
# synthesize the design
csynth_design
# do a co-simulation
#cosim_design
# close project and quit
close_project
# exit Vivado HLS
quit
