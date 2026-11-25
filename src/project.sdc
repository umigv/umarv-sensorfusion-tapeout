# Timing constraints for the sensor-fusion chip: the flow's standard constraints, plus one exception.
source $::env(SCRIPTS_DIR)/base.sdc

# The Kalman filter's sequencer waits one clock after its program counter changes before it uses
# anything derived from the step table (ukf_top.sv, word_ok). Every path that starts at the
# program counter therefore has two clocks.
set ukf_pc_nets [get_nets -quiet {u_ukf.pc[*]}]
if { [llength $ukf_pc_nets] == 0 } { set ukf_pc_nets [get_nets -quiet {u_ukf\.pc\[*\]}] }
puts "\[INFO\] UKF two-clock path: [llength $ukf_pc_nets] program counter nets"
if { [llength $ukf_pc_nets] > 0 } {
    set_multicycle_path -setup 2 -through $ukf_pc_nets
    set_multicycle_path -hold 1 -through $ukf_pc_nets
}
