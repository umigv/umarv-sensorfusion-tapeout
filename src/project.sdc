# Timing constraints for the sensor-fusion chip: the flow's standard constraints, plus the
# two-clock paths. Each is a place where the logic waits a clock (or more) before using a value,
# and each has a matching check in simulation that makes the value unknown during the wait.
source $::env(SCRIPTS_DIR)/base.sdc

proc two_clock_from { what patterns } {
    set nets {}
    foreach p $patterns { set nets [concat $nets [get_nets -quiet $p]] }
    puts "\[INFO\] UKF two-clock path, $what: [llength $nets] nets"
    if { [llength $nets] > 0 } {
        set_multicycle_path -setup 2 -through $nets
        set_multicycle_path -hold 1 -through $nets
    }
}

# The same, for paths that go through one of the nets and then one of the pins. Everything is
# kept inside the procedures: the flow's own scripts use global Tcl variables, and a variable set
# here at the top level can collide with one of them.
proc two_clock_between { what net_patterns pin_patterns } {
    set nets {}
    foreach p $net_patterns { set nets [concat $nets [get_nets -quiet $p]] }
    set pins {}
    foreach p $pin_patterns { set pins [concat $pins [get_pins -quiet $p]] }
    puts "\[INFO\] UKF two-clock path, $what: [llength $nets] nets, [llength $pins] pins"
    if { [llength $nets] > 0 && [llength $pins] > 0 } {
        set_multicycle_path -setup 2 -through $nets -through $pins
        set_multicycle_path -hold 1 -through $nets -through $pins
    }
}

# 1. The Kalman filter's sequencer waits one clock after its program counter changes before it
#    uses anything derived from the step table (ukf_top.sv, word_ok).
two_clock_from "filter program counter" [list {u_ukf.pc[*]} {u_ukf\.pc\[*\]}]

# 2. The SPI block registers the address of the next byte to send and takes the byte two clocks
#    later (spi_regs.sv, rd_addr_q and fetch).
#    Synthesis merges rd_addr_q with the address counter (addr), which always holds the same
#    value, and keeps the name addr. The counter's other uses are also safe with two clocks: it
#    only changes when a byte completes, and is next used when the following byte completes.
two_clock_from "SPI read address" [list {u_regs.rd_addr_q[*]} {u_regs\.rd_addr_q\[*\]} {u_regs.addr[*]} {u_regs\.addr\[*\]}]

# 3. The lidar point buffer's read pointer moves at least nine clocks before the distance it
#    selects is taken (lidar_point_buffer.sv and lidar_xyz.sv). Only the way through the
#    256-word selector is relaxed: the pointer's other use (how full the buffer is) is not.
two_clock_between "lidar buffer read" \
    [list {u_lidar.u_buffer.rd_ptr[*]} {u_lidar\.u_buffer\.rd_ptr\[*\]}] \
    [list {u_lidar.u_buffer.u_mem.ob*.u_obuf/X} {u_lidar\.u_buffer\.u_mem\.ob*\.u_obuf/X}]

# 4. The Kalman filter's storage read. The read address comes from a register (ra), or from the
#    SPI block while its borrow signal (spi_sel) is high, and the word read is not taken until
#    both have been steady for a full clock (ukf_top.sv, rd_wait; spi_regs.sv, fetch). Only the
#    way through the buffers on the word read is relaxed: the borrow signal also holds the
#    sequencer, and that must happen within one clock.
#    After synthesis the borrow signal's net is named spi_rd (the port it drives), not spi_sel.
two_clock_between "filter storage read" \
    [list {u_ukf.ra[*]} {u_ukf\.ra\[*\]} {spi_rd} {u_regs.spi_rd} {u_ukf.spi_rd} {u_regs.spi_sel} {u_regs\.spi_sel} {u_ukf.cfg_index[*]} {u_ukf\.cfg_index\[*\]}] \
    [list {u_ukf.dinb*.u_dinbuf/X} {u_ukf\.dinb*\.u_dinbuf/X}]
