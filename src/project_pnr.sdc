# Constraints used while placing, repairing and routing: the sign-off constraints (project.sdc),
# with a tighter limit on signal edge times so that edges stay within the sign-off limit at the
# slow corner as well, where they are about twice as slow.
source [file join [file dirname [info script]] project.sdc]
set_max_transition 0.5 [current_design]
puts "\[INFO\] Place-and-route edge time limit 0.5 ns"
