# Usage:
#   vivado -mode batch -source run_psc_masked_impl.tcl -tclargs 128

set V_PARAM [lindex $argv 0]
if {$V_PARAM eq ""} {
  puts "ERROR: Missing V parameter. Example: -tclargs 128"
  quit
}

set TOP  "proposed_masked"
set PART "xcku5p-ffvb676-2-e"
set RTL  "proposed_masked.v"
set XDC  "psc_masked.xdc"

set OUT  "impl_${TOP}_V${V_PARAM}"
file mkdir $OUT

create_project -in_memory -part $PART
set_property target_language Verilog [current_project]

add_files -norecurse $RTL
add_files -fileset constrs_1 -norecurse $XDC
update_compile_order -fileset sources_1
set_property top $TOP [current_fileset]

# Synthesis with top-level parameter override
synth_design -top $TOP -part $PART -generic V=$V_PARAM
write_checkpoint -force $OUT/post_synth.dcp
report_utilization -hierarchical -file $OUT/util_post_synth.rpt
report_timing_summary -max_paths 20 -file $OUT/timing_post_synth.rpt

# Implementation
opt_design
place_design
phys_opt_design
route_design

write_checkpoint -force $OUT/post_route.dcp
report_utilization -hierarchical -file $OUT/util_post_route.rpt
report_timing_summary -max_paths 20 -file $OUT/timing_post_route.rpt
report_power -file $OUT/power_post_route.rpt
report_drc -file $OUT/drc_post_route.rpt

quit
