set DCP    [lindex $argv 0]
set SAIF   [lindex $argv 1]
set OUTDIR [lindex $argv 2]

file mkdir $OUTDIR
open_checkpoint $DCP

read_saif -strip_path tb_proposed_masked/dut -out_file $OUTDIR/read_saif_unmatched.rpt $SAIF

report_power -file $OUTDIR/power_saif.rpt
report_power -hier all -hierarchical_depth 0 -file $OUTDIR/power_saif_hier.rpt
report_switching_activity -file $OUTDIR/switching_saif.rpt
quit
