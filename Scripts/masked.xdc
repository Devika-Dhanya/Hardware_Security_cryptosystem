## Minimal clock constraint for Vivado
set CLK_PERIOD_NS 10.000
create_clock -name clk -period $CLK_PERIOD_NS [get_ports clk]

# Don't time async control pins
set_false_path -from [get_ports rst_n]
set_false_path -from [get_ports start]
set_false_path -from [get_ports load_en]
