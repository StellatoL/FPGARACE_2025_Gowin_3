//Copyright (C)2014-2025 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.12 (64-bit) 
//Created Time: 2025-10-27 21:56:35
create_clock -name clk_50M_in -period 20 -waveform {0 10} [get_ports {clk_50M}]
create_generated_clock -name clk_400M -source [get_ports {clk_50M}] -master_clock clk_50M_in -divide_by 3 -multiply_by 32 [get_pins {your_instance_name/u_pll/PLLA_inst/CLKOUT2}]
create_generated_clock -name clk_out -source [get_nets {clk_400M}] -master_clock clk_400M -divide_by 4 [get_pins {ddr3_ctrl_2port/DDR3_Memory_Interface_Top/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]
set_clock_groups -asynchronous -group [get_clocks {clk_50M_in}] -group [get_clocks {clk_400M}]
set_clock_groups -asynchronous -group [get_clocks {clk_400M}] -group [get_clocks {clk_out}]
set_clock_groups -asynchronous -group [get_clocks {clk_50M_in}] -group [get_clocks {clk_out}]