//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.12 (64-bit)
//Part Number: GW5A-LV25UG324C2/I1
//Device: GW5A-25
//Device Version: A
//Created Time: Mon Nov  3 18:34:26 2025

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

    Gowin_PLL_MOD your_instance_name(
        .lock(lock), //output lock
        .clkout0(clkout0), //output clkout0
        .clkout2(clkout2), //output clkout2
        .mdrdo(mdrdo), //output [7:0] mdrdo
        .clkin(clkin), //input clkin
        .reset(reset), //input reset
        .mdclk(mdclk), //input mdclk
        .mdopc(mdopc), //input [1:0] mdopc
        .mdainc(mdainc), //input mdainc
        .mdwdi(mdwdi) //input [7:0] mdwdi
    );

//--------Copy end-------------------
