//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.12 (64-bit)
//Part Number: GW5A-LV25UG324C2/I1
//Device: GW5A-25
//Device Version: A
//Created Time: Thu Oct 30 20:53:56 2025

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

    wave_ram your_instance_name(
        .dout(dout), //output [7:0] dout
        .clka(clka), //input clka
        .cea(cea), //input cea
        .clkb(clkb), //input clkb
        .ceb(ceb), //input ceb
        .oce(oce), //input oce
        .reset(reset), //input reset
        .ada(ada), //input [8:0] ada
        .din(din), //input [7:0] din
        .adb(adb) //input [8:0] adb
    );

//--------Copy end-------------------
