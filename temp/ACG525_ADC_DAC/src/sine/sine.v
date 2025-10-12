//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//Tool Version: V1.9.12 (64-bit)
//Part Number: GW5A-LV25UG324C2/I1
//Device: GW5A-25
//Device Version: A
//Created Time: Tue Oct  7 14:57:19 2025

module sine (dout, clk, oce, ce, reset, ad);

output [7:0] dout;
input clk;
input oce;
input ce;
input reset;
input [7:0] ad;

wire [23:0] prom_inst_0_dout_w;
wire gw_gnd;

assign gw_gnd = 1'b0;

pROM prom_inst_0 (
    .DO({prom_inst_0_dout_w[23:0],dout[7:0]}),
    .CLK(clk),
    .OCE(oce),
    .CE(ce),
    .RESET(reset),
    .AD({gw_gnd,gw_gnd,gw_gnd,ad[7:0],gw_gnd,gw_gnd,gw_gnd})
);

defparam prom_inst_0.READ_MODE = 1'b0;
defparam prom_inst_0.BIT_WIDTH = 8;
defparam prom_inst_0.RESET_MODE = "SYNC";
defparam prom_inst_0.INIT_RAM_00 = 256'hD7D4D2D0CDCBC8C6C3C0BEBBB8B5B2B0ADAAA7A4A19E9B9895928F8B8885827F;
defparam prom_inst_0.INIT_RAM_01 = 256'hFEFEFEFDFDFDFCFCFBFAF9F9F8F7F5F4F3F2F0EFEEECEAE9E7E5E3E1DFDDDBD9;
defparam prom_inst_0.INIT_RAM_02 = 256'hDBDDDFE1E3E5E7E9EAECEEEFF0F2F3F4F5F7F8F9F9FAFBFCFCFDFDFDFEFEFEFE;
defparam prom_inst_0.INIT_RAM_03 = 256'h8285888B8F9295989B9EA1A4A7AAADB0B2B5B8BBBEC0C3C6C8CBCDD0D2D4D7D9;
defparam prom_inst_0.INIT_RAM_04 = 256'h272A2C2E313336383B3E404346494C4E5154575A5D606366696C6F7376797C7F;
defparam prom_inst_0.INIT_RAM_05 = 256'h0000000101010202030405050607090A0B0C0E0F1012141517191B1D1F212325;
defparam prom_inst_0.INIT_RAM_06 = 256'h23211F1D1B1917151412100F0E0C0B0A09070605050403020201010100000000;
defparam prom_inst_0.INIT_RAM_07 = 256'h7C7976736F6C696663605D5A5754514E4C494643403E3B383633312E2C2A2725;

endmodule //sine
