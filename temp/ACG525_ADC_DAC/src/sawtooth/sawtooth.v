//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//Tool Version: V1.9.12 (64-bit)
//Part Number: GW5A-LV25UG324C2/I1
//Device: GW5A-25
//Device Version: A
//Created Time: Tue Oct  7 15:00:17 2025

module sawtooth (dout, clk, oce, ce, reset, ad);

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
defparam prom_inst_0.INIT_RAM_00 = 256'h1F1E1D1C1B1A191817161514131211100F0E0D0C0B0A09080706050403020100;
defparam prom_inst_0.INIT_RAM_01 = 256'h3F3E3D3C3B3A393837363534333231302F2E2D2C2B2A29282726252423222120;
defparam prom_inst_0.INIT_RAM_02 = 256'h5F5E5D5C5B5A595857565554535251504F4E4D4C4B4A49484746454443424140;
defparam prom_inst_0.INIT_RAM_03 = 256'h7F7E7D7C7B7A797877767574737271706F6E6D6C6B6A69686766656463626160;
defparam prom_inst_0.INIT_RAM_04 = 256'h9E9D9C9B9A999897969594939291908F8E8D8C8B8A8988878685848382818080;
defparam prom_inst_0.INIT_RAM_05 = 256'hBEBDBCBBBAB9B8B7B6B5B4B3B2B1B0AFAEADACABAAA9A8A7A6A5A4A3A2A1A09F;
defparam prom_inst_0.INIT_RAM_06 = 256'hDEDDDCDBDAD9D8D7D6D5D4D3D2D1D0CFCECDCCCBCAC9C8C7C6C5C4C3C2C1C0BF;
defparam prom_inst_0.INIT_RAM_07 = 256'hFEFDFCFBFAF9F8F7F6F5F4F3F2F1F0EFEEEDECEBEAE9E8E7E6E5E4E3E2E1E0DF;

endmodule //sawtooth
