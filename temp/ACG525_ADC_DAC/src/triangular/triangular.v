//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//Tool Version: V1.9.12 (64-bit)
//Part Number: GW5A-LV25UG324C2/I1
//Device: GW5A-25
//Device Version: A
//Created Time: Tue Oct  7 14:59:04 2025

module triangular (dout, clk, oce, ce, reset, ad);

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
defparam prom_inst_0.INIT_RAM_00 = 256'h3E3C3A38363432302E2C2A28262422201E1C1A18161412100E0C0A0806040200;
defparam prom_inst_0.INIT_RAM_01 = 256'h7E7C7A78767472706E6C6A68666462605E5C5A58565452504E4C4A4846444240;
defparam prom_inst_0.INIT_RAM_02 = 256'hBDBBB9B7B5B3B1AFADABA9A7A5A3A19F9D9B99979593918F8D8B898785838180;
defparam prom_inst_0.INIT_RAM_03 = 256'hFDFBF9F7F5F3F1EFEDEBE9E7E5E3E1DFDDDBD9D7D5D3D1CFCDCBC9C7C5C3C1BF;
defparam prom_inst_0.INIT_RAM_04 = 256'hC1C3C5C7C9CBCDCFD1D3D5D7D9DBDDDFE1E3E5E7E9EBEDEFF1F3F5F7F9FBFDFF;
defparam prom_inst_0.INIT_RAM_05 = 256'h81838587898B8D8F91939597999B9D9FA1A3A5A7A9ABADAFB1B3B5B7B9BBBDBF;
defparam prom_inst_0.INIT_RAM_06 = 256'h424446484A4C4E50525456585A5C5E60626466686A6C6E70727476787A7C7E80;
defparam prom_inst_0.INIT_RAM_07 = 256'h020406080A0C0E10121416181A1C1E20222426282A2C2E30323436383A3C3E40;

endmodule //triangular
