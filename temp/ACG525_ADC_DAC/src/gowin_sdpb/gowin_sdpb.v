//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//Tool Version: V1.9.12 (64-bit)
//Part Number: GW5A-LV25UG324C2/I1
//Device: GW5A-25
//Device Version: A
//Created Time: Tue Oct  7 13:56:09 2025

module Gowin_SDPB (dout, clka, cea, clkb, ceb, oce, reset, ada, din, adb);

output [7:0] dout;
input clka;
input cea;
input clkb;
input ceb;
input oce;
input reset;
input [15:0] ada;
input [7:0] din;
input [15:0] adb;

wire [30:0] sdpb_inst_0_dout_w;
wire [0:0] sdpb_inst_0_dout;
wire [30:0] sdpb_inst_1_dout_w;
wire [0:0] sdpb_inst_1_dout;
wire [30:0] sdpb_inst_2_dout_w;
wire [0:0] sdpb_inst_2_dout;
wire [30:0] sdpb_inst_3_dout_w;
wire [0:0] sdpb_inst_3_dout;
wire [30:0] sdpb_inst_4_dout_w;
wire [1:1] sdpb_inst_4_dout;
wire [30:0] sdpb_inst_5_dout_w;
wire [1:1] sdpb_inst_5_dout;
wire [30:0] sdpb_inst_6_dout_w;
wire [1:1] sdpb_inst_6_dout;
wire [30:0] sdpb_inst_7_dout_w;
wire [1:1] sdpb_inst_7_dout;
wire [30:0] sdpb_inst_8_dout_w;
wire [2:2] sdpb_inst_8_dout;
wire [30:0] sdpb_inst_9_dout_w;
wire [2:2] sdpb_inst_9_dout;
wire [30:0] sdpb_inst_10_dout_w;
wire [2:2] sdpb_inst_10_dout;
wire [30:0] sdpb_inst_11_dout_w;
wire [2:2] sdpb_inst_11_dout;
wire [30:0] sdpb_inst_12_dout_w;
wire [3:3] sdpb_inst_12_dout;
wire [30:0] sdpb_inst_13_dout_w;
wire [3:3] sdpb_inst_13_dout;
wire [30:0] sdpb_inst_14_dout_w;
wire [3:3] sdpb_inst_14_dout;
wire [30:0] sdpb_inst_15_dout_w;
wire [3:3] sdpb_inst_15_dout;
wire [30:0] sdpb_inst_16_dout_w;
wire [4:4] sdpb_inst_16_dout;
wire [30:0] sdpb_inst_17_dout_w;
wire [4:4] sdpb_inst_17_dout;
wire [30:0] sdpb_inst_18_dout_w;
wire [4:4] sdpb_inst_18_dout;
wire [30:0] sdpb_inst_19_dout_w;
wire [4:4] sdpb_inst_19_dout;
wire [30:0] sdpb_inst_20_dout_w;
wire [5:5] sdpb_inst_20_dout;
wire [30:0] sdpb_inst_21_dout_w;
wire [5:5] sdpb_inst_21_dout;
wire [30:0] sdpb_inst_22_dout_w;
wire [5:5] sdpb_inst_22_dout;
wire [30:0] sdpb_inst_23_dout_w;
wire [5:5] sdpb_inst_23_dout;
wire [30:0] sdpb_inst_24_dout_w;
wire [6:6] sdpb_inst_24_dout;
wire [30:0] sdpb_inst_25_dout_w;
wire [6:6] sdpb_inst_25_dout;
wire [30:0] sdpb_inst_26_dout_w;
wire [6:6] sdpb_inst_26_dout;
wire [30:0] sdpb_inst_27_dout_w;
wire [6:6] sdpb_inst_27_dout;
wire [30:0] sdpb_inst_28_dout_w;
wire [7:7] sdpb_inst_28_dout;
wire [30:0] sdpb_inst_29_dout_w;
wire [7:7] sdpb_inst_29_dout;
wire [30:0] sdpb_inst_30_dout_w;
wire [7:7] sdpb_inst_30_dout;
wire [30:0] sdpb_inst_31_dout_w;
wire [7:7] sdpb_inst_31_dout;
wire dff_q_0;
wire dff_q_1;
wire mux_o_0;
wire mux_o_1;
wire mux_o_3;
wire mux_o_4;
wire mux_o_6;
wire mux_o_7;
wire mux_o_9;
wire mux_o_10;
wire mux_o_12;
wire mux_o_13;
wire mux_o_15;
wire mux_o_16;
wire mux_o_18;
wire mux_o_19;
wire mux_o_21;
wire mux_o_22;
wire gw_gnd;

assign gw_gnd = 1'b0;

SDPB sdpb_inst_0 (
    .DO({sdpb_inst_0_dout_w[30:0],sdpb_inst_0_dout[0]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[0]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_0.READ_MODE = 1'b0;
defparam sdpb_inst_0.BIT_WIDTH_0 = 1;
defparam sdpb_inst_0.BIT_WIDTH_1 = 1;
defparam sdpb_inst_0.BLK_SEL_0 = 3'b000;
defparam sdpb_inst_0.BLK_SEL_1 = 3'b000;
defparam sdpb_inst_0.RESET_MODE = "SYNC";

SDPB sdpb_inst_1 (
    .DO({sdpb_inst_1_dout_w[30:0],sdpb_inst_1_dout[0]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[0]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_1.READ_MODE = 1'b0;
defparam sdpb_inst_1.BIT_WIDTH_0 = 1;
defparam sdpb_inst_1.BIT_WIDTH_1 = 1;
defparam sdpb_inst_1.BLK_SEL_0 = 3'b001;
defparam sdpb_inst_1.BLK_SEL_1 = 3'b001;
defparam sdpb_inst_1.RESET_MODE = "SYNC";

SDPB sdpb_inst_2 (
    .DO({sdpb_inst_2_dout_w[30:0],sdpb_inst_2_dout[0]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[0]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_2.READ_MODE = 1'b0;
defparam sdpb_inst_2.BIT_WIDTH_0 = 1;
defparam sdpb_inst_2.BIT_WIDTH_1 = 1;
defparam sdpb_inst_2.BLK_SEL_0 = 3'b010;
defparam sdpb_inst_2.BLK_SEL_1 = 3'b010;
defparam sdpb_inst_2.RESET_MODE = "SYNC";

SDPB sdpb_inst_3 (
    .DO({sdpb_inst_3_dout_w[30:0],sdpb_inst_3_dout[0]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[0]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_3.READ_MODE = 1'b0;
defparam sdpb_inst_3.BIT_WIDTH_0 = 1;
defparam sdpb_inst_3.BIT_WIDTH_1 = 1;
defparam sdpb_inst_3.BLK_SEL_0 = 3'b011;
defparam sdpb_inst_3.BLK_SEL_1 = 3'b011;
defparam sdpb_inst_3.RESET_MODE = "SYNC";

SDPB sdpb_inst_4 (
    .DO({sdpb_inst_4_dout_w[30:0],sdpb_inst_4_dout[1]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[1]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_4.READ_MODE = 1'b0;
defparam sdpb_inst_4.BIT_WIDTH_0 = 1;
defparam sdpb_inst_4.BIT_WIDTH_1 = 1;
defparam sdpb_inst_4.BLK_SEL_0 = 3'b000;
defparam sdpb_inst_4.BLK_SEL_1 = 3'b000;
defparam sdpb_inst_4.RESET_MODE = "SYNC";

SDPB sdpb_inst_5 (
    .DO({sdpb_inst_5_dout_w[30:0],sdpb_inst_5_dout[1]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[1]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_5.READ_MODE = 1'b0;
defparam sdpb_inst_5.BIT_WIDTH_0 = 1;
defparam sdpb_inst_5.BIT_WIDTH_1 = 1;
defparam sdpb_inst_5.BLK_SEL_0 = 3'b001;
defparam sdpb_inst_5.BLK_SEL_1 = 3'b001;
defparam sdpb_inst_5.RESET_MODE = "SYNC";

SDPB sdpb_inst_6 (
    .DO({sdpb_inst_6_dout_w[30:0],sdpb_inst_6_dout[1]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[1]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_6.READ_MODE = 1'b0;
defparam sdpb_inst_6.BIT_WIDTH_0 = 1;
defparam sdpb_inst_6.BIT_WIDTH_1 = 1;
defparam sdpb_inst_6.BLK_SEL_0 = 3'b010;
defparam sdpb_inst_6.BLK_SEL_1 = 3'b010;
defparam sdpb_inst_6.RESET_MODE = "SYNC";

SDPB sdpb_inst_7 (
    .DO({sdpb_inst_7_dout_w[30:0],sdpb_inst_7_dout[1]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[1]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_7.READ_MODE = 1'b0;
defparam sdpb_inst_7.BIT_WIDTH_0 = 1;
defparam sdpb_inst_7.BIT_WIDTH_1 = 1;
defparam sdpb_inst_7.BLK_SEL_0 = 3'b011;
defparam sdpb_inst_7.BLK_SEL_1 = 3'b011;
defparam sdpb_inst_7.RESET_MODE = "SYNC";

SDPB sdpb_inst_8 (
    .DO({sdpb_inst_8_dout_w[30:0],sdpb_inst_8_dout[2]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[2]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_8.READ_MODE = 1'b0;
defparam sdpb_inst_8.BIT_WIDTH_0 = 1;
defparam sdpb_inst_8.BIT_WIDTH_1 = 1;
defparam sdpb_inst_8.BLK_SEL_0 = 3'b000;
defparam sdpb_inst_8.BLK_SEL_1 = 3'b000;
defparam sdpb_inst_8.RESET_MODE = "SYNC";

SDPB sdpb_inst_9 (
    .DO({sdpb_inst_9_dout_w[30:0],sdpb_inst_9_dout[2]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[2]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_9.READ_MODE = 1'b0;
defparam sdpb_inst_9.BIT_WIDTH_0 = 1;
defparam sdpb_inst_9.BIT_WIDTH_1 = 1;
defparam sdpb_inst_9.BLK_SEL_0 = 3'b001;
defparam sdpb_inst_9.BLK_SEL_1 = 3'b001;
defparam sdpb_inst_9.RESET_MODE = "SYNC";

SDPB sdpb_inst_10 (
    .DO({sdpb_inst_10_dout_w[30:0],sdpb_inst_10_dout[2]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[2]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_10.READ_MODE = 1'b0;
defparam sdpb_inst_10.BIT_WIDTH_0 = 1;
defparam sdpb_inst_10.BIT_WIDTH_1 = 1;
defparam sdpb_inst_10.BLK_SEL_0 = 3'b010;
defparam sdpb_inst_10.BLK_SEL_1 = 3'b010;
defparam sdpb_inst_10.RESET_MODE = "SYNC";

SDPB sdpb_inst_11 (
    .DO({sdpb_inst_11_dout_w[30:0],sdpb_inst_11_dout[2]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[2]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_11.READ_MODE = 1'b0;
defparam sdpb_inst_11.BIT_WIDTH_0 = 1;
defparam sdpb_inst_11.BIT_WIDTH_1 = 1;
defparam sdpb_inst_11.BLK_SEL_0 = 3'b011;
defparam sdpb_inst_11.BLK_SEL_1 = 3'b011;
defparam sdpb_inst_11.RESET_MODE = "SYNC";

SDPB sdpb_inst_12 (
    .DO({sdpb_inst_12_dout_w[30:0],sdpb_inst_12_dout[3]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[3]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_12.READ_MODE = 1'b0;
defparam sdpb_inst_12.BIT_WIDTH_0 = 1;
defparam sdpb_inst_12.BIT_WIDTH_1 = 1;
defparam sdpb_inst_12.BLK_SEL_0 = 3'b000;
defparam sdpb_inst_12.BLK_SEL_1 = 3'b000;
defparam sdpb_inst_12.RESET_MODE = "SYNC";

SDPB sdpb_inst_13 (
    .DO({sdpb_inst_13_dout_w[30:0],sdpb_inst_13_dout[3]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[3]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_13.READ_MODE = 1'b0;
defparam sdpb_inst_13.BIT_WIDTH_0 = 1;
defparam sdpb_inst_13.BIT_WIDTH_1 = 1;
defparam sdpb_inst_13.BLK_SEL_0 = 3'b001;
defparam sdpb_inst_13.BLK_SEL_1 = 3'b001;
defparam sdpb_inst_13.RESET_MODE = "SYNC";

SDPB sdpb_inst_14 (
    .DO({sdpb_inst_14_dout_w[30:0],sdpb_inst_14_dout[3]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[3]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_14.READ_MODE = 1'b0;
defparam sdpb_inst_14.BIT_WIDTH_0 = 1;
defparam sdpb_inst_14.BIT_WIDTH_1 = 1;
defparam sdpb_inst_14.BLK_SEL_0 = 3'b010;
defparam sdpb_inst_14.BLK_SEL_1 = 3'b010;
defparam sdpb_inst_14.RESET_MODE = "SYNC";

SDPB sdpb_inst_15 (
    .DO({sdpb_inst_15_dout_w[30:0],sdpb_inst_15_dout[3]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[3]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_15.READ_MODE = 1'b0;
defparam sdpb_inst_15.BIT_WIDTH_0 = 1;
defparam sdpb_inst_15.BIT_WIDTH_1 = 1;
defparam sdpb_inst_15.BLK_SEL_0 = 3'b011;
defparam sdpb_inst_15.BLK_SEL_1 = 3'b011;
defparam sdpb_inst_15.RESET_MODE = "SYNC";

SDPB sdpb_inst_16 (
    .DO({sdpb_inst_16_dout_w[30:0],sdpb_inst_16_dout[4]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[4]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_16.READ_MODE = 1'b0;
defparam sdpb_inst_16.BIT_WIDTH_0 = 1;
defparam sdpb_inst_16.BIT_WIDTH_1 = 1;
defparam sdpb_inst_16.BLK_SEL_0 = 3'b000;
defparam sdpb_inst_16.BLK_SEL_1 = 3'b000;
defparam sdpb_inst_16.RESET_MODE = "SYNC";

SDPB sdpb_inst_17 (
    .DO({sdpb_inst_17_dout_w[30:0],sdpb_inst_17_dout[4]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[4]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_17.READ_MODE = 1'b0;
defparam sdpb_inst_17.BIT_WIDTH_0 = 1;
defparam sdpb_inst_17.BIT_WIDTH_1 = 1;
defparam sdpb_inst_17.BLK_SEL_0 = 3'b001;
defparam sdpb_inst_17.BLK_SEL_1 = 3'b001;
defparam sdpb_inst_17.RESET_MODE = "SYNC";

SDPB sdpb_inst_18 (
    .DO({sdpb_inst_18_dout_w[30:0],sdpb_inst_18_dout[4]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[4]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_18.READ_MODE = 1'b0;
defparam sdpb_inst_18.BIT_WIDTH_0 = 1;
defparam sdpb_inst_18.BIT_WIDTH_1 = 1;
defparam sdpb_inst_18.BLK_SEL_0 = 3'b010;
defparam sdpb_inst_18.BLK_SEL_1 = 3'b010;
defparam sdpb_inst_18.RESET_MODE = "SYNC";

SDPB sdpb_inst_19 (
    .DO({sdpb_inst_19_dout_w[30:0],sdpb_inst_19_dout[4]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[4]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_19.READ_MODE = 1'b0;
defparam sdpb_inst_19.BIT_WIDTH_0 = 1;
defparam sdpb_inst_19.BIT_WIDTH_1 = 1;
defparam sdpb_inst_19.BLK_SEL_0 = 3'b011;
defparam sdpb_inst_19.BLK_SEL_1 = 3'b011;
defparam sdpb_inst_19.RESET_MODE = "SYNC";

SDPB sdpb_inst_20 (
    .DO({sdpb_inst_20_dout_w[30:0],sdpb_inst_20_dout[5]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[5]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_20.READ_MODE = 1'b0;
defparam sdpb_inst_20.BIT_WIDTH_0 = 1;
defparam sdpb_inst_20.BIT_WIDTH_1 = 1;
defparam sdpb_inst_20.BLK_SEL_0 = 3'b000;
defparam sdpb_inst_20.BLK_SEL_1 = 3'b000;
defparam sdpb_inst_20.RESET_MODE = "SYNC";

SDPB sdpb_inst_21 (
    .DO({sdpb_inst_21_dout_w[30:0],sdpb_inst_21_dout[5]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[5]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_21.READ_MODE = 1'b0;
defparam sdpb_inst_21.BIT_WIDTH_0 = 1;
defparam sdpb_inst_21.BIT_WIDTH_1 = 1;
defparam sdpb_inst_21.BLK_SEL_0 = 3'b001;
defparam sdpb_inst_21.BLK_SEL_1 = 3'b001;
defparam sdpb_inst_21.RESET_MODE = "SYNC";

SDPB sdpb_inst_22 (
    .DO({sdpb_inst_22_dout_w[30:0],sdpb_inst_22_dout[5]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[5]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_22.READ_MODE = 1'b0;
defparam sdpb_inst_22.BIT_WIDTH_0 = 1;
defparam sdpb_inst_22.BIT_WIDTH_1 = 1;
defparam sdpb_inst_22.BLK_SEL_0 = 3'b010;
defparam sdpb_inst_22.BLK_SEL_1 = 3'b010;
defparam sdpb_inst_22.RESET_MODE = "SYNC";

SDPB sdpb_inst_23 (
    .DO({sdpb_inst_23_dout_w[30:0],sdpb_inst_23_dout[5]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[5]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_23.READ_MODE = 1'b0;
defparam sdpb_inst_23.BIT_WIDTH_0 = 1;
defparam sdpb_inst_23.BIT_WIDTH_1 = 1;
defparam sdpb_inst_23.BLK_SEL_0 = 3'b011;
defparam sdpb_inst_23.BLK_SEL_1 = 3'b011;
defparam sdpb_inst_23.RESET_MODE = "SYNC";

SDPB sdpb_inst_24 (
    .DO({sdpb_inst_24_dout_w[30:0],sdpb_inst_24_dout[6]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[6]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_24.READ_MODE = 1'b0;
defparam sdpb_inst_24.BIT_WIDTH_0 = 1;
defparam sdpb_inst_24.BIT_WIDTH_1 = 1;
defparam sdpb_inst_24.BLK_SEL_0 = 3'b000;
defparam sdpb_inst_24.BLK_SEL_1 = 3'b000;
defparam sdpb_inst_24.RESET_MODE = "SYNC";

SDPB sdpb_inst_25 (
    .DO({sdpb_inst_25_dout_w[30:0],sdpb_inst_25_dout[6]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[6]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_25.READ_MODE = 1'b0;
defparam sdpb_inst_25.BIT_WIDTH_0 = 1;
defparam sdpb_inst_25.BIT_WIDTH_1 = 1;
defparam sdpb_inst_25.BLK_SEL_0 = 3'b001;
defparam sdpb_inst_25.BLK_SEL_1 = 3'b001;
defparam sdpb_inst_25.RESET_MODE = "SYNC";

SDPB sdpb_inst_26 (
    .DO({sdpb_inst_26_dout_w[30:0],sdpb_inst_26_dout[6]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[6]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_26.READ_MODE = 1'b0;
defparam sdpb_inst_26.BIT_WIDTH_0 = 1;
defparam sdpb_inst_26.BIT_WIDTH_1 = 1;
defparam sdpb_inst_26.BLK_SEL_0 = 3'b010;
defparam sdpb_inst_26.BLK_SEL_1 = 3'b010;
defparam sdpb_inst_26.RESET_MODE = "SYNC";

SDPB sdpb_inst_27 (
    .DO({sdpb_inst_27_dout_w[30:0],sdpb_inst_27_dout[6]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[6]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_27.READ_MODE = 1'b0;
defparam sdpb_inst_27.BIT_WIDTH_0 = 1;
defparam sdpb_inst_27.BIT_WIDTH_1 = 1;
defparam sdpb_inst_27.BLK_SEL_0 = 3'b011;
defparam sdpb_inst_27.BLK_SEL_1 = 3'b011;
defparam sdpb_inst_27.RESET_MODE = "SYNC";

SDPB sdpb_inst_28 (
    .DO({sdpb_inst_28_dout_w[30:0],sdpb_inst_28_dout[7]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[7]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_28.READ_MODE = 1'b0;
defparam sdpb_inst_28.BIT_WIDTH_0 = 1;
defparam sdpb_inst_28.BIT_WIDTH_1 = 1;
defparam sdpb_inst_28.BLK_SEL_0 = 3'b000;
defparam sdpb_inst_28.BLK_SEL_1 = 3'b000;
defparam sdpb_inst_28.RESET_MODE = "SYNC";

SDPB sdpb_inst_29 (
    .DO({sdpb_inst_29_dout_w[30:0],sdpb_inst_29_dout[7]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[7]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_29.READ_MODE = 1'b0;
defparam sdpb_inst_29.BIT_WIDTH_0 = 1;
defparam sdpb_inst_29.BIT_WIDTH_1 = 1;
defparam sdpb_inst_29.BLK_SEL_0 = 3'b001;
defparam sdpb_inst_29.BLK_SEL_1 = 3'b001;
defparam sdpb_inst_29.RESET_MODE = "SYNC";

SDPB sdpb_inst_30 (
    .DO({sdpb_inst_30_dout_w[30:0],sdpb_inst_30_dout[7]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[7]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_30.READ_MODE = 1'b0;
defparam sdpb_inst_30.BIT_WIDTH_0 = 1;
defparam sdpb_inst_30.BIT_WIDTH_1 = 1;
defparam sdpb_inst_30.BLK_SEL_0 = 3'b010;
defparam sdpb_inst_30.BLK_SEL_1 = 3'b010;
defparam sdpb_inst_30.RESET_MODE = "SYNC";

SDPB sdpb_inst_31 (
    .DO({sdpb_inst_31_dout_w[30:0],sdpb_inst_31_dout[7]}),
    .CLKA(clka),
    .CEA(cea),
    .CLKB(clkb),
    .CEB(ceb),
    .OCE(oce),
    .RESET(reset),
    .BLKSELA({gw_gnd,ada[15],ada[14]}),
    .BLKSELB({gw_gnd,adb[15],adb[14]}),
    .ADA(ada[13:0]),
    .DI({gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,gw_gnd,din[7]}),
    .ADB(adb[13:0])
);

defparam sdpb_inst_31.READ_MODE = 1'b0;
defparam sdpb_inst_31.BIT_WIDTH_0 = 1;
defparam sdpb_inst_31.BIT_WIDTH_1 = 1;
defparam sdpb_inst_31.BLK_SEL_0 = 3'b011;
defparam sdpb_inst_31.BLK_SEL_1 = 3'b011;
defparam sdpb_inst_31.RESET_MODE = "SYNC";

DFFRE dff_inst_0 (
  .Q(dff_q_0),
  .D(adb[15]),
  .CLK(clkb),
  .CE(ceb),
  .RESET(gw_gnd)
);
DFFRE dff_inst_1 (
  .Q(dff_q_1),
  .D(adb[14]),
  .CLK(clkb),
  .CE(ceb),
  .RESET(gw_gnd)
);
MUX2 mux_inst_0 (
  .O(mux_o_0),
  .I0(sdpb_inst_0_dout[0]),
  .I1(sdpb_inst_1_dout[0]),
  .S0(dff_q_1)
);
MUX2 mux_inst_1 (
  .O(mux_o_1),
  .I0(sdpb_inst_2_dout[0]),
  .I1(sdpb_inst_3_dout[0]),
  .S0(dff_q_1)
);
MUX2 mux_inst_2 (
  .O(dout[0]),
  .I0(mux_o_0),
  .I1(mux_o_1),
  .S0(dff_q_0)
);
MUX2 mux_inst_3 (
  .O(mux_o_3),
  .I0(sdpb_inst_4_dout[1]),
  .I1(sdpb_inst_5_dout[1]),
  .S0(dff_q_1)
);
MUX2 mux_inst_4 (
  .O(mux_o_4),
  .I0(sdpb_inst_6_dout[1]),
  .I1(sdpb_inst_7_dout[1]),
  .S0(dff_q_1)
);
MUX2 mux_inst_5 (
  .O(dout[1]),
  .I0(mux_o_3),
  .I1(mux_o_4),
  .S0(dff_q_0)
);
MUX2 mux_inst_6 (
  .O(mux_o_6),
  .I0(sdpb_inst_8_dout[2]),
  .I1(sdpb_inst_9_dout[2]),
  .S0(dff_q_1)
);
MUX2 mux_inst_7 (
  .O(mux_o_7),
  .I0(sdpb_inst_10_dout[2]),
  .I1(sdpb_inst_11_dout[2]),
  .S0(dff_q_1)
);
MUX2 mux_inst_8 (
  .O(dout[2]),
  .I0(mux_o_6),
  .I1(mux_o_7),
  .S0(dff_q_0)
);
MUX2 mux_inst_9 (
  .O(mux_o_9),
  .I0(sdpb_inst_12_dout[3]),
  .I1(sdpb_inst_13_dout[3]),
  .S0(dff_q_1)
);
MUX2 mux_inst_10 (
  .O(mux_o_10),
  .I0(sdpb_inst_14_dout[3]),
  .I1(sdpb_inst_15_dout[3]),
  .S0(dff_q_1)
);
MUX2 mux_inst_11 (
  .O(dout[3]),
  .I0(mux_o_9),
  .I1(mux_o_10),
  .S0(dff_q_0)
);
MUX2 mux_inst_12 (
  .O(mux_o_12),
  .I0(sdpb_inst_16_dout[4]),
  .I1(sdpb_inst_17_dout[4]),
  .S0(dff_q_1)
);
MUX2 mux_inst_13 (
  .O(mux_o_13),
  .I0(sdpb_inst_18_dout[4]),
  .I1(sdpb_inst_19_dout[4]),
  .S0(dff_q_1)
);
MUX2 mux_inst_14 (
  .O(dout[4]),
  .I0(mux_o_12),
  .I1(mux_o_13),
  .S0(dff_q_0)
);
MUX2 mux_inst_15 (
  .O(mux_o_15),
  .I0(sdpb_inst_20_dout[5]),
  .I1(sdpb_inst_21_dout[5]),
  .S0(dff_q_1)
);
MUX2 mux_inst_16 (
  .O(mux_o_16),
  .I0(sdpb_inst_22_dout[5]),
  .I1(sdpb_inst_23_dout[5]),
  .S0(dff_q_1)
);
MUX2 mux_inst_17 (
  .O(dout[5]),
  .I0(mux_o_15),
  .I1(mux_o_16),
  .S0(dff_q_0)
);
MUX2 mux_inst_18 (
  .O(mux_o_18),
  .I0(sdpb_inst_24_dout[6]),
  .I1(sdpb_inst_25_dout[6]),
  .S0(dff_q_1)
);
MUX2 mux_inst_19 (
  .O(mux_o_19),
  .I0(sdpb_inst_26_dout[6]),
  .I1(sdpb_inst_27_dout[6]),
  .S0(dff_q_1)
);
MUX2 mux_inst_20 (
  .O(dout[6]),
  .I0(mux_o_18),
  .I1(mux_o_19),
  .S0(dff_q_0)
);
MUX2 mux_inst_21 (
  .O(mux_o_21),
  .I0(sdpb_inst_28_dout[7]),
  .I1(sdpb_inst_29_dout[7]),
  .S0(dff_q_1)
);
MUX2 mux_inst_22 (
  .O(mux_o_22),
  .I0(sdpb_inst_30_dout[7]),
  .I1(sdpb_inst_31_dout[7]),
  .S0(dff_q_1)
);
MUX2 mux_inst_23 (
  .O(dout[7]),
  .I0(mux_o_21),
  .I1(mux_o_22),
  .S0(dff_q_0)
);
endmodule //Gowin_SDPB
