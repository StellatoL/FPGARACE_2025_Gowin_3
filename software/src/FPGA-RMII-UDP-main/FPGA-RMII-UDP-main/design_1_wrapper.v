//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2023.2.2 (win64) Build 4126759 Thu Feb  8 23:53:51 MST 2024
//Date        : Sat Feb  8 18:57:59 2025
//Host        : DESKTOP-BGESMJR running 64-bit major release  (build 9200)
//Command     : generate_target design_1_wrapper.bd
//Design      : design_1_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module design_1_wrapper
   (CLK100MHZ,
    ETH_CRSDV,
    ETH_REFCLK,
    ETH_RXD,
    ETH_RXERR,
    ETH_TXD,
    ETH_TXEN,
    LED,
    RESET_N,
    SW,
    UART_RXD_OUT);
  input CLK100MHZ;
  input ETH_CRSDV;
  output ETH_REFCLK;
  input [1:0]ETH_RXD;
  input ETH_RXERR;
  output [1:0]ETH_TXD;
  output ETH_TXEN;
  output [15:0]LED;
  input RESET_N;
  input [15:0]SW;
  output UART_RXD_OUT;

  wire CLK100MHZ;
  wire ETH_CRSDV;
  wire ETH_REFCLK;
  wire [1:0]ETH_RXD;
  wire ETH_RXERR;
  wire [1:0]ETH_TXD;
  wire ETH_TXEN;
  wire [15:0]LED;
  wire RESET_N;
  wire [15:0]SW;
  wire UART_RXD_OUT;

  design_1 design_1_i
       (.CLK100MHZ(CLK100MHZ),
        .ETH_CRSDV(ETH_CRSDV),
        .ETH_REFCLK(ETH_REFCLK),
        .ETH_RXD(ETH_RXD),
        .ETH_RXERR(ETH_RXERR),
        .ETH_TXD(ETH_TXD),
        .ETH_TXEN(ETH_TXEN),
        .LED(LED),
        .RESET_N(RESET_N),
        .SW(SW),
        .UART_RXD_OUT(UART_RXD_OUT));
endmodule
