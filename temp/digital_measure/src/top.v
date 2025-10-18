module top #(
    parameter CLK_FREQ = 50_000_000
)(
    input clk,
    input rst_n,

    input extern_clk,

    output uart_txd
);

assign uart_tx_data_size = 16;
assign uart_tx_data = {extern_freq,extern_high_cycles,extern_low_cycles,extern_duty, 16'h0d0a};
assign uart_send_en = extern_done;

// ===================串口发送模块=====================
parameter UART_BAUD     = 115200;
parameter MAX_TX_DATA_WIDTH = 8*16;

wire uart_send_en;
wire [MAX_TX_DATA_WIDTH - 1 :0] uart_tx_data;
wire [7:0] uart_tx_data_size;

// 串口发送实例化
uart_data_tx
#(
    .CLK_FREQ(CLK_FREQ),
    .UART_BAUD(UART_BAUD),
	.MAX_DATA_WIDTH(MAX_TX_DATA_WIDTH),
	.MSB_FIRST(1'b1)
)
uart_data_tx_inst(
	.Clk(clk),
	.Rst_n(rst_n),
	.data(uart_tx_data),
	.send_en(uart_send_en),
    .tx_data_size(uart_tx_data_size),
	.uart_tx(uart_txd),
	.Tx_Done(),
	.uart_state()
);

// ==================频率测量模块=======================
wire [31:0] extern_freq;
wire [31:0] extern_high_cycles;
wire [31:0] extern_low_cycles;
wire [15:0] extern_duty;

wire extern_done;

cymometer_equal #(
    .CLK_FS(200_000_000)
) cymometer_equal_inst(
    .clk_fs(PLL_200M),
    .rst_n(rst_n),
    .clk_fx(extern_clk),
    .GATE_TIME(75),
    .frequence(extern_freq),
    // .high_cycles(extern_high_cycles),
    // .low_cycles(extern_low_cycles),
    .duty_permille(extern_duty),
    .measure_done(extern_done)
);

// ==================锁相环模块=========================
wire PLL_200M;

Gowin_rPLL Gowin_rPLL_inst(
    .clkin(clk),
    .reset(~rst_n),
    .clkout(PLL_200M)
);


endmodule


