module ACG525_top #(
    parameter CLK_FREQ  = 50_000_000,
    parameter CLK_FS    = 200_000_000,

    parameter UART_BAUD = 115200,
    parameter MAX_RX_DATA_SIZE = 25,

    parameter I2C_FREQ  = 100_000,
    parameter IIC_DATA_BYTE_NUM = 16,

    parameter SPI_FREQ  = 400_000,
    parameter SPI_MAX_BYTES = 16
)(
    input clk,
    input rst_n,

    // 串口相关端口
    output uart_txd,
    input  uart_rxd,

    output uart_txd2,

    // I2C相关端口
    output scl,
    inout  sda,

    // SPI相关端口
    output wire spi_cs,
    output wire spi_clk,
    output wire spi_mosi,
    input  wire spi_miso,

    // PWM相关端口
    output [3:0] PWM,

    // CAN相关端口
    input  wire can_rxd,
    output wire can_txd,

    // 数字电路测量端口
    input extern_clk
);



















assign uart_tx_data_size = 10;
assign uart_tx_data = {extern_freq,8'h0,6'b0,extern_duty, 16'h0d0a};
assign uart_send_en = extern_done;

// ===================串口发送模块=====================

parameter MAX_TX_DATA_WIDTH2 = MAX_RX_DATA_SIZE * 8- 8;
wire uart_send_en;
wire [MAX_TX_DATA_WIDTH2 - 1 :0] uart_tx_data;
wire [7:0] uart_tx_data_size;

// 串口发送实例化
uart_data_tx
#(
    .CLK_FREQ(CLK_FREQ),
    .UART_BAUD(UART_BAUD),
	.MAX_DATA_WIDTH(MAX_TX_DATA_WIDTH2),
	.MSB_FIRST(1'b1)
) uart_data_tx_inst2(
	.Clk(clk),
	.Rst_n(rst_n),
	.data(uart_tx_data),
	.send_en(uart_send_en),
    .tx_data_size(uart_tx_data_size),
	.uart_tx(uart_txd2),
	.Tx_Done(),
	.uart_state()
);

// =======================uart2others模块=======================

uart #(
    .CLK_FREQ(CLK_FREQ),
    
    .UART_BAUD(UART_BAUD),
    .MAX_RX_DATA_SIZE(MAX_RX_DATA_SIZE),

    .I2C_FREQ(I2C_FREQ),
    .IIC_DATA_BYTE_NUM(IIC_DATA_BYTE_NUM),

    .SPI_FREQ(SPI_FREQ),
    .SPI_MAX_BYTES(SPI_MAX_BYTES),
    .SPI_DATA_SIDE("MSB"),
    .CPOL(0),
    .CPHA(0)
)uart_inst(
    .clk(clk),
    .rst_n(rst_n),

    // 串口相关端口
    .uart_txd(uart_txd),
    .uart_rxd(uart_rxd),

    // I2C相关端口
    .scl(scl),
    .sda(sda),

    // SPI相关端口
    .spi_cs(spi_cs),
    .spi_clk(spi_clk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),

    // CAN相关端口
    .can_rx(can_rxd),
    .can_tx(can_txd),

    // PWM相关端口
    .PWM(PWM)
);

// ======================测量频率、占空比模块=========================
wire [31:0] extern_freq;
wire [31:0] extern_high_cycles;
wire [31:0] extern_low_cycles;
wire [ 9:0] extern_duty;

wire extern_done;
digital_measure #(
//    .CLK_FREQ(CLK_FREQ),
    .CLK_FS(50_000_000)
) digital_measure_top_inst(
    .clk_fs(clk),
    .rst_n(rst_n),
    .clk_fx(extern_clk),
    .digital_frequency(extern_freq),

    .digital_duty(extern_duty),
    .digital_done(extern_done)
);


// ==================锁相环模块=========================
wire PLL_200M;

Gowin_PLL Gowin_PLL_inst(
    .clkin(clk),
    .reset(~rst_n),
    .clkout0(PLL_200M)
);




endmodule
