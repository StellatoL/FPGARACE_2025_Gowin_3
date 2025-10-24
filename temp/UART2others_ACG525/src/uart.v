module uart(
    input clk,
    input rst_n,

    // 串口相关端口
    output uart_txd,
    input  uart_rxd,

    // I2C相关端口
    output scl,
    inout  sda,

    // SPI相关端口
    output wire spi_cs,
    output wire spi_clk,
    output wire spi_mosi,
    input  wire spi_miso,

    // CAN相关端口
    input  wire can_rx,
    output wire can_tx,

    // PWM相关端口
    output [3:0] PWM
);

parameter CLK_FREQ      = 50_000_000;
parameter UART_BAUD     = 115200;

//************************串口接收部分***************************//
parameter MAX_RX_DATA_SIZE = 25;

wire uart_rx_done;
wire [MAX_RX_DATA_SIZE * 8 - 1:0] uart_rx_packet;
wire [7:0] rx_data_size;

uart_packet_rx #(
    .CLK_FREQ(CLK_FREQ),
    .UART_BAUD(UART_BAUD),
    .MAX_DATA_SIZE(MAX_RX_DATA_SIZE)
) uart_packet_rx_inst (
    .clk(clk),
    .rst_n(rst_n),
    .uart_rxd(uart_rxd),
    .uart_rx_done(uart_rx_done),
    .uart_rx_packet(uart_rx_packet),
    .rx_data_size(rx_data_size)
);

//***************************************************************//
//************************数据类型转换部分*************************//
//**************************************************************//
wire [2:0] packet_type;

uart_rx_packet_trans #(
    .MAX_RX_DATA_SIZE(MAX_RX_DATA_SIZE)
) uart_rx_packet_trans_inst (
    .clk(clk),
    .rst_n(rst_n),
    .uart_rx_packet(uart_rx_packet),
    .rx_data_size(rx_data_size),
    .packet_type(packet_type)
);

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        uart_send_en <= 1'b0;
    end
    else begin
        case(packet_type)
            3'd0: begin // UART数据处理喵
                uart_send_en <= uart_rx_done;
                uart_tx_data <= uart_rx_packet[MAX_RX_DATA_SIZE * 8 - 9:0];
                uart_tx_data_size <= rx_data_size - 1;
            end
            3'd1: begin // I2C数据处理喵
                I2C_start <= uart_rx_done;
                uart_send_en <= IIC_done;
                if (IIC_issend) begin
                    uart_tx_data <= {"IIC send OK\n"};
                    uart_tx_data_size <= 12;
                end
                else begin
                    uart_tx_data <= IIC_rdata >> (IIC_DATA_BYTE_NUM - IIC_rx_size) * 8;
                    uart_tx_data_size <= IIC_rx_size;
                end
            end
            3'd2: begin // SPI数据处理喵
                SPI_start <= uart_rx_done;
                spi_tx_data <= uart_rx_packet << (SPI_MAX_BYTES - rx_data_size + 1) * 8;
                uart_send_en <= spi_valid;
                // SPI发送数据位
                if(uart_rx_packet[rx_data_size * 8 - 5]) begin
                    uart_tx_data <= {"SPI send OK\n"};
                    uart_tx_data_size <= 12;
                end
                else begin
                    uart_tx_data <= spi_rx_data >> (SPI_MAX_BYTES - rx_data_size + 1)*8;
                    uart_tx_data_size <= rx_data_size - 1;
                end
            end
            3'd3: begin // PWM数据处理喵
                PWM_start <= uart_rx_done;
                uart_send_en <= PWM_error;
                uart_tx_data <= PWM_error;
                uart_tx_data_size <= 1;
            end
            3'd4: begin
                // CAN数据处理喵
                if(uart_rx_packet[rx_data_size * 8 - 4]) begin
                    if(uart_rx_packet[rx_data_size * 8 - 5]) begin
                        // 扩展帧相关代码（未完成）
                    end
                    else begin
                        // 标准帧相关代码
                        can_local_id <= uart_rx_packet[74:64];
                        can_tx_data <= uart_rx_packet[63:0];
                        can_tx_start <= uart_rx_done;
                        uart_send_en <= uart_rx_done;
                        uart_tx_data <= {"CAN send OK\n"};
                        uart_tx_data_size <= 12;
                    end
                end
                else begin
                    can_rx_ready <= 1'b1;
                    uart_send_en <= uart_rx_done;
                    uart_tx_data <= {can_rx_id,can_rx_data};
                    uart_tx_data_size <= 6 + 8;
                end
            end
            default: begin
                // 默认按照UART数据处理喵
                uart_send_en <= uart_rx_done;
                uart_tx_data <= uart_rx_packet[MAX_RX_DATA_SIZE * 8 - 9:0];
            end
        endcase
    end
end

//************************串口发送部分***************************//
localparam MAX_TX_DATA_WIDTH = MAX_RX_DATA_SIZE * 8 - 8;

reg uart_send_en;
reg [MAX_TX_DATA_WIDTH - 1 :0] uart_tx_data;
reg [7:0] uart_tx_data_size;

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

//*************************I2C部分***************************//
parameter I2C_FREQ = 100_000;
parameter IIC_DATA_BYTE_NUM = 16;

reg I2C_start;
wire IIC_done;
wire [IIC_DATA_BYTE_NUM * 8 - 1:0] IIC_rdata;
wire [7:0] IIC_rx_size;
wire IIC_issend;
wire ack_flag;

packet2IIC #(
    .CLK_FREQ(CLK_FREQ),
    .I2C_FREQ(I2C_FREQ),
    .MAX_RX_DATA_SIZE(MAX_RX_DATA_SIZE),
    .DATA_BYTE_NUM(IIC_DATA_BYTE_NUM)
)
packet2IIC_inst(
    .clk(clk),
    .rst_n(rst_n),
    
    .uart_rx_packet(uart_rx_packet),
    .rx_data_size(rx_data_size),
    .trans_start(I2C_start),

    .rdata(IIC_rdata),
    .IIC_rx_size(IIC_rx_size),
    .IIC_done(IIC_done),
    .issend(IIC_issend),
    .ack_flag(ack_flag),

    .scl(scl),
    .sda(sda)
);

//*************************SPI部分***************************//
parameter SPI_FREQ      = 400_000;
parameter SPI_MAX_BYTES = 16;
parameter SPI_DATA_SIDE = "MSB";
parameter CPOL = 0;
parameter CPHA = 0;

reg  SPI_start;
reg  [SPI_MAX_BYTES * 8 - 1:0] spi_tx_data;
wire [SPI_MAX_BYTES * 8 - 1:0] spi_rx_data;
wire spi_valid;
wire spi_busy;

spi_top #(
    .CLK_FREQ(CLK_FREQ),
    .SPI_FREQ(SPI_FREQ),
    .MAX_BYTES(SPI_MAX_BYTES),
    .DATA_SIDE(SPI_DATA_SIDE),
    .CPOL(CPOL),
    .CPHA(CPHA)
) spi_top_inst(
    .clk(clk),
    .rst_n(rst_n),
    .tx_packet(spi_tx_data),
    .rx_packet(spi_rx_data),
    .tx_size(uart_tx_data_size),
    .start(SPI_start),
    .rx_valid(spi_valid),
    .busy(spi_busy),
    .spi_cs_n(spi_cs),
    .spi_sclk(spi_clk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso)
);

//*************************PWM部分***************************//
reg PWM_start;
wire PWM_error;

packet2PWM #(
    .CLK_FREQ(CLK_FREQ),
    .MAX_RX_DATA_SIZE(MAX_RX_DATA_SIZE)
)
packet2PWM_inst(
    .clk(clk),
    .rst_n(rst_n),
    
    .uart_rx_packet(uart_rx_packet),
    .rx_data_size(rx_data_size),
    .pwm_start(PWM_start),

    .PWM(PWM),
    .error(PWM_error)
);

//************************CAN模块****************************//
localparam CAN_RX_ID_SHORT_FILTER = 11'h123;
localparam CAN_RX_ID_SHORT_MASK   = 11'h7ff;
localparam CAN_RX_ID_LONG_FILTER  = 29'h12345678;
localparam CAN_RX_ID_LONG_MASK    = 29'h1fffffff;
localparam CAN_DEFAULT_C_PTS      = 16'd34;
localparam CAN_DEFAULT_C_PBS1     = 16'd5;
localparam CAN_DEFAULT_C_PBS2     = 16'd10;

reg [10:0] can_local_id;
wire[28:0] can_rx_id;
reg        can_tx_start;
wire       can_tx_done;
reg [63:0] can_tx_data;
wire       can_rx_valid;
wire[ 3:0] can_rx_size;
wire[63:0] can_rx_data;
wire       can_rx_ide;
reg        can_rx_ready;


can_top #(
    .RX_ID_SHORT_FILTER(CAN_RX_ID_SHORT_FILTER),
    .RX_ID_SHORT_MASK  (CAN_RX_ID_SHORT_MASK  ),
    .RX_ID_LONG_FILTER (CAN_RX_ID_LONG_FILTER ),
    .RX_ID_LONG_MASK   (CAN_RX_ID_LONG_MASK   ),
    .default_c_PTS     (CAN_DEFAULT_C_PTS     ),      // 波特率 = 1M
    .default_c_PBS1    (CAN_DEFAULT_C_PBS1    ),
    .default_c_PBS2    (CAN_DEFAULT_C_PBS2    )
) can_top_inst (
    .rstn    (rst_n       ),
    .clk     (clk         ),
    .can_rx  (can_rx      ),
    .can_tx  (can_tx      ),
    .LOCAL_ID(can_local_id),
    .tx_valid(can_tx_start),
    .tx_ready(can_tx_done ),
    .tx_data (can_tx_data ),
    .rx_valid(can_rx_valid),
    .rx_data (can_rx_data ),
    .rx_len  (can_rx_size ),
    .rx_id   (can_rx_id   ),
    .rx_ide  (can_rx_ide  ),
    .rx_ready(can_rx_ready)
);



// // ********************定时模块**********************//
// // 以下为定时模块
// localparam auto_period = CLK_FREQ / 1 - 1;  // 周期为1000ms

// reg [31:0] counter;

// always @(posedge clk or negedge rst_n) begin
//     if(!rst_n) begin
//         counter <= 0;
//     end
//     else if(counter < auto_period)
//         counter <= counter + 1'b1;
//     else begin
// // 在此处添加定时指令
//         counter <= 0;
//         uart_send_en <= 1'b1;
//     end
// end

endmodule
