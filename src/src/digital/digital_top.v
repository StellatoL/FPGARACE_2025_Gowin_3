module digital_top(
    input clk,
    input rst_n,
    
    // cdc物理接口
    inout [7:0] fx2_fdata,
    input       fx2_flagb,
    input       fx2_flagc,
    input       fx2_ifclk,
    output [1:0] fx2_faddr,
    output fx2_sloe,
    output fx2_slwr,
    output fx2_slrd,
    output fx2_pkt_end,
    output fx2_slcs,

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
    input  wire can_rxd,
    output wire can_txd,

    // PWM相关端口
    output [3:0] PWM,

    // 数字电路测量端口
    input  wire extern_clk
);
//*****************参数定义区******************//
parameter CLK_FREQ = 50_000_000; // 时钟频率50MHz
parameter UART_BAUD = 115200;   // 串口波特率115200bps
parameter CDC_MAX_DATA_SIZE = 32; // CDC最大数据长度32字节
parameter I2C_FREQ = 100_000; // I2C时钟频率
parameter I2C_DATA_SIZE = 16; // I2C数据长度
parameter SPI_FREQ = 400_000; // SPI时钟频率
parameter SPI_MAX_BYTES = 16; // SPI最大传输字节数
parameter SPI_DATA_SIDE = "MSB"; // SPI数据位顺序
parameter CPOL = 0; // SPI时钟极性
parameter CPHA = 0; // SPI时钟相位
//**************FX2 CDC接口实例化******************//
wire cdc_tx_send_en;
wire [CDC_MAX_DATA_SIZE * 8 - 1:0] cdc_tx_data;
wire [7:0] cdc_send_len;
wire cdc_tx_done;

wire cdc_rx_valid;
wire [CDC_MAX_DATA_SIZE * 8 - 1:0] cdc_rx_data;
wire [7:0] cdc_rx_len;

FX2_CDC_TOP #(
    .MAX_DATA_SIZE(CDC_MAX_DATA_SIZE),
    .MSB_FIRST(1),
    .RX_TIMEOUT(10000)
)FX2_CDC_TOP_inst(
    .reset_n(rst_n),
    .fx2_fdata(fx2_fdata),
    .fx2_flagb(fx2_flagb),
    .fx2_flagc(fx2_flagc),
    .fx2_ifclk(fx2_ifclk),
    .fx2_faddr(fx2_faddr),
    .fx2_sloe(fx2_sloe),
    .fx2_slwr(fx2_slwr),
    .fx2_slrd(fx2_slrd),
    .fx2_pkt_end(fx2_pkt_end),
    .fx2_slcs(fx2_slcs),

    .tx_send_en(cdc_tx_send_en),
    .tx_data(cdc_tx_data),
    .tx_data_size(cdc_send_len),
    .tx_done(cdc_tx_done),

    .rx_valid(cdc_rx_valid),
    .rx_data_size(cdc_rx_len),
    .rx_data(cdc_rx_data)
);
assign cdc_tx_send_en = cdc_tx_protocol_en? 1'b1 : extern_done;
assign cdc_tx_data = cdc_tx_protocol_en? cdc_tx_protocol_data : extern_data;
assign cdc_send_len = cdc_tx_protocol_en? cdc_send_protocol_len : 4'd9;
//***************************************************************//
//************************数据类型转换部分*************************//
//**************************************************************//
wire [2:0] packet_type;
reg cdc_tx_protocol_en;
reg [CDC_MAX_DATA_SIZE * 8 - 1:0] cdc_tx_protocol_data;
reg [7:0] cdc_send_protocol_len;

assign packet_type = cdc_rx_data[cdc_rx_len*8-6-:3]; // 标识符位置

always @(posedge fx2_ifclk or negedge rst_n) begin
    if(!rst_n) begin
        uart_send_en <= 1'b0;
        cdc_tx_protocol_en <= 1'b0;
    end
    else begin
        case(packet_type)
            3'b000: begin // UART数据处理喵
                uart_send_en <= cdc_rx_valid;
                uart_tx_data <= {cdc_rx_data, 24'h000D0A};
                uart_tx_data_size <= cdc_rx_len + 2;
            end
            3'b001: begin // I2C数据处理喵
                I2C_start <= cdc_rx_valid;
                cdc_tx_protocol_en <= IIC_done;
                if(IIC_issend) begin
                    cdc_tx_protocol_data <= 24'h010D0A; // I2C发送成功
                    cdc_send_protocol_len <= 3;
                end
                else begin
                    cdc_tx_protocol_data <= {IIC_rdata, 24'h010D0A};
                    cdc_send_protocol_len <= IIC_rx_size + 3;
                end
            end
            3'b010: begin // SPI数据处理喵
                SPI_start <= cdc_rx_valid;
                spi_tx_data <= cdc_rx_data;
                cdc_tx_protocol_en <= spi_valid;
                // SPI发送数据位
                if(cdc_rx_data[cdc_rx_len * 8 - 5]) begin
                    cdc_tx_protocol_data <= 24'h010D0A; // SPI发送成功
                    cdc_send_protocol_len <= 3;
                end
                else begin
                    cdc_tx_protocol_data <= {spi_rx_data,24'h020D0A};
                    cdc_send_protocol_len <= cdc_rx_len + 2;
                end
            end
            3'b011: begin // PWM数据处理喵
                PWM_start <= cdc_rx_valid;
                cdc_tx_protocol_en <= cdc_rx_valid;
                cdc_tx_protocol_data <= {7'd0, ~PWM_error,24'h030D0A};
//                cdc_tx_protocol_data <= cdc_rx_data;
                cdc_send_protocol_len <= 4;
            end
            3'b100: begin // CAN数据处理喵
                if(cdc_rx_data[cdc_rx_len * 8 - 4]) begin
                    if(cdc_rx_data[cdc_rx_len * 8 - 5]) begin
                        // 扩展帧相关代码（未完成）
                    end
                    else begin
                        // 发送数据
                        can_local_id <= cdc_rx_data[74:64];
                        can_tx_data <= cdc_rx_data[63:0];
                        can_tx_start <= cdc_rx_valid;
                        cdc_tx_protocol_en <= cdc_rx_valid;
                        cdc_tx_protocol_data <= 24'h010D0A; // CAN发送成功
                        cdc_send_protocol_len <= 3;
                    end
                end
                else begin
                    // 接收数据
                    can_rx_ready <= 1'b1;
                    cdc_tx_protocol_en <= cdc_rx_valid;
                    cdc_tx_protocol_data <= {can_rx_data, 24'h040D0A};
                    cdc_tx_protocol_data[can_rx_size*8 +24 +:40] <= can_rx_id;
                    cdc_send_protocol_len <= can_rx_size + 8;
                end
            end
            default: begin
//                 如果标识符与实际不符，则通过cdc将数据传回上位机
                cdc_tx_protocol_en <= cdc_rx_valid;
                cdc_tx_protocol_data <= {cdc_rx_data, 24'hFF0D0A};
                cdc_send_protocol_len <= cdc_rx_len + 2;
            end
        endcase
    end
end
//**************普通串口实例化****************//
reg uart_send_en;
reg [CDC_MAX_DATA_SIZE * 8 - 1:0] uart_tx_data;
reg [7:0] uart_tx_data_size;

localparam MAX_UART_TX_DATA_WIDTH = CDC_MAX_DATA_SIZE * 8 + 24;
uart_data_tx#(
    .CLK_FREQ(CLK_FREQ),
    .UART_BAUD(UART_BAUD),
	.MAX_DATA_WIDTH(MAX_UART_TX_DATA_WIDTH),
	.MSB_FIRST(1'b1)
)
uart_data_tx_inst(
	.clk(fx2_ifclk),
	.rst_n(rst_n),
	.data(uart_tx_data),
	.send_en(uart_send_en),
    .tx_data_size(uart_tx_data_size),
	.uart_tx(uart_txd),
	.Tx_Done(),
	.uart_state()
);
//**************I2C实例化****************//
reg I2C_start;
wire IIC_done;
wire [I2C_DATA_SIZE * 8 - 1:0] IIC_rdata;
wire [7:0] IIC_rx_size;
wire IIC_issend;
wire ack_flag;
packet2IIC #(
    .CLK_FREQ(CLK_FREQ),
    .I2C_FREQ(I2C_FREQ),
    .MAX_RX_DATA_SIZE(CDC_MAX_DATA_SIZE),
    .DATA_BYTE_NUM(I2C_DATA_SIZE)
) packet2IIC_inst (
    .clk(fx2_ifclk),
    .rst_n(rst_n),
    .uart_rx_packet(cdc_rx_data),
    .rx_data_size(cdc_rx_len),
    .trans_start(I2C_start),
    .rdata(IIC_rdata),
    .IIC_rx_size(IIC_rx_size),
    .IIC_done(IIC_done),
    .issend(IIC_issend),
    .ack_flag(ack_flag),
    .scl(scl),
    .sda(sda)
);
//**************SPI实例化****************//
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
    .clk(fx2_ifclk),
    .rst_n(rst_n),
    .tx_packet(spi_tx_data),
    .rx_packet(spi_rx_data),
    .tx_size(cdc_rx_len-1),
    .start(SPI_start),
    .rx_valid(spi_valid),
    .busy(spi_busy),
    .spi_cs_n(spi_cs),
    .spi_sclk(spi_clk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso)
);
//**************多路PWM输出实例化****************//
reg PWM_start;
wire PWM_error;

packet2PWM #(
    .CLK_FREQ(CLK_FREQ),
    .MAX_RX_DATA_SIZE(CDC_MAX_DATA_SIZE)
) packet2PWM_inst (
    .clk(fx2_ifclk),
    .rst_n(rst_n),
    .uart_rx_packet(cdc_rx_data),
    .rx_data_size(cdc_rx_len),
    .pwm_start(PWM_start),
    .PWM(PWM),
    .error(PWM_error)
);
//**************CAN通信模块实例化****************//
localparam CAN_RX_ID_SHORT_FILTER = 11'h123;
localparam CAN_RX_ID_SHORT_MASK   = 11'h7ff;
localparam CAN_RX_ID_LONG_FILTER  = 29'h12345678;
localparam CAN_RX_ID_LONG_MASK    = 29'h1fffffff;
localparam CAN_DEFAULT_C_PTS      = 16'd32;
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
    .clk     (fx2_ifclk         ),
    .can_rx  (can_rxd     ),
    .can_tx  (can_txd     ),
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
// ======================测量频率、占空比模块=========================
wire [31:0] extern_freq;
wire [ 9:0] extern_duty;
wire [87:0] extern_data;
wire extern_done;

assign extern_data = {extern_freq,6'b0,extern_duty,24'h050D0A};

digital_measure #(
    .CLK_FS(50_000_000)
) digital_measure_top_inst(
    .clk_fs(clk),
    .rst_n(rst_n),
    .clk_fx(extern_clk),
    .digital_frequency(extern_freq),

    .digital_duty(extern_duty),
    .digital_done(extern_done)
);


// // ********************定时模块**********************//
// // 以下为定时模块
//  localparam auto_period = 50_000_000 / 1 - 1;  // 周期为1000ms

//  reg [31:0] counter;

//  always @(posedge clk or negedge rst_n) begin
//      if(!rst_n) begin
//          counter <= 0;
//      end
//      else if(counter < auto_period) begin
//          counter <= counter + 1'b1;
//         uart_send_en <= 1'b0;
//     end
//      else begin
//  // 在此处添加定时指令
//          counter <= 0;
//          uart_send_en <= 1'b1;
//      end
//  end

endmodule
















