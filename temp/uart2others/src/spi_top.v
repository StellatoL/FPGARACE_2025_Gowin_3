`timescale 1ns/1ns
module spi_top #(
    parameter CLK_FREQ = 50_000_000,      // 系统时钟频率 (Hz)
    parameter SPI_FREQ = 400_000,         // SPI时钟频率 (Hz)
    parameter MAX_BYTES = 16,             // 最大字节数
    parameter DATA_SIDE = "MSB",          // MSB:大端模式 LSB:小端模式
    parameter CPOL = 1'b0,                // SPI时钟极性
    parameter CPHA = 1'b0                 // SPI时钟相位
)(
    input clk,                            // 系统时钟
    input rst_n,                          // 系统复位
    
    input [MAX_BYTES*8-1:0] tx_packet,    // 发送数据包
    input [7:0] tx_size,                  // 发送字节数 (1-16)
    input start,                          // 传输启动信号
    output reg [MAX_BYTES*8-1:0] rx_packet, // 接收数据包
    output reg rx_valid,                  // 接收数据有效标志
    output reg spi_done,                  // SPI完成信号
    output busy,                          // SPI忙信号
    
    // SPI物理接口
    output spi_cs_n,
    output spi_sclk,
    output spi_mosi,
    input  spi_miso
);

// ======================== 内部信号定义 ========================
localparam BIT_NUM = 8;                   // 单字节位宽
localparam IDLE    = 3'd0;                // 空闲状态
localparam START   = 3'd1;                // 启动传输状态
localparam SEND    = 3'd2;                // 发送字节状态
localparam WAIT    = 3'd3;                // 等待字节完成状态
localparam NEXT    = 3'd4;                // 准备下一字节状态
localparam DONE    = 3'd5;                // 传输完成状态

reg [2:0] state;                          // 状态机寄存器
reg [7:0] byte_cnt;                       // 字节计数器
reg swop_en;                              // SPI模块交换使能
reg [BIT_NUM-1:0] send_data;              // 发送数据寄存器
wire [BIT_NUM-1:0] recv_data;             // SPI模块接收数据
wire recv_vld;                            // SPI模块接收有效标志
wire comm_done;                           // SPI模块通信完成标志
wire byte_done;                           // 单字节传输完成标志
reg [MAX_BYTES*8-1:0] rx_packet_reg;      // 接收数据包寄存器
reg start_reg;                            // 启动信号寄存器
wire start_rise;                          // 启动信号上升沿标志

// ======================== 主控制逻辑 ========================

// 启动信号上升沿检测
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        start_reg <= 1'b0;
    end else begin
        start_reg <= start;
    end
end
assign start_rise = start && !start_reg;

// 忙信号生成
assign busy = (state != IDLE);

// 状态机控制多字节传输
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        byte_cnt <= 0;
        swop_en <= 0;
        send_data <= 0;
        rx_packet_reg <= 0;
        rx_valid <= 0;
        spi_done <= 0;
    end 
    else begin
        // 默认值
        rx_valid <= 0;
        spi_done <= 0;
        swop_en <= 1'b0;
        
        case (state)
            // 空闲状态 - 等待启动信号
            IDLE: begin
                spi_done <= 1'b0;
                if (start_rise && tx_size > 0) begin
                    state <= START;
                    byte_cnt <= 0;
                    rx_packet_reg <= 0;
                end
            end
            
            // 启动传输状态
            START: begin
                // 提取当前字节数据
                send_data <= tx_packet[(MAX_BYTES - byte_cnt - 1)*8 +: 8];
                state <= SEND;
            end
            
            // 发送字节状态
            SEND: begin
                swop_en <= 1'b1; // 启动字节传输
                state <= WAIT;
            end
            
            // 等待字节传输完成
            WAIT: begin
                if (byte_done) begin
                    // 存储接收到的字节
                    rx_packet_reg[(MAX_BYTES - byte_cnt - 1)*8 +: 8] <= recv_data;
                    
                    // 检查是否完成所有字节传输
                    if (byte_cnt == tx_size - 1) begin
                        state <= DONE;
                    end 
                    else begin
                        state <= NEXT;
                    end
                end
            end
            
            // 准备下一字节状态
            NEXT: begin
                byte_cnt <= byte_cnt + 1;
                state <= START;
            end
            
            // 传输完成状态
            DONE: begin
                rx_packet <= rx_packet_reg; // 输出接收数据包
                rx_valid <= 1'b1;           // 接收数据有效标志
                spi_done <= 1'b1;           // SPI传输完成标志
                state <= IDLE;              // 返回空闲状态
            end
            
            default: state <= IDLE;
        endcase
    end
end

// ======================== SPI主机实例化 ========================
spi_modu #(
    .CLK_FREQ   (CLK_FREQ  ),  // 转换为MHz
    .SPI_FREQ   (SPI_FREQ  ),      // 转换为kHz
    .DATA_SIDE  (DATA_SIDE ),             // MSB:大端模式 LSB:小端模式
    .BIT_NUM    (BIT_NUM   ),             // 数据长度 |单位bit
    .CPHA       (CPHA      ),             // 时钟相位
    .CPOL       (CPOL      )              // 时钟极性
) spi_modu_inst (
    .clk        (clk       ),             // 模块时钟输入
    .rst_n      (rst_n     ),             // 模块复位输入
    .swop_en    (swop_en   ),             // 开始交换使能
    .send_data  (send_data ),             // 发送数据输入
    .recv_vld   (recv_vld  ),             // 接收数据有效标志
    .recv_data  (recv_data ),             // 接收数据输出
    .swop_done  (byte_done ),             // 交换结束标志 (关键修复)
    .comm_done  (comm_done ),             // 通信结束标志
    .spi_cs_n   (spi_cs_n  ),             // 设备片选
    .spi_sclk   (spi_sclk  ),             // 通信时钟
    .spi_mosi   (spi_mosi  ),             // 数据输出
    .spi_miso   (spi_miso  )              // 数据输入
);

endmodule