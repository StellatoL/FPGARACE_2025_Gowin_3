module uart_packet_rx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter UART_BAUD = 115200,
    parameter MAX_DATA_SIZE = 25
)(
    input clk,
    input rst_n,
    input uart_rxd,
    output reg uart_rx_done,
    output reg [MAX_DATA_SIZE * 8 - 1:0] uart_rx_packet,
    output reg [7:0] rx_data_size
);
// 状态定义（扩展为3位）
localparam UART_IDLE = 3'd0;
localparam UART_SOF  = 3'd1;
localparam UART_DATA = 3'd2;
localparam UART_CR   = 3'd3;
localparam UART_EOF  = 3'd4;
// 状态寄存器
reg [2:0] state;
//*********************** 串口接收实例化 ***********************//
wire uart_byte_rx_done;
wire [7:0] uart_byte_rx_data;
uart_byte_rx #(
    .CLK_FREQ(CLK_FREQ),
    .UART_BAUD(UART_BAUD)
) uart_byte_rx_inst (
    .CLK(clk),
    .RESET(rst_n),
    .UART_RXD(uart_rxd),
    .UART_RX_done(uart_byte_rx_done),
    .UART_RX_data(uart_byte_rx_data)
);
//*************************main code***************************//
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state <= UART_IDLE;
        uart_rx_packet <= 0;
        uart_rx_done <= 1'b0;
        rx_data_size <= 0;
    end
    else begin
        uart_rx_done <= 1'b0;  // 默认完成信号为低
        
        case(state)
            UART_IDLE: begin
                if(uart_byte_rx_done && uart_byte_rx_data == 8'h20) begin
                    state <= UART_SOF;
                    rx_data_size <= 0;
                    uart_rx_packet <= 0;
                end
            end
            
            UART_SOF: begin
                if(uart_byte_rx_done) begin
                    if(uart_byte_rx_data == 8'h25) begin
                        state <= UART_DATA;
                    end
                    else begin
                        state <= UART_IDLE; // 无效起始字节
                    end
                end
            end
            
            UART_DATA: begin
                if(uart_byte_rx_done) begin
                    if(rx_data_size < MAX_DATA_SIZE) begin
                        if(uart_byte_rx_data == 8'h0D) begin
                            state <= UART_CR; // 检测到回车符，进入CR状态
                        end
                        else begin
                            // 移位存储数据
                            uart_rx_packet <= (uart_rx_packet << 8) | uart_byte_rx_data;
                            rx_data_size <= rx_data_size + 1;
                        end
                    end
                    else begin // 达到最大长度
                        state <= UART_EOF;
                    end
                end
            end
            
            // 处理回车符状态
            UART_CR: begin
                if(uart_byte_rx_done) begin
                    if(uart_byte_rx_data == 8'h0A) begin
                        state <= UART_EOF; // 正确帧尾序列 \r\n
                    end
                    else begin
                        // 存储之前跳过的0x0D
                        if(rx_data_size < MAX_DATA_SIZE) begin
                            uart_rx_packet <= (uart_rx_packet << 8) | 8'h0D;
                            rx_data_size <= rx_data_size + 1;
                        end
                        // 存储当前字节（如果非0x0A且空间足够）
                        if(rx_data_size < MAX_DATA_SIZE) begin
                            uart_rx_packet <= (uart_rx_packet << 8) | uart_byte_rx_data;
                            rx_data_size <= rx_data_size + 1;
                        end
                        state <= UART_DATA; // 返回数据接收状态
                    end
                end
            end
            
            UART_EOF: begin
                uart_rx_done <= 1'b1;  // 标记数据包接收完成
                state <= UART_IDLE;     // 返回空闲状态
            end
            
            default: state <= UART_IDLE;
        endcase
    end
end
endmodule