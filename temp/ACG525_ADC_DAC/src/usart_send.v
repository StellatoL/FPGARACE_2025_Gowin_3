module usart_send(
    input clk,          // 200MHz时钟
    input rst_n,        // 低电平复位
    output reg tx,      // 串口发送数据线
    output reg led,     // LED指示灯
    input [7:0] ADC_data, // 从RAM读取的数据
    output reg [14:0] addr, // 读地址
    input start_send,   // 开始发送信号
    output reg send_done, // 发送完成标志
    output reg buffer_sel, // 当前读取的缓冲区
    output reg busy     // 发送忙信号
);

// 波特率计算：200MHz / 115200 ≈ 1736
parameter BAUD_DIV = 1736; 
parameter IDLE = 3'b000;
parameter START = 3'b001;
parameter DATA = 3'b010;
parameter STOP = 3'b011;
parameter NEXT_BYTE = 3'b100;

reg [2:0] state;       // 状态机状态
reg [11:0] baud_cnt;   // 波特率计数器 (扩展到12位)
reg [2:0] bit_cnt;     // 数据位计数器
reg [7:0] tx_data;     // 发送数据寄存器
reg [31:0] timeout_cnt; // 超时计数器
reg [14:0] byte_count; // 已发送字节计数

// 状态机主控
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        tx <= 1'b1;         // 空闲状态为高电平
        baud_cnt <= 0;
        bit_cnt <= 0;
        addr <= 0;
        led <= 1'b0;
        send_done <= 0;
        byte_count <= 0;
        buffer_sel <= 0;
        busy <= 0;
        tx_data <= 0;
        timeout_cnt <= 0;
    end else begin
        // 默认值
        send_done <= 0;
        busy <= (state != IDLE);
        
        // 超时计数器（防止死锁）
        if (state != IDLE && baud_cnt == 0) begin
            if (timeout_cnt < 200_000_000) begin // 约1秒超时
                timeout_cnt <= timeout_cnt + 1;
            end else begin
                state <= IDLE; // 超时后复位状态机
                timeout_cnt <= 0;
                led <= 1'b0;
            end
        end else if (state == IDLE) begin
            timeout_cnt <= 0;
        end
        
        case (state)
            IDLE: begin
                tx <= 1'b1; // 保持高电平
                led <= 1'b0; // LED熄灭
                byte_count <= 0;
                
                // 接收到开始发送信号
                if (start_send) begin
                    state <= START;
                    baud_cnt <= 0;
                    addr <= 0;
                    buffer_sel <= ~buffer_sel; // 切换到另一缓冲区
                    led <= 1'b1; // LED亮
                end
            end
            
            START: begin
                tx <= 1'b0; // 起始位
                
                if (baud_cnt == BAUD_DIV - 1) begin
                    state <= DATA;
                    baud_cnt <= 0;
                    bit_cnt <= 0;
                    tx_data <= ADC_data; // 读取第一个字节
                    tx <= tx_data[0]; // 发送最低位
                end else begin
                    baud_cnt <= baud_cnt + 1;
                end
            end
            
            DATA: begin
                if (baud_cnt == BAUD_DIV - 1) begin
                    baud_cnt <= 0;
                    if (bit_cnt == 7) begin
                        state <= STOP;
                        tx <= 1'b1; // 停止位
                    end else begin
                        bit_cnt <= bit_cnt + 1;
                        tx <= tx_data[bit_cnt + 1];
                    end
                end else begin
                    baud_cnt <= baud_cnt + 1;
                end
            end
            
            STOP: begin
                if (baud_cnt == BAUD_DIV - 1) begin
                    baud_cnt <= 0;
                    state <= NEXT_BYTE;
                end else begin
                    baud_cnt <= baud_cnt + 1;
                end
            end
            
            NEXT_BYTE: begin
                if (byte_count == 15'd32767) begin // 发送完整个缓冲区
                    state <= IDLE;
                    send_done <= 1;
                    led <= 1'b0;
                end else begin
                    state <= START;
                    baud_cnt <= 0;
                    addr <= addr + 1; // 递增地址
                    byte_count <= byte_count + 1;
                    tx_data <= ADC_data; // 预取下一个字节
                end
            end
            
            default: state <= IDLE;
        endcase
    end
end

endmodule