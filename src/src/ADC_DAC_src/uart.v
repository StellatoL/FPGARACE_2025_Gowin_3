module uart_tx(
    input clk,          // 时钟信号 (引脚T9)
    input rst_n,        // 低电平复位 (引脚B17)
    input tx_en,        // 发送使能信号
    input [7:0] tx_data,// 待发送数据
    output reg tx,      // 串口发送引脚 (引脚V8)
    output reg tx_done  // 字节发送完成标志
);

// 波特率参数设置 (115200 bps @ 35MHz)
parameter CLK_FREQ = 200_000_000;
parameter BAUD_RATE = 1000000;
localparam BAUD_DIV = CLK_FREQ / BAUD_RATE;

// 状态机定义
localparam IDLE  = 2'b00;
localparam START = 2'b01;
localparam DATA  = 2'b10;
localparam STOP  = 2'b11;

reg [1:0] state = IDLE;
reg [15:0] baud_counter = 0;
reg [2:0] bit_index = 0;
reg [7:0] data_reg;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        tx <= 1'b1;
        tx_done <= 1'b0;
        baud_counter <= 0;
        bit_index <= 0;
    end else begin
        tx_done <= 1'b0;  // 默认清零完成标志
        
        case (state)
            IDLE: begin
                tx <= 1'b1;  // 空闲状态高电平
                if (tx_en) begin
                    state <= START;
                    data_reg <= tx_data;
                    baud_counter <= 0;
                end
            end
            
            START: begin
                tx <= 1'b0;  // 起始位
                if (baud_counter == BAUD_DIV - 1) begin
                    state <= DATA;
                    baud_counter <= 0;
                    bit_index <= 0;
                end else begin
                    baud_counter <= baud_counter + 1;
                end
            end
            
            DATA: begin
                tx <= data_reg[bit_index];  // 发送数据位
                if (baud_counter == BAUD_DIV - 1) begin
                    baud_counter <= 0;
                    if (bit_index == 7) begin
                        state <= STOP;
                    end else begin
                        bit_index <= bit_index + 1;
                    end
                end else begin
                    baud_counter <= baud_counter + 1;
                end
            end
            
            STOP: begin
                tx <= 1'b1;  // 停止位
                if (baud_counter == BAUD_DIV - 1) begin
                    state <= IDLE;
                    tx_done <= 1'b1;  // 发送完成标志
                end else begin
                    baud_counter <= baud_counter + 1;
                end
            end
        endcase
    end
end
endmodule