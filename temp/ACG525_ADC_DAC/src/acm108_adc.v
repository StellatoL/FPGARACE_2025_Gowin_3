module ACM108_ADC(

    input clk_50M,
    input clk_35M,
    input clk_200M,

    input reset,
    input [7:0] ADC_data,
    output uart_tx,
    output uart_led
);

wire lock;
wire clk_35M;
wire clk_200M;

assign ADC_clk = clk_35M;


// FIFO信号
wire [7:0] ADC_fifo;
wire fifo_empty;
wire fifo_full;
wire fifo_almost_empty;
wire fifo_almost_full;

// FIFO实例化 
fifo_top fifo_top(
    .Data(ADC_data),           
    .Reset(!reset),            
    .WrClk(clk_35M),           
    .RdClk(clk_200M),         
    .WrEn(~fifo_full),         
    .RdEn(wr_en & ~fifo_empty),
    .AlmostFullSetTh(8'd240), 
    .AlmostFullClrTh(8'd224), 
    .Almost_Empty(fifo_almost_empty), 
    .Almost_Full(fifo_almost_full),   
    .Q(ADC_fifo),             
    .Empty(fifo_empty),        
    .Full(fifo_full)           
);

// FIFO输出同步器（跨时钟域保护）
reg [7:0] ADC_fifo_sync0, ADC_fifo_sync;
always @(posedge clk_200M) begin
    if (!reset) begin
        ADC_fifo_sync0 <= 0;
        ADC_fifo_sync <= 0;
    end else begin
        ADC_fifo_sync0 <= ADC_fifo;
        ADC_fifo_sync <= ADC_fifo_sync0;
    end
end

// 双缓冲控制信号
reg buffer_sel = 0; // 0=缓冲A, 1=缓冲B
reg [2:0] state;
localparam IDLE = 0, FILL_A = 1, FILL_B = 2, WAIT_A = 3, WAIT_B = 4;

reg [14:0] wr_addr; // 15位地址 (32K深度)
reg wr_en = 1;
wire [7:0] ADC_RAM;

// 双缓冲RAM (64K深度分为两个32K缓冲区)
Gowin_SDPB Gowin_SDPB(
    .dout(ADC_RAM),
    .clka(clk_200M),
    .cea(wr_en),
    .clkb(clk_200M),
    .ceb(1),
    .oce(1),
    .reset(!reset),
    .ada({buffer_sel, wr_addr}), // 高位选择缓冲区
    .din(ADC_fifo_sync),         // 使用同步后的数据
    .adb({~uart_buffer_sel, uart_addr}) // 串口读取另一缓冲区
);

// 写地址控制
reg start_send;
wire send_busy; // 串口忙信号

always @(posedge clk_200M) begin
    if (!reset) begin
        wr_addr <= 0;
        buffer_sel <= 0;
        state <= IDLE;
        wr_en <= 1;
        start_send <= 0;
    end else begin
        start_send <= 0; // 默认不触发发送
        
        case(state)
            IDLE: begin
                wr_addr <= 0;
                state <= FILL_A;
                buffer_sel <= 0;
            end
            
            FILL_A: begin
                if (wr_en && ~fifo_empty) begin
                    if (wr_addr == 15'd32767) begin // 缓冲区A满
                        wr_addr <= 0;
                        buffer_sel <= 1; // 切换到B
                        state <= WAIT_A;
                        start_send <= 1; // 触发发送A
                    end else begin
                        wr_addr <= wr_addr + 1;
                    end
                end
            end
            
            FILL_B: begin
                if (wr_en && ~fifo_empty) begin
                    if (wr_addr == 15'd32767) begin // 缓冲区B满
                        wr_addr <= 0;
                        buffer_sel <= 0; // 切换回A
                        state <= WAIT_B;
                        start_send <= 1; // 触发发送B
                    end else begin
                        wr_addr <= wr_addr + 1;
                    end
                end
            end
            
            WAIT_A: begin
                // 等待串口完成发送且FIFO有足够空间
                if (!send_busy && ~fifo_almost_full) begin
                    state <= FILL_B;
                end
            end
            
            WAIT_B: begin
                if (!send_busy && ~fifo_almost_full) begin
                    state <= FILL_A;
                end
            end
        endcase
    end
end

// 错误计数器
reg [15:0] fifo_underflow_count;
always @(posedge clk_200M) begin
    if (!reset) begin
        fifo_underflow_count <= 0;
    end else if (fifo_empty && wr_en) begin
        fifo_underflow_count <= fifo_underflow_count + 1;
    end
end

// 串口接口信号
wire [14:0] uart_addr;
wire send_done;
wire uart_buffer_sel; // 串口当前读取的缓冲区

usart_send usart_send(
    .clk(clk_200M),
    .rst_n(reset),
    .tx(uart_tx),
    .led(uart_led),
    .ADC_data(ADC_RAM),
    .addr(uart_addr),
    .start_send(start_send),
    .send_done(send_done),
    .buffer_sel(uart_buffer_sel),
    .busy(send_busy) 
);

endmodule