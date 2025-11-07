module usart_send(
    input clk,          // 时钟引脚 (T9)
    input rst_n,        // 复位引脚 (B17)
    output tx,          // 串口发送引脚 (V8)
    input   [7:0]ADC_data,
    output  reg tx_all_done ,
    input uart_en
);


parameter BYTE_NUM = 1;  // 可配置的字节数
wire [7:0] data_array [0:BYTE_NUM-1];



      assign data_array[0] =ADC_data;
//    data_array[0] = 48+hundreds;  // ASCII码 '0' + 数值
//    data_array[1] = 48+tens;  // ASCII码 '0' + 数值
//    data_array[2] = 48+ones;  // ASCII码 '0' + 数值
//    data_array[3] = 8'h0A;


wire [3:0] hundreds;
wire [3:0] tens;
wire [3:0] ones;

bin2bcd bin2bcd(
    .clk(clk),
    .bin(ADC_data),
    .hundreds(hundreds),
    .tens(tens),
    .ones(ones)
);

// 控制信号
reg tx_en = 0;
wire tx_done;
reg [7:0] tx_data;
reg [31:0] byte_count = 0;
reg led_reg = 0;

// 实例化串口发送模块
uart_tx uart_tx_inst(
    .clk(clk),
    .rst_n(rst_n),
    .tx_en(tx_en),
    .tx_data(tx_data),
    .tx(tx),
    .tx_done(tx_done)
);


reg uart_en_reg;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        uart_en_reg <= 0;
    end 
    else if(uart_en) begin
        uart_en_reg <= 1;
    end
    else if(tx_all_done) begin
        uart_en_reg <=0;
    end
    else begin
        uart_en_reg <= uart_en_reg;
    end
end


always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_en <= 0;
        byte_count <= 0;
    end 
    else begin
        // 发送状态机
        if (!tx_en&&uart_en) begin
            tx_en <= 1'b1;
            tx_data <= data_array[byte_count];
        end 
        else if (tx_done) begin
            if (byte_count == BYTE_NUM - 1) begin
                byte_count <= 0;  // 循环发送
                tx_all_done<=1;
            end 
            else begin
                byte_count <= byte_count + 1;
                tx_all_done<=0;
            end
            tx_data <= data_array[byte_count];
        end
    end
end
endmodule