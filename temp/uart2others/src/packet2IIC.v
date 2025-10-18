module packet2IIC #(
    parameter CLK_FREQ      = 50_000_000,
    parameter I2C_FREQ      = 100_000,
    parameter MAX_RX_DATA_SIZE = 25,
    parameter DATA_BYTE_NUM = 16    // 最大数据字节数
)(
    input clk,
    input rst_n,
    input [MAX_RX_DATA_SIZE * 8 - 1:0] uart_rx_packet,
    input [7:0] rx_data_size,
    input trans_start, // 启动信号

    // IIC接收数据
    output wire [DATA_BYTE_NUM*8-1:0] rdata,
    output reg [7:0] IIC_rx_size,

    output IIC_done,
    output ack_flag,
    output wire issend,

    // IIC物理接口
    output scl,
    inout  sda

    
);

// localparam define
localparam REG_ADDR_BYTE_NUM = 2; // 最大寄存器地址字节数

//*****************************数据帧解析*****************************//
reg         I2C_start;
reg [6:0]   DEVICE_ADDR;
reg         rw_flag;
reg         reg_size_crtl;
reg [REG_ADDR_BYTE_NUM*8-1:0] reg_addr;
reg [DATA_BYTE_NUM*8-1:0] wdata;
reg [7:0]   user_byte_num;

assign issend = ~rw_flag;

// 计算数据包中各个字段的位置
wire [7:0] last_byte_pos = (rx_data_size - 1) * 8;        // 最后一个字节位置
wire [7:0] second_last_byte_pos = (rx_data_size - 2) * 8; // 倒数第二个字节位置
wire [7:0] third_last_byte_pos = (rx_data_size - 3) * 8;  // 倒数第三个字节位置
wire [7:0] fourth_last_byte_pos = (rx_data_size - 4) * 8; // 倒数第四个字节位置

// 寄存器地址大小指示位位置
localparam REG_SIZE_BIT_POS = 4; // 最低字节的倒数第3位

// 解析状态机
localparam IDLE = 0;
localparam PARSE = 1;
reg [1:0] state;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state <= IDLE;
        I2C_start <= 0;
        wdata <= 0;
        user_byte_num <= 0;
        DEVICE_ADDR <= 0;
        rw_flag <= 0;
        reg_size_crtl <= 0;
        reg_addr <= 0;
    end
    else begin
        I2C_start <= 0; // 默认不启动
        
        case(state)
            IDLE: begin
                if(trans_start) begin
                    state <= PARSE;
                end
            end
            
            PARSE: begin
                // 解析寄存器地址大小指示位 (最低字节的bit3)
                reg_size_crtl <= uart_rx_packet[last_byte_pos + REG_SIZE_BIT_POS];
                // 解析从机地址和读写标志位 (倒数第二个字节)
                // bit[0]: 读写标志位, bit[7:1]: 从机地址
                rw_flag <= uart_rx_packet[second_last_byte_pos];
                DEVICE_ADDR <= uart_rx_packet[(second_last_byte_pos + 1) +: 7];
                
                // 解析寄存器地址
                if(reg_size_crtl) begin
                    // 16位寄存器地址: 使用倒数第三和第四字节
                    reg_addr <= {
                        uart_rx_packet[third_last_byte_pos +: 8], // 高8位
                        uart_rx_packet[fourth_last_byte_pos +: 8]   // 低8位
                    };
                end
                else begin
                    // 8位寄存器地址: 使用倒数第三字节
                    reg_addr <= {8'h00, uart_rx_packet[third_last_byte_pos +: 8]};
                end
                if (!uart_rx_packet[second_last_byte_pos]) begin
                    // 计算数据字节数
                    user_byte_num <= rx_data_size - 3 - reg_size_crtl;
                    
                    // 提取写数据
                    if(user_byte_num > 0) begin
                        // 数据位于数据包的尾部
                        wdata <= uart_rx_packet << 
                                ((DATA_BYTE_NUM - rx_data_size + 3 + reg_size_crtl) * 8);
                    end
                    else begin
                        wdata <= 0;
                    end
                end
                else begin
                    user_byte_num <= uart_rx_packet[(fourth_last_byte_pos - reg_size_crtl * 8) +: 8];
                    IIC_rx_size <= uart_rx_packet[(fourth_last_byte_pos - reg_size_crtl * 8) +: 8];
                end

                // 启动I2C传输
                I2C_start <= 1'b1;
                state <= IDLE;
            end
        endcase
    end
end

//****************************I2C驱动实例化***************************//

I2C_drive #(
    .CLK_FREQ(CLK_FREQ),
    .I2C_FREQ(I2C_FREQ),  // 使用参数化频率

    .REG_ADDR_BYTE_NUM(REG_ADDR_BYTE_NUM),
    .DATA_BYTE_NUM(DATA_BYTE_NUM)
) I2C_drive_inst (
    .clk(clk),
    .rst_n(rst_n),
    .start(I2C_start),
    .DEVICE_ADDR(DEVICE_ADDR),
    .rw_flag(rw_flag),
    .reg_size_crtl(reg_size_crtl),
    .reg_addr(reg_addr),
    .wdata(wdata),
    .user_byte_num(user_byte_num),
    .rdata(rdata),
    .rdata_vld(),
    .byte_valid(),
    .IIC_done(IIC_done),
    .scl(scl),
    .sda(sda),
    .ack_flag(ack_flag)
);

endmodule
