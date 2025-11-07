module packet2PWM #(
    parameter CLK_FREQ = 50_000_000,
    parameter MAX_RX_DATA_SIZE = 21
)(
    input clk,
    input rst_n,
    input [MAX_RX_DATA_SIZE * 8 - 1:0] uart_rx_packet,
    input [7:0] rx_data_size,
    input pwm_start,

    output wire [3:0] PWM,
    output reg error
);


localparam PWM_DATA_SIZE = 21;                         // 期望数据包大小(21字节)
localparam CTRL_BYTE_POS = 21 * 8 - 1;                 // 控制字节位置(第20字节的高5位-高8位)
localparam PSC_START_POS = 12 * 8;                     // PSC起始位置(第12字节)
localparam ARR_START_POS =  4 * 8;                     // ARR起始位置(第4字节)
localparam COMPARE_START_POS =  0;                     // 占空比起始位置(第0字节)

localparam IDLE      = 0;
localparam CHECK     = 1;
localparam EXTRACT   = 2;
localparam ERROR     = 3;

reg [1:0] state;              // 状态寄存器
reg [3:0] PWM_switch;         // PWM通道使能控制
reg [15:0] PSC [0:3];         // 4通道预分频值(16位)
reg [15:0] ARR [0:3];         // 4通道自动重装值(16位)
reg [7:0]  compare [0:3];     // 4通道占空比值(8位)

//*****************************数据帧解析*****************************//

// 状态机控制逻辑
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        PWM_switch <= 4'h0;
        error <= 1'b0;
        // 初始化PWM参数
        PSC[0] <= 16'd0;
        ARR[0] <= 16'd0;
        compare[0] <= 8'd0;
        PSC[1] <= 16'd0;
        ARR[1] <= 16'd0;
        compare[1] <= 8'd0;
        PSC[2] <= 16'd0;
        ARR[2] <= 16'd0;
        compare[2] <= 8'd0;
        PSC[3] <= 16'd0;
        ARR[3] <= 16'd0;
        compare[3] <= 8'd0;
    end
    else begin
        case (state)
            IDLE: begin
                error <= 1'b0;  // 清除错误标志
                if (pwm_start) begin
                    state <= CHECK;  // 收到启动信号
                end
            end
            
            CHECK: begin
                // 验证数据包大小
                if (rx_data_size != PWM_DATA_SIZE) begin
                    state <= ERROR;
                end
                // 验证参数范围(可选)
                else if (!check(uart_rx_packet)) begin
                    state <= ERROR;
                end
                else begin
                    state <= EXTRACT;
                end
            end
            
            EXTRACT: begin
                // 提取控制字节的低4位(每个bit控制一个通道)
                PWM_switch <= uart_rx_packet[CTRL_BYTE_POS -: 4];
                
                // 提取PSC参数(4个通道)
                PSC[0] <= uart_rx_packet[PSC_START_POS + 0*16 +: 16];
                PSC[1] <= uart_rx_packet[PSC_START_POS + 1*16 +: 16];
                PSC[2] <= uart_rx_packet[PSC_START_POS + 2*16 +: 16];
                PSC[3] <= uart_rx_packet[PSC_START_POS + 3*16 +: 16];

                // 提取ARR参数(4个通道)
                ARR[0] <= uart_rx_packet[ARR_START_POS + 0*16 +: 16];
                ARR[1] <= uart_rx_packet[ARR_START_POS + 1*16 +: 16];
                ARR[2] <= uart_rx_packet[ARR_START_POS + 2*16 +: 16];
                ARR[3] <= uart_rx_packet[ARR_START_POS + 3*16 +: 16];
                
                // 提取占空比参数(4个通道)
                compare[0] <= uart_rx_packet[COMPARE_START_POS + 0*8 +: 8];
                compare[1] <= uart_rx_packet[COMPARE_START_POS + 1*8 +: 8];
                compare[2] <= uart_rx_packet[COMPARE_START_POS + 2*8 +: 8];
                compare[3] <= uart_rx_packet[COMPARE_START_POS + 3*8 +: 8];
                
                state <= IDLE;  // 返回空闲状态
            end
            
            ERROR: begin
                error <= 1'b1;  // 设置错误标志
                state <= IDLE;   // 返回空闲状态
            end
            
            default: state <= IDLE;
        endcase
    end
end

// 参数验证函数(可选)
function automatic check;
    input [MAX_RX_DATA_SIZE*8-1:0] packet;
    begin
        check = 1'b1;
        
        // 检查PSC值不为零(防止除零错误)
        for (integer i = 0; i < 4; i = i + 1) begin
            if (packet[PSC_START_POS + i*16 +: 16] == 16'd0) begin
                packet[PSC_START_POS + i*16 +: 16] = 16'd1;
            end
        end
        
        // 检查占空比不超过ARR值
        for (integer i = 0; i < 4; i = i + 1) begin
            // 注意: 占空比是8位，ARR是16位，直接比较安全
            if (packet[COMPARE_START_POS + i*8 +: 8] > 8'd255) begin
                packet[COMPARE_START_POS + i*8 +: 8] = 8'd255;
            end
        end
    end
endfunction

//************************************PWM驱动实例化*********************************//

pwm_drive #(
    .CLK_FREQ(CLK_FREQ)
) pwm_drive_inst_ch0 (
    .clk(clk),
    .rst_n(rst_n),
    .PSC(PSC[0]),         // 通道i的预分频值
    .ARR(ARR[0]),         // 通道i的自动重装值
    .compare(compare[0]), // 通道i的占空比
    .pwm_start(PWM_switch[3]), // 通道i的使能信号
    .PWM(PWM[0])          // 通道i的PWM输出
);

pwm_drive #(
    .CLK_FREQ(CLK_FREQ)
) pwm_drive_inst_ch1 (
    .clk(clk),
    .rst_n(rst_n),
    .PSC(PSC[1]),         // 通道i的预分频值
    .ARR(ARR[1]),         // 通道i的自动重装值
    .compare(compare[1]), // 通道i的占空比
    .pwm_start(PWM_switch[2]), // 通道i的使能信号
    .PWM(PWM[1])          // 通道i的PWM输出
);

pwm_drive #(
    .CLK_FREQ(CLK_FREQ)
) pwm_drive_inst_ch2 (
    .clk(clk),
    .rst_n(rst_n),
    .PSC(PSC[2]),         // 通道i的预分频值
    .ARR(ARR[2]),         // 通道i的自动重装值
    .compare(compare[2]), // 通道i的占空比
    .pwm_start(PWM_switch[1]), // 通道i的使能信号
    .PWM(PWM[2])          // 通道i的PWM输出
);

pwm_drive #(
    .CLK_FREQ(CLK_FREQ)
) pwm_drive_inst_ch3 (
    .clk(clk),
    .rst_n(rst_n),
    .PSC(PSC[3]),         // 通道i的预分频值
    .ARR(ARR[3]),         // 通道i的自动重装值
    .compare(compare[3]), // 通道i的占空比
    .pwm_start(PWM_switch[0]),
    .PWM(PWM[3])          // 通道i的PWM输出
);


endmodule
