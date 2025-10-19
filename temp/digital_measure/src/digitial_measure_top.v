// module digital_measure_top #(
//     parameter CLK_FREQ = 50_000_000,  // 系统时钟频率 (Hz)
//     parameter CLK_FS   = 200_000_000   // 基准时钟频率 (Hz)
// )(
//     input  sys_clk,          // 系统时钟
//     input  rst_n,            // 低有效复位
//     input  clk_fs,           // 基准时钟
//     input  clk_fx,           // 被测时钟
    
//     output reg [31:0] digital_freq,      // 最终频率输出
//     // output reg [31:0] digital_high_time, // 最终高电平时间
//     // output reg [31:0] digital_low_time,  // 最终低电平时间
//     output reg [9:0] digital_duty,      // 最终占空比
//     output reg digital_done              // 测量完成标志
// );

// // ====================== 参数和寄存器定义 ======================
// reg [11:0] GATE_TIME;        // 动态门控时间
// reg        measure_ctrl;     // 测量模式控制 (0:粗略, 1:精确)
// wire       measure_done;     // 测量完成信号

// // 测量模块输出
// wire [31:0] digital_freq_temp;  // 临时频率
// wire [31:0] high_cycles_temp;   // 临时高电平计数
// wire [31:0] low_cycles_temp;    // 临时低电平计数
// wire [15:0] digital_duty_temp;  // 临时占空比

// // 状态机定义
// localparam IDLE           = 0;  // 空闲状态
// localparam FIRST_MEASURE  = 1;  // 第一次测量（粗略）
// localparam WAIT_FIRST     = 2;  // 等待第一次测量完成
// localparam SET_PRECISE    = 3;  // 设置精确测量参数
// localparam SECOND_MEASURE = 4;  // 第二次测量（精确）
// localparam WAIT_SECOND    = 5;  // 等待第二次测量完成
// localparam MEASURE_DONE   = 6;  // 测量完成

// reg [2:0] digital_state;  // 状态寄存器
// reg [31:0] fast_freq;     // 快速估算的频率值

// // ====================== 状态机控制 ======================
// always @(posedge sys_clk or negedge rst_n) begin
//     if(!rst_n) begin
//         measure_ctrl <= 0;          // 初始为粗略测量模式
//         GATE_TIME <= 16'd10;        // 初始门控时间=10（快速估算）
//         digital_state <= IDLE;      // 初始状态
//         digital_freq <= 0;
//         digital_high_time <= 0;
//         digital_low_time <= 0;
//         digital_duty <= 0;
//         digital_done <= 0;
//     end
//     else begin
//         digital_done <= 0;  // 默认完成信号为低
        
//         case(digital_state)
//             IDLE: begin
//                 // 初始化后自动开始测量
//                 digital_state <= FIRST_MEASURE;
//             end
            
//             FIRST_MEASURE: begin
//                 measure_ctrl <= 0;        // 粗略测量模式
//                 GATE_TIME <= 16'd10;      // 小门控时间快速估算
//                 digital_state <= WAIT_FIRST;
//             end
            
//             WAIT_FIRST: begin
//                 // 等待第一次测量完成
//                 if(measure_done) begin
//                     fast_freq <= digital_freq_temp;  // 保存粗略频率
//                     digital_state <= SET_PRECISE;
//                 end
//             end
            
//             SET_PRECISE: begin
//                 // 根据粗略频率设置精确测量参数
//                 if(fast_freq < 32'd10_000) begin
//                     GATE_TIME <= 50;
//                 end
//                 else if(fast_freq < 32'd100_000) begin
//                     GATE_TIME <= 200;
//                 end
//                 else if(fast_freq < 32'd1_000_000) begin
//                     // 中频信号：门控时间 = 10,000 / (fast_freq/1e6)
//                     // 目标：约10ms测量窗口
//                     GATE_TIME <= 400;
//                 end
//                 else begin
//                     // 高频信号：固定门控时间
//                     GATE_TIME <= 1000;
//                 end
                
//                 digital_state <= SECOND_MEASURE;
//             end
            
//             SECOND_MEASURE: begin
//                 measure_ctrl <= 1;  // 精确测量模式
//                 digital_state <= WAIT_SECOND;
//             end
            
//             WAIT_SECOND: begin
//                 // 等待精确测量完成
//                 if(measure_done) begin
//                     // 锁存最终结果
//                     digital_freq <= digital_freq_temp;
//                     digital_high_time <= high_cycles_temp;
//                     digital_low_time <= low_cycles_temp;
//                     digital_duty <= digital_duty_temp;
//                     digital_state <= MEASURE_DONE;
//                 end
//             end
            
//             MEASURE_DONE: begin
//                 digital_done <= 1;  // 输出完成脉冲
//                 digital_state <= IDLE;  // 返回IDLE准备下一次测量
//             end
            
//             default: digital_state <= IDLE;
//         endcase
//     end
// end

// // ==================== 数字信号测量实例化 ======================
// digital_measure #(
//     .CLK_FS(CLK_FS)
// ) digital_measure_inst(
//     .clk_fs(clk_fs),            // 基准时钟
//     .rst_n(rst_n),              // 复位
//     .clk_fx(clk_fx),            // 被测时钟
//     .GATE_TIME(GATE_TIME),      // 动态门控时间
//     .frequency(digital_freq_temp),  // 频率输出
//     .high_cycles(high_cycles_temp), // 高电平计数
//     .low_cycles(low_cycles_temp),   // 低电平计数
//     .duty_permille(digital_duty_temp), // 占空比
//     .measure_done(measure_done)     // 测量完成标志
// );

// endmodule