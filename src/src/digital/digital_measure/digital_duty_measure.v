module digital_duty_measure#(
    parameter    CLK_FS = 26'd50_000_000 // 基准时钟频率值 (Hz)
) (
    input  wire        clk_fs,    // 基准时钟（参考时钟）
    input  wire        rst_n,     // 低有效复位
    input  wire        clk_fx,    // 被测时钟信号
    input       [9:0] GATE_TIME, // 门控时间（被测时钟周期数）
    output wire [9:0] duty_permille, // 占空比千分比（0-1000）
    output reg         measure_done // 测量完成标志（单周期脉冲）
);

// ====================== 内部信号声明 ======================
// 门控信号相关寄存器
reg gate;                // 被测时钟域生成的门控信号
reg  [ 9:0] gate_cnt;    // 门控时间计数器（clk_fx域）

reg  [31:0] high_cycles; // 门控期间高电平计数（基准时钟周期数）
reg  [31:0] low_cycles;  // 门控期间低电平计数（基准时钟周期数）

// 基准时钟域同步寄存器
reg gate_fs_r;           // 门控信号同步第一级
reg gate_fs;             // 门控信号同步第二级

// 边沿检测寄存器（只保留基准域用于产生完成脉冲）
reg gate_fs_d0, gate_fs_d1; // 基准时钟域门控信号打拍

// 占空比测量相关
reg clk_fx_sync0, clk_fx_sync1; // 被测时钟同步到基准时钟域的寄存器
reg [31:0] high_temp, low_temp; // 高/低电平临时计数器

// ====================== 边沿检测信号 ======================
wire neg_gate_fs = gate_fs_d1 & (~gate_fs_d0); // 基准域门控下降沿

// ====================== 门控信号生成（clk_fx域） ======================
// 功能：生成周期性门控信号，包含前导低电平、测量高电平和结束低电平
always @(posedge clk_fx or negedge rst_n) begin
    if(!rst_n) begin
        gate_cnt <= 10'd0;
        gate <= 1'b0;
    end 
    // 当计数器达到门控时间+20时复位（20周期为保护间隔）
    else if(gate_cnt == GATE_TIME + 5'd20) begin
        gate_cnt <= 10'd0;
    end 
    else begin
        gate_cnt <= gate_cnt + 1'b1;
        // 门控信号时序控制：
        //   0-9周期：低电平（前导）
        //   10至(10+GATE_TIME-1)：高电平（测量窗口）
        //   其余周期：低电平（结束）
        if(gate_cnt < 4'd10)
            gate <= 1'b0;
        else if(gate_cnt < 4'd10 + GATE_TIME)
            gate <= 1'b1;
        else
            gate <= 1'b0;
    end
end

// ====================== 跨时钟域同步（clk_fx域->clk_fs域） ======================
// 功能：将门控信号同步到基准时钟域（两级同步消除亚稳态）
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        gate_fs_r <= 1'b0;
        gate_fs   <= 1'b0;
    end else begin
        gate_fs_r <= gate;      // 第一级同步
        gate_fs   <= gate_fs_r; // 第二级同步
    end
end

// 在基准时钟域检测门控信号边沿（用于产生完成脉冲）
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        gate_fs_d0 <= 1'b0; 
        gate_fs_d1 <= 1'b0;
    end else begin
        gate_fs_d0 <= gate_fs;   // 当前值
        gate_fs_d1 <= gate_fs_d0; // 延迟一拍
    end
end

// ====================== 占空比测量（clk_fs域） ======================
// 功能：在基准时钟域采样被测时钟，统计高/低电平时间
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        clk_fx_sync0 <= 1'b0; 
        clk_fx_sync1 <= 1'b0;
        high_temp    <= 0; 
        low_temp     <= 0;
    end else begin
        // 两级同步被测时钟信号（消除亚稳态）
        clk_fx_sync0 <= clk_fx;      
        clk_fx_sync1 <= clk_fx_sync0;
        
        // 只在同步门控有效期间计数
        if(gate_fs) begin
            if(clk_fx_sync1) // 高电平计数
                high_temp <= high_temp + 1'b1;
            else             // 低电平计数
                low_temp <= low_temp + 1'b1;
        end 
        // 门控结束时锁存结果
        else if(neg_gate_fs) begin
            high_cycles <= high_temp;
            low_cycles  <= low_temp;
            high_temp   <= 0;  // 复位临时计数器
            low_temp    <= 0;
        end
    end
end

// ====================== 结果计算（clk_fs域） ======================
// 功能：在门控结束时只计算占空比并产生完成脉冲
reg [31:0] total_cycles; // 临时变量：总周期数

always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        measure_done   <= 1'b0;
    end 
    // 在同步门控下降沿计算占空比并产生单周期完成脉冲
    else if(neg_gate_fs) begin
        // total_cycles = high_cycles + low_cycles; // 总基准时钟周期数
        // if(total_cycles != 0) begin
        //     // 使用隐式64位计算避免溢出
        //     duty_permille <= (high_cycles * 1000) / total_cycles;
        // end else begin
        //     duty_permille <= 10'd0; // 避免除零
        // end

        measure_done <= 1'b1; // 产生单周期完成脉冲
    end else begin
        measure_done <= 1'b0; // 保持完成信号为低
    end
end

divider #(
    .WIDTH(40),
    .CACHING(0),
    .INIT_VLD(1)
) divider_inst(
    .clk(clk_fs),
    .rst(!rst_n),
    .start(neg_gate_fs),
    .dividend(high_cycles * 1000),
    .divisor(high_cycles + low_cycles),
    .quotient(duty_permille),
    .valid()  // 未使用
);


endmodule
