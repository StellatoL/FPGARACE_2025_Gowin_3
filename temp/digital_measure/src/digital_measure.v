module digital_measure#(
    parameter    CLK_FS = 26'd50_000_000 // 基准时钟频率值 (Hz)
) (
    input  wire        clk_fs,    // 基准时钟（参考时钟）
    input  wire        rst_n,     // 低有效复位
    input  wire        clk_fx,    // 被测时钟信号
    input       [15:0] GATE_TIME,
    output reg  [31:0] frequency,   // 频率输出，Hz
    output reg  [31:0] high_cycles, // 门控期间高电平计数（基准时钟周期数）
    output reg  [31:0] low_cycles,  // 门控期间低电平计数（基准时钟周期数）
    output reg  [15:0] duty_permille, // 占空比，千分制（0..1000）
    output reg         measure_done
);
// 参数
localparam MAX = 6'd32;
// 门控产生（在被测时钟域）
reg gate;
reg [15:0] gate_cnt;
// 基准域同步门控信号
reg gate_fs_r;
reg gate_fs;
reg gate_fs_d0, gate_fs_d1;
// 被测域下降沿打拍
reg gate_fx_d0, gate_fx_d1;
// 计数器
reg [MAX-1:0] fs_cnt_temp, fs_cnt;
reg [MAX-1:0] fx_cnt_temp, fx_cnt;
// 高低电平计数（在基准时钟域采样clk_fx）
reg clk_fx_sync0, clk_fx_sync1;
reg [31:0] high_temp, low_temp;
// 边沿检测
wire neg_gate_fs = gate_fs_d1 & (~gate_fs_d0);
wire neg_gate_fx = gate_fx_d1 & (~gate_fx_d0);
// -------------- 门控产生（在 clk_fx 域） --------------
always @(posedge clk_fx or negedge rst_n) begin
    if(!rst_n) begin
        gate_cnt <= 16'd0;
        gate <= 1'b0;
    end else if(gate_cnt == GATE_TIME + 5'd20) begin
        gate_cnt <= 16'd0;
    end else begin
        gate_cnt <= gate_cnt + 1'b1;
        // gate 信号：前10个周期低电平，中间 GATE_TIME 周期高电平
        if(gate_cnt < 4'd10) gate <= 1'b0;
        else if(gate_cnt < 4'd10 + GATE_TIME) gate <= 1'b1;
        else gate <= 1'b0;
    end
end
// 将 gate 同步到基准时钟域（两级）
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        gate_fs_r <= 1'b0;
        gate_fs <= 1'b0;
    end else begin
        gate_fs_r <= gate;
        gate_fs <= gate_fs_r;
    end
end
// 采集门控下降沿（基准域与被测域）
always @(posedge clk_fx or negedge rst_n) begin
    if(!rst_n) begin
        gate_fx_d0 <= 1'b0; gate_fx_d1 <= 1'b0;
    end else begin
        gate_fx_d0 <= gate;
        gate_fx_d1 <= gate_fx_d0;
    end
end
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        gate_fs_d0 <= 1'b0; gate_fs_d1 <= 1'b0;
    end else begin
        gate_fs_d0 <= gate_fs;
        gate_fs_d1 <= gate_fs_d0;
    end
end
// --------------- 被测时钟计数（在 clk_fx 域） ---------------
always @(posedge clk_fx or negedge rst_n) begin
    if(!rst_n) begin
        fx_cnt_temp <= 0; fx_cnt <= 0;
    end else begin
        if(gate) fx_cnt_temp <= fx_cnt_temp + 1'b1; // 在 gate 高期间计数 fx 的周期数
        else if(neg_gate_fx) begin
            fx_cnt <= fx_cnt_temp; // 在下降沿锁存
            fx_cnt_temp <= 0;
        end
    end
end
// --------------- 参考时钟计数（在 clk_fs 域） ---------------
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        fs_cnt_temp <= 0; fs_cnt <= 0;
    end else begin
        if(gate_fs) fs_cnt_temp <= fs_cnt_temp + 1'b1;
        else if(neg_gate_fs) begin
            fs_cnt <= fs_cnt_temp;
            fs_cnt_temp <= 0;
        end
    end
end
// --------------- 在基准时钟域采样被测信号以统计高/低时间 ---------------
// 双触发器同步 clk_fx
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        clk_fx_sync0 <= 1'b0; clk_fx_sync1 <= 1'b0;
        high_temp <= 0; low_temp <= 0;
    end else begin
        clk_fx_sync0 <= clk_fx;
        clk_fx_sync1 <= clk_fx_sync0;
        if(gate_fs) begin
            if(clk_fx_sync1) high_temp <= high_temp + 1'b1;
            else low_temp <= low_temp + 1'b1;
        end else if(neg_gate_fs) begin
            high_cycles <= high_temp;
            low_cycles <= low_temp;
            high_temp <= 0; low_temp <= 0;
        end
    end
end
// --------------- 计算频率、占空比并产生测量完成脉冲 ---------------
// 在基准时钟域的门控下降沿时进行计算
reg [31:0] total_cycles;
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        frequency <= 32'd0;
        duty_permille <= 16'd0;
        measure_done <= 1'b0;
    end else if(neg_gate_fs) begin
        // 频率 = fx_cnt * CLK_FS / fs_cnt
        if(fs_cnt != 0) begin
            frequency <= {48'h0, fx_cnt} * CLK_FS / fs_cnt;
        end else
            frequency <= 32'd0;
        // 占空比（千分制）: duty = high / (high+low) * 1000
        total_cycles = high_cycles + low_cycles;
        if(total_cycles != 0) begin
            // use 64-bit intermediate to avoid overflow
            duty_permille <= {32'h0,(high_cycles * 1000)} / (total_cycles);
        end else duty_permille <= 16'd0;
        measure_done <= 1'b1; // 单周期脉冲
    end else begin
        measure_done <= 1'b0;
    end
end
endmodule