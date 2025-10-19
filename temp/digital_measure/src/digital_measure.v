module digital_measure#(
    parameter CLK_FS  = 50_000_000   // 系统时钟频率 (Hz)
)(
    input             clk_fs,        // 系统时钟 (基准时钟)
    input             rst_n,         // 异步复位 (低有效)
    input             clk_fx,        // 外部信号输入 (被测时钟)
    input      [15:0] GATE_TIME,     // 门控时间计数值 (外部时钟周期数)
    output reg [31:0] frequence,     // 外部信号频率输出 (Hz)
    output reg        measure_done   // 测量完成标志 (单周期脉冲)
);
// ====================== 参数定义 ======================
localparam MAX_CNT_WIDTH = 32;       // 计数器最大位宽
// ====================== 寄存器定义 ======================
reg                  gate;              // 外部时钟域门控信号
reg                  gate_fs;           // 基准时钟门控信号
reg                  gate_fs_r;         // 同步门控信号寄存器
reg                  gate_fs_d0;        // 基准时钟下门控信号下降沿检测
reg                  gate_fs_d1;
reg                  gate_fx_d0;        // 外部时钟下门控信号下降沿检测
reg                  gate_fx_d1;
reg   [15:0]         gate_cnt;          // 门控时间计数器
reg   [MAX_CNT_WIDTH-1:0] fs_cnt;       // 系统时钟计数值 (锁存)
reg   [MAX_CNT_WIDTH-1:0] fs_cnt_temp;  // 系统时钟计数器 (临时)
reg   [MAX_CNT_WIDTH-1:0] fx_cnt;       // 外部时钟计数值 (锁存)
reg   [MAX_CNT_WIDTH-1:0] fx_cnt_temp;  // 外部时钟计数器 (临时)
// 跨时钟域同步寄存器
reg   [MAX_CNT_WIDTH-1:0] fx_cnt_sync0; // 第一级同步
reg   [MAX_CNT_WIDTH-1:0] fx_cnt_sync1; // 第二级同步
// ====================== 线网定义 ======================
wire neg_gate_fs;                       // 基准时钟门控下降沿
wire neg_gate_fx;                       // 外部时钟门控下降沿
// ====================== 边沿检测 ======================
assign neg_gate_fs = gate_fs_d1 & (~gate_fs_d0);  // 系统时钟域下降沿
assign neg_gate_fx = gate_fx_d1 & (~gate_fx_d0);  // 外部时钟域下降沿
// ====================== 门控信号生成 ======================
// 外部时钟域计数器
always @(posedge clk_fx or negedge rst_n) begin
    if(!rst_n)
        gate_cnt <= 16'd0;
    else if(gate_cnt == GATE_TIME + 19)  // 总计数周期 = 20(低) + GATE_TIME(高)
        gate_cnt <= 16'd0;
    else
        gate_cnt <= gate_cnt + 1'b1;
end
// 门控信号 (高电平持续GATE_TIME个外部时钟周期)
always @(posedge clk_fx or negedge rst_n) begin
    if(!rst_n)
        gate <= 1'b0;
    else if(gate_cnt < 4'd20)      // 前20个周期: 低电平
        gate <= 1'b0;
    else if(gate_cnt < 4'd20 + GATE_TIME) // 中间GATE_TIME周期: 高电平
        gate <= 1'b1;
    else                           // 其余时间: 低电平
        gate <= 1'b0;
end
// ====================== 跨时钟域同步 ======================
// 门控信号同步到系统时钟域
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        gate_fs_r <= 1'b0;
        gate_fs   <= 1'b0;
    end
    else begin
        gate_fs_r <= gate;
        gate_fs   <= gate_fs_r;
    end
end
// 系统时钟域下降沿检测
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        gate_fs_d0 <= 1'b0;
        gate_fs_d1 <= 1'b0;
    end
    else begin
        gate_fs_d0 <= gate_fs;
        gate_fs_d1 <= gate_fs_d0;
    end
end
// 外部时钟域下降沿检测
always @(posedge clk_fx or negedge rst_n) begin
    if(!rst_n) begin
        gate_fx_d0 <= 1'b0;
        gate_fx_d1 <= 1'b0;
    end
    else begin
        gate_fx_d0 <= gate;
        gate_fx_d1 <= gate_fx_d0;
    end
end
// ====================== 时钟计数 ======================
// 系统时钟计数 (门控期间)
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        fs_cnt_temp <= 0;
        fs_cnt      <= 0;
    end
    else begin
        if(gate_fs) begin
            fs_cnt_temp <= fs_cnt_temp + 1'b1;  // 门控高电平期间计数
        end
        else if(neg_gate_fs) begin              // 检测到下降沿
            fs_cnt      <= fs_cnt_temp;          // 锁存计数值
            fs_cnt_temp <= 0;                    // 复位临时计数器
        end
    end
end
// 外部时钟计数 (门控期间)
always @(posedge clk_fx or negedge rst_n) begin
    if(!rst_n) begin
        fx_cnt_temp <= 0;
        fx_cnt      <= 0;
    end
    else begin
        if(gate) begin
            fx_cnt_temp <= fx_cnt_temp + 1'b1;  // 门控高电平期间计数
        end
        else if(neg_gate_fx) begin              // 检测到下降沿
            fx_cnt      <= fx_cnt_temp;          // 锁存计数值
            fx_cnt_temp <= 0;                    // 复位临时计数器
        end
    end
end
// ====================== 跨时钟域同步外部计数值 ======================
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        fx_cnt_sync0 <= 0;
        fx_cnt_sync1 <= 0;
    end
    else begin
        fx_cnt_sync0 <= fx_cnt;      // 第一级同步
        fx_cnt_sync1 <= fx_cnt_sync0; // 第二级同步
    end
end

reg                fx_transfer_toggle;     // toggled in fx domain when fx_cnt captured
reg                fx_transfer_toggle_r0;  // sync stage 0 in fs domain
reg                fx_transfer_toggle_r1;  // sync stage 1 in fs domain
reg  [MAX_CNT_WIDTH-1:0] fx_cnt_latched;    // latched copy of fx_cnt in fs domain

always @(posedge clk_fx or negedge rst_n) begin
    if(!rst_n) begin
        fx_transfer_toggle <= 1'b0;
    end
    else begin
        if(neg_gate_fx) begin
            fx_transfer_toggle <= ~fx_transfer_toggle;
        end
    end
end

always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        fx_transfer_toggle_r0 <= 1'b0;
        fx_transfer_toggle_r1 <= 1'b0;
        fx_cnt_latched <= 0;
    end
    else begin
        fx_transfer_toggle_r0 <= fx_transfer_toggle;
        fx_transfer_toggle_r1 <= fx_transfer_toggle_r0;
        // detect edge: when r1 != r0 then a new transfer occurred
        if(fx_transfer_toggle_r0 != fx_transfer_toggle_r1) begin
            fx_cnt_latched <= fx_cnt; // capture stable fx_cnt (it is from fx domain, but captured shortly after toggle)
        end
    end
end
// ====================== 频率计算 ======================
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        frequence <= 0;
    end
    else if(neg_gate_fs) begin  // 在门控下降沿计算频率
        if(fs_cnt != 0) begin   // 避免除零错误
            frequence <= ({48'h0,CLK_FS} * fx_cnt_latched) / fs_cnt;
        end
        else begin
            frequence <= 0;   // 无效测量值
        end
    end
end

// ====================== 测量完成标志 ======================
always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        measure_done <= 0;
    end
    else begin
        // 单周期脉冲 (门控下降沿)
        measure_done <= neg_gate_fs;
    end
end
endmodule