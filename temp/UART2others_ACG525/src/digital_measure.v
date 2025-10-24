module digital_measure #(
    parameter CLK_FS = 200_000_000
) (
    input  wire        rst_n,

    input  wire        clk_fs,
    input  wire        clk_fx,

    output reg  [31:0] digital_frequency,
    output reg  [ 9:0] digital_duty,
    output reg         digital_done
);

// reg define
reg [ 9:0] freq_gate_time;
reg [ 9:0] duty_gate_time;

wire       measure_freq_done;
wire       measure_duty_done;
wire [31:0] digital_frequency_temp;
wire [ 9:0] digital_duty_temp;

// FSM 状态
reg [2:0] state;
reg measure_freq_done_d;
reg measure_duty_done_d;

// localparam define
localparam LOW_FREQ_GATE_TIME      = 50;    // 粗略/细测门控时间参考值
localparam MEDIUM_FREQ_GATE_TIME   = 200;
localparam HIGH_FREQ_GATE_TIME     = 500;
localparam VERY_HIGH_FREQ_ATE_TIME = 1000;
localparam LOW_DUTY_GATE_TIME      = 50;    // 占空比门控时间
localparam MEDIUM_DUTY_GATE_TIME   = 100;
localparam HIGH_DUTY_GATE_TIME     = 200;

// 频率阈值 (Hz)
localparam FREQ_TH_10K  = 10_000;
localparam FREQ_TH_100K = 100_000;
localparam FREQ_TH_1M   = 1_000_000;

// ======================频率测量模块实例化======================
digital_freq_measure #(
    .CLK_FS(CLK_FS)
) digital_freq_measure_inst (
    .clk_fs      (clk_fs),
    .rst_n       (rst_n),
    .clk_fx      (clk_fx),
    .GATE_TIME   (freq_gate_time),
    .frequency   (digital_frequency_temp),
    .measure_done(measure_freq_done)
);

// ======================占空比测量模块实例化=======================
digital_duty_measure #(
    .CLK_FS(CLK_FS)
) digital_duty_measure_inst (
    .clk_fs      (clk_fs),
    .rst_n       (rst_n),
    .clk_fx      (clk_fx),
    .GATE_TIME   (duty_gate_time),
    .duty_permille(digital_duty_temp),
    .measure_done(measure_duty_done)
);

// ====================== 顶层 FSM：先粗测量频率，再细测量并测占空比 ======================
localparam S_IDLE        = 3'd0;
localparam S_COARSE_WAIT = 3'd1;
localparam S_DECIDE      = 3'd2;
localparam S_FINE_WAIT   = 3'd3;
localparam S_DUTY_WAIT   = 3'd4;
localparam S_OUTPUT      = 3'd5;

always @(posedge clk_fs or negedge rst_n) begin
    if(!rst_n) begin
        state <= S_IDLE;
        freq_gate_time <= LOW_FREQ_GATE_TIME;
        duty_gate_time <= LOW_DUTY_GATE_TIME;
        digital_frequency <= 32'd0;
        digital_duty <= 10'd0;
        digital_done <= 1'b0;
        measure_freq_done_d <= 1'b0;
        measure_duty_done_d <= 1'b0;
    end else begin
        // 边沿检测
        measure_freq_done_d <= measure_freq_done;
        measure_duty_done_d <= measure_duty_done;

        case(state)
            S_IDLE: begin
                // 启动粗略测量：使用较小门控时间以快速判断频率级别
                freq_gate_time <= LOW_FREQ_GATE_TIME;
                duty_gate_time <= LOW_DUTY_GATE_TIME;
                state <= S_COARSE_WAIT;
                digital_done <= 1'b0;
            end

            S_COARSE_WAIT: begin
                // 等待频率测量模块完成一次测量（检测上升沿）
                if(measure_freq_done && !measure_freq_done_d) begin
                    // coarse result available in digital_frequency_temp
                    state <= S_DECIDE;
                end
            end

            S_DECIDE: begin
                // 根据粗测量结果选择细测门控时间和占空比门控时间
                if(digital_frequency_temp <= FREQ_TH_10K) begin
                    freq_gate_time <= LOW_FREQ_GATE_TIME;
                    duty_gate_time <= LOW_DUTY_GATE_TIME;
                    // digital_frequency <= digital_frequency_temp;
                    // state <= S_FINE_WAIT;
                end else if(digital_frequency_temp <= FREQ_TH_100K) begin
                    freq_gate_time <= MEDIUM_FREQ_GATE_TIME;
                    duty_gate_time <= MEDIUM_FREQ_GATE_TIME;
                    // state <= S_FINE_WAIT;
                end else if(digital_frequency_temp <= FREQ_TH_1M) begin
                    freq_gate_time <= HIGH_FREQ_GATE_TIME;
                    duty_gate_time <= HIGH_DUTY_GATE_TIME;
                    // state <= S_FINE_WAIT;
                end else begin
                    freq_gate_time <= VERY_HIGH_FREQ_ATE_TIME;
                    duty_gate_time <= HIGH_DUTY_GATE_TIME;
                    
                end
                state <= S_FINE_WAIT;
            end

            S_FINE_WAIT: begin
                if(measure_freq_done && !measure_freq_done_d) begin
                    // 细测完成，采样最终频率与占空比
                    digital_frequency <= digital_frequency_temp;
                    state <= S_DUTY_WAIT;
                end
            end

            S_DUTY_WAIT: begin
                if(measure_duty_done && !measure_duty_done_d) begin
                    digital_duty <= digital_duty_temp;
                    state <= S_OUTPUT;
                end
            end

            S_OUTPUT: begin
                digital_done <= 1'b1; // 输出完成脉冲
                // 下个周期清脉冲并回到空闲开始下一轮测量
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule