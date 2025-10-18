module pwm_drive #(
    parameter CLK_FREQ = 50_000_000
)(
    input clk,
    input rst_n,

    input [15:0] PSC,      // 预分频器
    input [15:0] ARR,      // 自动重装值
    input [ 7:0] compare,  // 占空比 (0~255 线性映射为0~100%的占空比)


    input pwm_start,       // pwm发生信号端口(1/0 发生/关闭)

    output reg PWM             //pwm输出端口
);

// reg define
reg [15:0] PSC_cnt;   // 预分频计数器
reg [15:0] ARR_cnt;   // 自动重装计数器

// wire define
wire [15:0] CCR;      // 占空比比较值

wire PSC_done;        // 预分频结束信号
wire ARR_done;        // 自动重装结束信号

assign CCR      = ((ARR + 1) * compare) >> 8;
assign PSC_done = (PSC_cnt == PSC);
assign ARR_done = (ARR_cnt == ARR);

// 预分频器计数逻辑
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        PSC_cnt <= 16'd0;
    else if (!pwm_start)
        PSC_cnt <= 16'd0;
    else if (PSC_done)
        PSC_cnt <= 16'd0;
    else
        PSC_cnt <= PSC_cnt + 1'b1;
end

// 自动重装计数逻辑
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        ARR_cnt <= 16'd0;
    else if (!pwm_start)
        ARR_cnt <= 16'd0;
    else if (PSC_done) begin
        if (ARR_done)
            ARR_cnt <= 16'd0;
        else
            ARR_cnt <= ARR_cnt + 1'b1;
    end
    else
        ARR_cnt <= ARR_cnt;
end

// PWM输出逻辑
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        PWM <= 1'b0;
    else if (!pwm_start)
        PWM <= 1'b0;
    else begin
        if (compare == 8'd255)
            PWM <= 1'b1;
        else
            PWM <= (CCR > ARR_cnt) ? 1'b1 : 1'b0;
    end
end

endmodule

