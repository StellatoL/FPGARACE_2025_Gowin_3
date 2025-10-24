module I2C_drive #(
    parameter CLK_FREQ          = 100_000_000, // 系统时钟频率 (100MHz)
    parameter I2C_FREQ          = 400_000,     // I2C时钟频率 (400KHz)
    parameter REG_ADDR_BYTE_NUM = 2,           // 寄存器地址字节数
    parameter DATA_BYTE_NUM     = 8            // 最大读写数据字节数 (默认8字节)
)(
    input                                  clk,           // 系统时钟信号
    input                                  rst_n,         // 系统复位信号，低电平有效
    input                                  start,         // 开始进行读写操作
    input               [6:0]              DEVICE_ADDR,   // 器件地址
    input                                  rw_flag,       // 读写标志信号 (1=读, 0=写)
    input                                  reg_size_crtl, // 寄存器地址大小标志位(1/0,16bit/8bit)
    input      [REG_ADDR_BYTE_NUM*8-1 : 0] reg_addr,      // 寄存器地址
    input      [DATA_BYTE_NUM*8 - 1 : 0]   wdata,         // 写数据
    input      [7:0]                       user_byte_num, // 用户指定的实际传输字节数 (1-DATA_BYTE_NUM)
    output reg [DATA_BYTE_NUM*8 - 1 : 0]   rdata,         // 读数据信号
    output reg                             rdata_vld,     // 读数据输出有效信号
    output reg [DATA_BYTE_NUM-1:0]         byte_valid,    // 字节有效掩码 (每个bit对应一个字节)
    output reg                             IIC_done,      // 模块忙闲指示信号
    output reg                             scl,           // IIC时钟信号
    inout                                  sda,           // IIC双向数据信号
    output reg                             ack_flag       // 应答失败标志
);
    // 内部参数计算
    localparam CLK_DIV    = CLK_FREQ / I2C_FREQ; // 计算分频系数
    localparam CLK_DIV_W  = clogb2(CLK_DIV - 1); // 分频计数器位宽
    localparam BYTE_CNT_W = clogb2(DATA_BYTE_NUM > REG_ADDR_BYTE_NUM ? 
                            DATA_BYTE_NUM-1 : REG_ADDR_BYTE_NUM-1); // 字节计数器位宽
    
    // 状态定义 (独热码)
    localparam IDLE          = 7'b0000001; // 空闲状态
    localparam W_DEVICE_ADDR = 7'b0000010; // 写器件地址状态
    localparam W_REG_ADDR    = 7'b0000100; // 写寄存器地址状态
    localparam WDATA         = 7'b0001000; // 写数据状态
    localparam R_DEVICE_ADDR = 7'b0010000; // 读器件地址状态
    localparam RDATA         = 7'b0100000; // 读数据状态
    localparam STOP          = 7'b1000000; // 停止状态
    
    // 内部寄存器
    reg                                l2h_flag;
    reg                                h2l_flag;
    reg                                wr_flag;
    reg                                rd_flag;
    reg                                end_div_cnt;
    reg                                rw_flag_r;
    reg                                sda_out;
    reg                                sda_out_en;
    reg  [6:0]                         state_n;
    reg  [6:0]                         state_c;
    reg  [3:0]                         bit_cnt;
    reg  [3:0]                         bit_cnt_num;
    reg  [CLK_DIV_W - 1 : 0]           div_cnt;
    reg  [BYTE_CNT_W - 1 : 0]          byte_cnt;
    reg  [BYTE_CNT_W - 1 : 0]          byte_cnt_num;
    reg  [DATA_BYTE_NUM*8 - 1 : 0]     wdata_r;
    reg  [REG_ADDR_BYTE_NUM*8 - 1 : 0] reg_addr_r;
    reg  [DATA_BYTE_NUM*8 - 1 : 0]     rdata_r;
    reg                                rdata_vld_r;
    reg  [7:0]                         actual_byte_num; // 实际传输字节数寄存器
    reg  [DATA_BYTE_NUM-1:0]           byte_valid_r;    // 字节有效掩码寄存器
    reg                                reg_size_crtl_r; // 寄存器地址大小控制寄存器
    
    wire                               add_byte_cnt;
    wire                               end_byte_cnt;
    wire [8:0]                         device_addr;
    wire                               sda_in;
    wire                               add_bit_cnt;
    wire                               end_bit_cnt;
    
    // 双向IO控制
    assign sda_in = sda;
    assign sda = sda_out_en ? sda_out : 1'bz;
    
    // 自动计算位宽函数
    function integer clogb2(input integer depth);
        begin
            if(depth == 0)
                clogb2 = 1;
            else if(depth != 0)
                for(clogb2=0 ; depth>0 ; clogb2=clogb2+1)
                    depth = depth >> 1;
        end
    endfunction
    
    // 器件地址组合
    assign device_addr = {1'b0, DEVICE_ADDR, 1'b0};
    
    // 暂存输入信号和实际字节数
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wdata_r <= 0;
            rw_flag_r <= 1'b0;
            reg_addr_r <= 0;
            actual_byte_num <= 0;
            reg_size_crtl_r <= 0;
        end
        else if (start) begin
            wdata_r <= wdata;
            rw_flag_r <= rw_flag;
            reg_addr_r <= reg_addr;
            reg_size_crtl_r <= reg_size_crtl; // 暂存寄存器地址大小控制信号
            // 确保字节数在有效范围内 (1到DATA_BYTE_NUM)
            actual_byte_num <= (user_byte_num > 0 && user_byte_num <= DATA_BYTE_NUM) ? 
                              user_byte_num : DATA_BYTE_NUM;
        end
    end
    
    // 生成字节有效掩码
    always @(*) begin
        byte_valid_r = 0;
        byte_valid_r[0] = (0 < actual_byte_num) ? 1'b1 : 1'b0;
        byte_valid_r[1] = (1 < actual_byte_num) ? 1'b1 : 1'b0;
        byte_valid_r[2] = (2 < actual_byte_num) ? 1'b1 : 1'b0;
        byte_valid_r[3] = (3 < actual_byte_num) ? 1'b1 : 1'b0;
    end
    
    // 状态机现态寄存器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_c <= IDLE;
        end
        else begin
            state_c <= state_n;
        end
    end
    
    // IIC_done信号用时序逻辑生成，避免锁存器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            IIC_done <= 1'b1; // 复位时设为空闲
        end
        else if (start || state_c == IDLE) begin
            IIC_done <= 1'b0;
        end
        else if (state_c == STOP && end_div_cnt) begin
            IIC_done <= 1'b1; // 停止状态结束时完成
        end
    end


    // 状态机次态逻辑
    always @(*) begin
        state_n = state_c; // 默认保持当前状态
        
        case (state_c)
            IDLE: begin
                if (start) begin
                    state_n = W_DEVICE_ADDR;
                end
                else begin
                    state_n = state_c;
                end
            end
            
            W_DEVICE_ADDR: begin
                if (end_bit_cnt) begin
                    state_n = W_REG_ADDR;
                end
                else begin
                    state_n = state_c;
                end
            end
            
            W_REG_ADDR: begin
                if (end_byte_cnt) begin
                    if (rw_flag_r) begin
                        state_n = R_DEVICE_ADDR;
                    end
                    else begin
                        state_n = WDATA;
                    end
                end
                else begin
                    state_n = state_c;
                end
            end
            
            WDATA: begin
                if (end_byte_cnt) begin
                    state_n = STOP;
                end
                else begin
                    state_n = state_c;
                end
            end
            
            R_DEVICE_ADDR: begin
                if (end_bit_cnt) begin
                    state_n = RDATA;
                end
                else begin
                    state_n = state_c;
                end
            end
            
            RDATA: begin
                if (end_byte_cnt) begin
                    state_n = STOP;
                end
                else begin
                    state_n = state_c;
                end
            end
            
            STOP: begin
                if (end_div_cnt) begin
                    state_n = IDLE;
                end
                else begin
                    state_n = state_c;
                end
            end
            
            default: state_n = IDLE;
        endcase
    end
    
    // 分频计数器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 0;
        end
        else if (state_c != IDLE) begin
            if (end_div_cnt)
                div_cnt <= 0;
            else
                div_cnt <= div_cnt + 1;
        end
        else
            div_cnt <= 0;
    end
    
    // 分频计数器标志生成
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            l2h_flag <= 1'b0;
            h2l_flag <= 1'b0;
            wr_flag  <= 1'b0;
            rd_flag  <= 1'b0;
            end_div_cnt <= 1'b0;
        end
        else begin
            l2h_flag <= (div_cnt == (CLK_DIV / 2) - 1);
            h2l_flag <= (div_cnt == CLK_DIV - 1);
            end_div_cnt <= (div_cnt == CLK_DIV - 2);
            wr_flag <= (div_cnt == CLK_DIV / 4);
            rd_flag <= (div_cnt == (3 * CLK_DIV) / 4);
        end
    end
    
    // 位计数器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt <= 0;
        end
        else if (state_c == IDLE) begin
            bit_cnt <= 0;
        end
        else if (add_bit_cnt) begin
            if (end_bit_cnt) begin
                bit_cnt <= 0;
            end
            else begin
                bit_cnt <= bit_cnt + 1;
            end
        end
    end
    
    assign add_bit_cnt = end_div_cnt;
    assign end_bit_cnt = add_bit_cnt && (bit_cnt == bit_cnt_num - 1);
    
    // 位计数器最大值设置
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt_num <= 4'd9;
        end
        else if ((state_c == W_DEVICE_ADDR) || (state_c == R_DEVICE_ADDR)) begin
            bit_cnt_num <= 4'd10;
        end
        else begin
            bit_cnt_num <= 4'd9;
        end
    end
    
    // 字节计数器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_cnt <= 0;
        end
        else if (state_c == IDLE) begin
            byte_cnt <= 0;
        end
        else if (add_byte_cnt) begin
            if (end_byte_cnt) begin
                byte_cnt <= 0;
            end
            else begin
                byte_cnt <= byte_cnt + 1;
            end
        end
    end
    
    assign add_byte_cnt = ((state_c == W_REG_ADDR) || (state_c == WDATA) || (state_c == RDATA)) && end_bit_cnt;
    assign end_byte_cnt = add_byte_cnt && (byte_cnt == byte_cnt_num);
    
    // 字节计数器最大值设置 (基于实际字节数)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_cnt_num <= REG_ADDR_BYTE_NUM - 1;
        end
        else begin
            case (state_c)
                W_REG_ADDR: begin
                    // 根据寄存器地址大小控制决定地址字节数
                    if (reg_size_crtl_r) // 16位地址
                        byte_cnt_num <= 2 - 1;
                    else // 8位地址
                        byte_cnt_num <= 1 - 1;
                end
                WDATA: begin
                    byte_cnt_num <= actual_byte_num - 1; // 使用实际字节数
                end
                RDATA: begin
                    byte_cnt_num <= actual_byte_num - 1; // 使用实际字节数
                end
                default: byte_cnt_num <= byte_cnt_num;
            endcase
        end
    end
    
    // SCL生成逻辑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl <= 1'b1;
        end
        else if (l2h_flag || state_c == IDLE) begin
            scl <= 1'b1;
        end
        else if ((((state_c == W_DEVICE_ADDR) && (bit_cnt > 0)) || 
                 (state_c != W_DEVICE_ADDR)) && h2l_flag) begin
            scl <= 1'b0;
        end
    end
    
    // SDA输出逻辑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sda_out <= 1'b1;
        end
        else begin
            case (state_c)
                W_DEVICE_ADDR: begin
                    if ((~(bit_cnt[3] & bit_cnt[0])) && wr_flag) begin
                        sda_out <= device_addr[8 - bit_cnt];
                    end
                end
                W_REG_ADDR: begin
                    if ((~bit_cnt[3]) && wr_flag) begin
                        // 根据寄存器地址大小选择发送的地址字节
                        if (reg_size_crtl_r) begin // 16位地址
                            // 当byte_cnt=0时发送高8位，byte_cnt=1时发送低8位
                            if (byte_cnt == 0)
                                sda_out <= reg_addr_r[15 - bit_cnt]; // 高8位
                            else
                                sda_out <= reg_addr_r[7 - bit_cnt];  // 低8位
                        end
                        else begin // 8位地址
                            sda_out <= reg_addr_r[7 - bit_cnt];      // 仅发送低8位
                        end
                    end
                end
                WDATA: begin
                    if ((~bit_cnt[3]) && wr_flag) begin
                        // 只输出有效字节
                        if (byte_cnt < actual_byte_num) begin
                            sda_out <= wdata_r[DATA_BYTE_NUM*8 - 1 - byte_cnt*8 - bit_cnt];
                        end
                        else begin
                            sda_out <= 1'b1; // 无效字节输出高电平
                        end
                    end
                end
                R_DEVICE_ADDR: begin
                    if (wr_flag) begin
                        if (bit_cnt == 0 || bit_cnt == bit_cnt_num - 2) begin
                            sda_out <= 1'b1;
                        end
                        else begin
                            sda_out <= device_addr[8 - bit_cnt];
                        end
                    end
                    else if (rd_flag && bit_cnt == 0) begin
                        sda_out <= 1'b0;
                    end
                end
                RDATA: begin
                    if (bit_cnt == bit_cnt_num - 1 && wr_flag) begin
                        if (byte_cnt == actual_byte_num - 1) begin
                            sda_out <= 1'b1; // 最后一字节不应答
                        end
                        else begin
                            sda_out <= 1'b0; // 其他字节应答
                        end
                    end
                end
                STOP: begin
                    if (wr_flag) begin
                        sda_out <= 1'b0;
                    end
                    else if (rd_flag) begin
                        sda_out <= 1'b1;
                    end
                end
                default: sda_out <= sda_out;
            endcase
        end
    end
    
    // SDA输出使能控制
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sda_out_en <= 1'b1;
        end
        else if (wr_flag) begin
            case (state_c)
                W_DEVICE_ADDR, WDATA, R_DEVICE_ADDR, W_REG_ADDR: begin
                    if (bit_cnt == 0) begin
                        sda_out_en <= 1'b1;
                    end
                    else if (bit_cnt == bit_cnt_num - 1) begin
                        sda_out_en <= 1'b0;
                    end
                end
                STOP: begin
                    if (bit_cnt == 0) begin
                        sda_out_en <= 1'b1;
                    end
                end
                RDATA: begin
                    if (bit_cnt == 0) begin
                        sda_out_en <= 1'b0;
                    end
                    else if (bit_cnt == bit_cnt_num - 1) begin
                        sda_out_en <= 1'b1;
                    end
                end
                default: ;
            endcase
        end
    end
    
    // 读数据处理
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdata_r <= 0;
        end
        else if (state_c == RDATA && rd_flag) begin
            // 只存储有效字节
            if (byte_cnt < actual_byte_num) begin
                rdata_r[DATA_BYTE_NUM*8 - 1 - byte_cnt*8 - bit_cnt] <= sda_in;
            end
        end
    end
    
    // 读数据有效信号
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdata_vld_r <= 1'b0;
        end
        else begin
            rdata_vld_r <= (state_c == RDATA) && rd_flag && 
                          (bit_cnt == bit_cnt_num - 2) && 
                          (byte_cnt == byte_cnt_num);
        end
    end
    
    // 输出信号
    always @(posedge clk) begin
        rdata <= rdata_vld_r ? rdata_r : rdata;
        rdata_vld <= rdata_vld_r;
        byte_valid <= rdata_vld_r ? byte_valid_r : byte_valid;
    end
    
    // 从机应答检测
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack_flag <= 1'b0;
        end
        else if (start) begin
            ack_flag <= 1'b0;
        end
        else if (((state_c == WDATA) || (state_c == W_DEVICE_ADDR) || 
                 (state_c == W_REG_ADDR) || (state_c == R_DEVICE_ADDR)) && 
                 rd_flag && (bit_cnt == bit_cnt_num - 1)) begin
            ack_flag <= sda_in;
        end
    end
endmodule
