`timescale 1ns / 1ps

// 参数说明：
// width = 8         - 数据位宽
// depth = 1024      - FIFO深度
// depth_log2 = 10   - 地址指针位宽
// is_async_fifo = 0 - 同步FIFO（单时钟）
// is_sync_read = 1  - 同步读取（时钟上升沿输出）
// is_read_without_pop = 0 - 不支持无弹出读取（读取即弹出）
// is_multi_pop = 0  - 不支持单次多数据弹出
// is_write_commit = 0 - 直接写入（无写缓冲）
// is_flush = 1      - 支持刷新功能

module fifo_1024x8 (
    input        	 fifo_clk,   // FIFO主时钟（读写共用）
    input        	 rst_n,      // 异步复位（低电平有效）
    input 	   [7:0] din,        // 输入数据（8位宽）
    input        	 write_busy, // 写使能信号（高电平有效）
    input        	 read_busy,  // 读使能信号（高电平有效）
    input        	 fifo_flush, // FIFO刷新信号（清空FIFO）
    output reg [7:0] dout,     	 // 输出数据寄存器
    output reg       fifo_empty, // FIFO空标志
    output reg       fifo_full   // FIFO满标志
);

// 内部信号声明
reg  [10:0] write_occupancy;      // 写侧数据计数器（带额外位防溢出）
reg  [10:0] read_occupancy;       // 读侧数据计数器（带额外位防溢出）
reg  [10:0] next_write_occupancy; // 写侧计数器下一状态
reg  [10:0] next_read_occupancy;  // 读侧计数器下一状态
wire [ 9:0] next_write_ptr;       // 下一写指针
reg  [ 9:0] write_ptr;            // 当前写指针（10位，寻址1024）
wire [ 9:0] next_read_ptr;        // 下一读指针
reg  [ 9:0] read_ptr;             // 当前读指针（10位，寻址1024）
reg  [ 7:0] data_array [0:1023];  // 双端口RAM（1024 x 8位）
wire [ 9:0] read_index;           // RAM读地址选择器
wire [ 7:0] dout_next;            // 下一输出数据（组合逻辑）
reg         write_busy_d;         // 写使能延迟一拍（用于计数器同步）

//=========================================================================
// 写指针控制逻辑
//=========================================================================
always @(posedge fifo_clk or negedge rst_n) begin
    if (!rst_n) begin 
        write_ptr <= 10'b0;  // 复位时写指针归零
    end else begin 
        write_ptr <= next_write_ptr;  // 正常更新写指针
    end 
end

// 写指针更新逻辑：
// 刷新时复位指针，否则在写使能且非满时递增
assign next_write_ptr = fifo_flush ? 10'b0 : 
                       (write_ptr + (write_busy & ~fifo_full));

//=========================================================================
// 读指针控制逻辑
//=========================================================================
always @(posedge fifo_clk or negedge rst_n) begin
    if (!rst_n) begin 
        read_ptr <= 10'b0;  // 复位时读指针归零
    end else begin  
        read_ptr <= next_read_ptr;  // 正常更新读指针
    end 
end

// 读指针更新逻辑：
// 刷新时复位指针，否则在读使能且非空时递增
assign next_read_ptr = fifo_flush ? 10'b0 : 
                      (read_ptr + (read_busy & ~fifo_empty));

//=========================================================================
// RAM读写控制
//=========================================================================
// 读地址选择器：读使能时用下一读指针，否则用当前指针
assign read_index = read_busy ? next_read_ptr : read_ptr;

// 同步写操作：写使能有效时将数据写入RAM
always @(posedge fifo_clk) begin
    if (write_busy) begin
        data_array[write_ptr] <= din;
    end
end

// 同步读操作：输出数据寄存器打拍
always @(posedge fifo_clk) begin
    dout <= dout_next;  // 寄存器输出保证时序
end

// RAM读数据组合逻辑（实际会综合成同步RAM读取）
assign dout_next = data_array[read_index];

//=========================================================================
// 写使能延迟寄存器（用于计数器状态机）
//=========================================================================
always @(posedge fifo_clk or negedge rst_n) begin
    if(!rst_n) begin 
        write_busy_d <= 1'b0;
    end else begin  
        write_busy_d <= write_busy;  // 延迟一拍写使能信号
    end 
end

//=========================================================================
// 数据计数器状态机
//=========================================================================
// 读侧计数器更新逻辑
always @(*) begin
    if (fifo_flush) begin 
        next_read_occupancy = 10'b0;  // 刷新时计数器清零
    end else begin 
        // 状态机：根据读写操作更新计数器
        case ({write_busy_d, read_busy})
            2'b00: next_read_occupancy = read_occupancy;  // 无操作
            2'b01: next_read_occupancy = read_occupancy - 1'b1; // 仅读（减1）
            2'b10: next_read_occupancy = read_occupancy + 1'b1; // 仅写（加1）
            2'b11: next_read_occupancy = read_occupancy;        // 同时读写（不变）
            default: next_read_occupancy = read_occupancy;
        endcase    
    end 
end

// 写侧计数器更新逻辑（与读侧独立但逻辑相同）
always @(*) begin
    if (fifo_flush) begin 
        next_write_occupancy = 10'b0;  // 刷新时清零
    end else begin 
        case ({write_busy, read_busy})
            2'b00: next_write_occupancy = write_occupancy;  // 无操作
            2'b01: next_write_occupancy = write_occupancy - 1'b1; // 仅读
            2'b10: next_write_occupancy = write_occupancy + 1'b1; // 仅写
            2'b11: next_write_occupancy = write_occupancy;        // 同时读写
            default: next_write_occupancy = write_occupancy;
        endcase
    end 
end

// 计数器寄存器更新
always @(posedge fifo_clk or negedge rst_n) begin
    if (!rst_n) begin 
        write_occupancy <= 10'b0;
    end else begin 
        write_occupancy <= next_write_occupancy;  // 更新写侧计数器
    end 
end

always @(posedge fifo_clk or negedge rst_n) begin
    if (!rst_n) begin 
        read_occupancy <= 10'b0;
    end else begin 
        read_occupancy <= next_read_occupancy;    // 更新读侧计数器
    end 
end

//=========================================================================
// FIFO状态标志生成
//=========================================================================
// 满标志生成：当下一写计数值达到1024时置位
always @(posedge fifo_clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_full <= 0;  // 复位时非满
    end else if (next_write_occupancy == 1024) begin
        fifo_full <= 1;  // 下一周期将满
    end else begin 
        fifo_full <= 0;  // 非满状态
    end 
end

// 空标志生成：当下一读计数值为0时置位
always @(posedge fifo_clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_empty <= 1;  // 复位时为空
    end else if (next_read_occupancy == 0) begin
        fifo_empty <= 1;  // 下一周期将空
    end else begin
        fifo_empty <= 0;  // 非空状态
    end
end

endmodule