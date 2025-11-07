module data_8to16(
  input         clk,          // 系统时钟
  input         rst_n,        // 异步复位（低电平有效）
  input         data_en,      // 输入数据有效信号
  input  [7:0]  data_in,      // 8位输入数据
  output [15:0] data_out,     // 16位输出数据
  output        data_out_valid // 输出数据有效信号
);

  // 内部寄存器定义
  reg [7:0]  data_reg;       // 数据暂存寄存器
  reg        data_ready;     // 高字节数据就绪标志
  reg [15:0] data_out_reg;   // 输出数据寄存器
  reg        out_valid_reg;  // 输出有效寄存器
  
  // 数据拼接状态机
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      data_reg      <= 8'h0;
      data_ready    <= 1'b0;
      data_out_reg  <= 16'h0;
      out_valid_reg <= 1'b0;
    end else begin
      // 默认输出无效
      out_valid_reg <= 1'b0;
      
      // 当输入数据有效时处理
      if (data_en) begin
        if (!data_ready) begin
          // 保存第一个字节（高字节）
          data_reg   <= data_in;
          data_ready <= 1'b1;
        end else begin
          // 当已有高字节时，拼接低字节并输出
          data_out_reg  <= {data_reg, data_in};
          out_valid_reg <= 1'b1;
          data_ready    <= 1'b0; // 重置就绪标志
        end
      end
    end
  end

  // 输出赋值
  assign data_out      = data_out_reg;
  assign data_out_valid = out_valid_reg;

endmodule