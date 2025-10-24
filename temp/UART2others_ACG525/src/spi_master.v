//==========================================
//SPI线制定义 |三线制 或者 四线制
//==========================================
   `define four_wire  1 //四线制SPI申明
   // `define three_wire 1 //三线制SPI申明
   `ifdef four_wire
         //==========================================
         //四线SPI类型定义 |全双工类型 或者 半双工类型
         //==========================================
            // `define half_duplex 1 //半双工类型
            `define full_duplex 1 //全双工类型
    `endif
//==========================================
`resetall
`timescale 1ns/1ps 
module spi_modu #(
   parameter   CLK_FREQ  = 'd50_000_000,// 模块时钟频率 |单位:hz
   parameter   SPI_FREQ  = 'd100_000,   // SPI通信频率 |单位:hz
   parameter   DATA_SIDE = "MSB" ,  // MSB:大端模式 LSB:小端模式
   parameter   BIT_NUM   = 'd8   ,  // 数据长度 |单位bit
   parameter   CPOL      = 'd0   ,  // 时钟极性
   parameter   CPHA      = 'd0   ,  // 时钟相位
   localparam  W1 = BIT_NUM-1    
)(
   input  wire          clk         ,  // input  wire [0        :0] |模块时钟输入
   input  wire          rst_n       ,  // input  wire [0        :0] |模块复位输入 |0:无效 1:复位
`ifdef four_wire
 `ifdef full_duplex
   input  wire          swop_en     ,  // input  wire [0        :0] |开始交换使能
   input  wire [W1:0]   send_data   ,  // input  wire [BIT_NUM-1:0] |发送数据输入
   output reg           recv_vld    ,  // output reg  [0        :0] |接收数据有效标志
   output reg  [W1:0]   recv_data   ,  // output reg  [BIT_NUM-1:0] |接收数据输出
   output reg           swop_done   ,  // output reg  [0        :0] |交换结束标志
   output reg           comm_done   ,  // output reg  [0        :0] |通信结束标志

   output reg           spi_cs_n    ,  // output reg  [0        :0] |设备片选
   output reg           spi_sclk    ,  // output reg  [0        :0] |通信时钟
   output reg           spi_mosi    ,  // output reg  [0        :0] |数据输出
   input  wire          spi_miso       // input  wire [0        :0] |数据输入
 `elsif half_duplex
   input  wire          send_en     ,  // input  wire [0        :0] |发送数据使能
   input  wire [W1:0]   send_data   ,  // input  wire [BIT_NUM-1:0] |发送数据输入
   output reg           send_done   ,  // output reg  [0        :0] |发送数据结束标志
   input  wire          recv_en     ,  // input  wire [0        :0] |接收数据使能
   output reg           recv_vld    ,  // output reg  [0        :0] |接收数据有效标志
   output reg  [W1:0]   recv_data   ,  // output reg  [BIT_NUM-1:0] |接收数据输出
   output reg           recv_done   ,  // output reg  [0        :0] |接收数据结束标志
   output reg           comm_done   ,  // output reg  [0        :0] |通信结束标志

   output reg           spi_cs_n    ,  // output reg  [0        :0] |设备片选
   output reg           spi_sclk    ,  // output reg  [0        :0] |通信时钟
   output reg           spi_mosi    ,  // output reg  [0        :0] |数据输出
   input  wire          spi_miso       // input  wire [0        :0] |数据输入
 `endif
`elsif three_wire
   input  wire          send_en     ,  // input  wire [0        :0] |发送数据使能
   input  wire [W1:0]   send_data   ,  // input  wire [BIT_NUM-1:0] |发送数据输入
   output reg           send_done   ,  // output reg  [0        :0] |发送数据结束标志
   input  wire          recv_en     ,  // input  wire [0        :0] |接收数据使能
   output reg           recv_vld    ,  // output reg  [0        :0] |接收数据有效标志
   output reg  [W1:0]   recv_data   ,  // output reg  [BIT_NUM-1:0] |接收数据输出
   output reg           recv_done   ,  // output reg  [0        :0] |接收数据结束标志
   output reg           comm_done   ,  // output reg  [0        :0] |通信结束标志

   output reg           spi_cs_n    ,  // output reg  [0        :0] |设备片选
   output reg           spi_sclk    ,  // output reg  [0        :0] |通信时钟
   inout  wire          spi_sdio       // inout  wire [0        :0] |数据输入输出
`endif
);
//================================================
// 
//================================================
localparam MAX1 = ((CLK_FREQ/4)/(SPI_FREQ*2))-1;
localparam W2   = $clog2(MAX1+1)-1;    //时钟计数器最大值 |两倍SPI时钟
localparam W3   = $clog2(BIT_NUM)-1;   //数据位宽
localparam W4   = W1+1;
reg  [W2:0] cnt_a = 'd0; //时钟计数器（八倍SPI时钟）
reg  [1 :0] cnt_b = 'd0; //分段计数器（将半个SPI时钟周期分为四段）
reg  [W3:0] cnt_c = 'd0; //分段计数器（将一个SPI时钟周期分为八段）
reg  [3 :0] cnt_d = 'd0; //bit计数器
reg  [2 :0] add_r = 'd0; //输入数据累加寄存器
reg  [W4:0] t_data_r = 'd0; //输出数据
reg  [W4:0] t_data   = 'd0; //待输出数据
reg  [W1:0] r_data_r = 'd0; //接收数据
reg         again    = 'd0; //再一次标志寄存器
reg  [2 :0] NOW_S0   = 'd0;

`ifdef four_wire
 `ifdef full_duplex
   reg         en_r = 'd0;
   wire        en_up;
   always @(posedge clk) en_r<=swop_en;
   assign   en_up = {en_r,swop_en}==2'b01 ? 1'b1 : 1'b0;
   always @(posedge clk or negedge rst_n) begin 
      if(!rst_n) begin 
         recv_vld  <= 'd0; // output reg  [0        :0] |接收数据有效标志
         recv_data <= 'd0; // output reg  [BIT_NUM-1:0] |接收数据输出
         swop_done <= 'd0; // output reg  [0        :0] |交换结束标志
         comm_done <= 'd0; // output reg  [0        :0] |通信结束标志
         spi_cs_n  <= 'd1; // output reg  [0        :0] |设备片选
         spi_mosi  <= 'd0; // output reg  [0        :0] |数据输出
         if(CPOL) spi_sclk<='d1;
         else     spi_sclk<='d0;

         cnt_a    <= 'd0; 
         cnt_b    <= 'd0; 
         cnt_c    <= 'd0; 
         cnt_d    <= 'd0; 
         add_r    <= 'd0; 
         t_data_r <= 'd0;
         t_data   <= 'd0;
         r_data_r <= 'd0;
         again    <= 'd0;
         NOW_S0   <= 'd0;
      end 
      else begin 
         case(NOW_S0)
            'd0 : begin //空闲状态
               comm_done<='d0;
               spi_cs_n <='d1;
               if(CPOL) spi_sclk<='d1;
               else     spi_sclk<='d0;
               if(en_up) begin t_data_r<=send_data; 
                  if(DATA_SIDE=="MSB") t_data_r<={1'b0,send_data};
                  else                 t_data_r<={send_data,1'b0};
                  NOW_S0<='d1; 
               end 
            end 
            'd1 : begin //收发数据状态
               spi_cs_n<='d0;
               if(DATA_SIDE=="MSB") spi_mosi<=t_data_r[W4];
               else                 spi_mosi<=t_data_r[0];
               if(cnt_a==MAX1) begin 
                  cnt_a<='d0;
                  if(cnt_d=='d8&&cnt_c=='d1) begin 
                     if(again=='d1) begin cnt_b<='d2; cnt_c<='d2; cnt_d<='d1; t_data_r<=t_data; end 
                     else           begin cnt_b<='d0; cnt_c<='d0; cnt_d<='d0; NOW_S0<='d0; comm_done<='d1; end 
                  end else begin 
                     cnt_b<=cnt_b+1;
                     cnt_c<=cnt_c+1;
                     if(&cnt_b)spi_sclk <=~spi_sclk;
                     if(cnt_c=='d1) begin 
                        cnt_d<=cnt_d+1;
                        if(DATA_SIDE=="MSB") t_data_r<=t_data_r<<1;
                        else                 t_data_r<=t_data_r>>1;
                     end 
                  end 
                  if(cnt_d=='d8&&cnt_c=='d7) recv_data<=r_data_r;
                  if(DATA_SIDE=="MSB") begin 
                     if(cnt_c=='d6) r_data_r<={r_data_r[W1-1:0],add_r[2]};
                  end else begin 
                     if(cnt_c=='d6) r_data_r<={add_r[2],r_data_r[W1:1]};
                  end 
               end 
               else cnt_a<=cnt_a+1;
               if(cnt_c=='d6&&cnt_a<='d6) add_r<=add_r+spi_miso;
               else if(cnt_c=='d7)        add_r<='d0;
               if(cnt_a=='d0 &&cnt_d=='d8&&cnt_c=='d0) recv_vld <='d1;
               else                                    recv_vld <='d0;
               if(cnt_a==MAX1&&cnt_d=='d8&&cnt_c=='d0) swop_done<='d1;
               else                                    swop_done<='d0;
               if(cnt_d=='d8&&cnt_c<='d1&&en_up=='d1) begin 
                  if(DATA_SIDE=="MSB") t_data<={send_data,1'b0};
                  else                 t_data<={1'b0,send_data};
               end 
               if(cnt_d=='d8&&cnt_c<='d1&&en_up=='d1) again <= 'd1;
               else if(cnt_d=='d1&&cnt_c<='d1)        again <= 'd0;
            end 
         endcase 
      end 
   end 
 `elsif half_duplex
   reg  [1 :0] tr_reg = 'd3;
   reg         t_en = 'd0, r_en = 'd0;
   wire        t_up, r_up, run;
   always @(posedge clk) t_en<= #20 send_en;
   always @(posedge clk) r_en<= #20 recv_en;
   assign t_up = {t_en,send_en}==2'b01 ? 1'b1 : 1'b0;
   assign r_up = {r_en,recv_en}==2'b01 ? 1'b1 : 1'b0;
   assign run  = (t_up || r_up) ? 1'b1 : 1'b0;
   always @(posedge clk or negedge rst_n) begin 
      if(!rst_n) begin 
         send_done <= 'd0;// output reg  [0        :0] |发送数据结束标志
         recv_vld  <= 'd0;// output reg  [0        :0] |接收数据有效标志
         recv_data <= 'd0;// output reg  [BIT_NUM-1:0] |接收数据输出
         recv_done <= 'd0;// output reg  [0        :0] |接收数据结束标志
         comm_done <= 'd0;// output reg  [0        :0] |通信结束标志
         spi_cs_n  <= 'd0;// output reg  [0        :0] |设备片选
         spi_mosi  <= 'd0;// output reg  [0        :0] |数据输出
         if(CPOL) spi_sclk<='d1;
         else     spi_sclk<='d0;

         cnt_a    <= 'd0; 
         cnt_b    <= 'd0; 
         cnt_c    <= 'd0; 
         cnt_d    <= 'd0; 
         add_r    <= 'd0; 
         t_data_r <= 'd0;
         t_data   <= 'd0;
         r_data_r <= 'd0;
         again    <= 'd0;
         NOW_S0   <= 'd0;
      end 
      else begin 
         case(NOW_S0)
            'd0 : begin //空闲状态
               comm_done<='d0;
               spi_cs_n <='d1;
               if(run) NOW_S0<='d1;
               if     (t_up) tr_reg[0]<='d0;
               else if(r_up) tr_reg[0]<='d1;
               if(t_up) begin 
                  if(DATA_SIDE=="MSB") t_data_r<={1'b0,send_data};
                  else                 t_data_r<={send_data,1'b0};
               end 
            end 
            'd1 : begin //工作状态
               spi_cs_n <='d0;
               if(tr_reg[0]=='d0) begin 
                  if(DATA_SIDE=="MSB") spi_mosi<=t_data_r[W4];
                  else                 spi_mosi<=t_data_r[0];
               end 
               if(cnt_a==MAX1) begin 
                  cnt_a<='d0;
                  if(cnt_d=='d8&&cnt_c=='d1) begin 
                     if(again) begin cnt_b<='d2; cnt_c<='d2; cnt_d<='d1; t_data_r<=t_data; tr_reg<=tr_reg>>1; end 
                     else      begin cnt_b<='d0; cnt_c<='d0; cnt_d<='d0; NOW_S0<='d0; comm_done<='d1; end 
                  end else begin 
                     cnt_b<=cnt_b+1;
                     cnt_c<=cnt_c+1;
                     if(&cnt_b) spi_sclk<=~spi_sclk;
                     if(cnt_c=='d1) begin 
                        cnt_d<=cnt_d+1;
                        if(~tr_reg[0]) begin 
                           if(DATA_SIDE=="MSB") t_data_r<=t_data_r<<1;
                           else                 t_data_r<=t_data_r>>1;
                        end 
                     end 
                  end 
                  if(tr_reg[0]) begin 
                     if(DATA_SIDE=="MSB") begin 
                        if(cnt_c=='d6) r_data_r<={r_data_r[W1-1:0],add_r[2]};
                     end else begin 
                        if(cnt_c=='d6) r_data_r<={add_r[2],r_data_r[W1:1]};
                     end 
                  end 
                  if(tr_reg[0]&&cnt_d=='d8&&cnt_c=='d7) recv_data<=r_data_r;
               end else cnt_a<=cnt_a+1;

               if(cnt_a==MAX1&&cnt_d=='d8&&cnt_c=='d0) begin 
                  if(tr_reg[0])  recv_done<='d1;
                  else           send_done<='d1;
               end else begin recv_done<='d0; send_done<='d0; end 
               if(cnt_d=='d8&&cnt_c=='d1) begin 
                  if(t_up=='d1) begin 
                     tr_reg[1]<='d0;
                     if(DATA_SIDE=="MSB") t_data<={send_data,1'b0};
                     else                 t_data<={1'b0,send_data};
                  end 
                  if(r_up) tr_reg[1]<='d1;
               end 
               if(cnt_d=='d8&&cnt_c=='d1&&run=='d1) again<='d1;
               else if(cnt_d=='d1)                  again<='d0;
               if(tr_reg[0]) begin 
                  if(cnt_c=='d6&&cnt_a<='d6) add_r<=add_r+spi_miso;
                  else if(cnt_c=='d7)        add_r<='d0;
                  if(cnt_a=='d0 &&cnt_d=='d8&&cnt_c=='d0) recv_vld <='d1;
                  else                                    recv_vld <='d0;
               end 
            end 
         endcase 
      end 
   end 
 `endif 
`elsif three_wire
   reg         T_dio='d1, O_sdi='d0;
   wire        I_sdo;
   reg         t_en = 'd0, r_en = 'd0;
   wire        t_up, r_up, run;
   reg  [1 :0] tr_reg = 'd3;
   always @(posedge clk) t_en<=send_en;
   always @(posedge clk) r_en<=recv_en;
   assign t_up = {t_en,send_en}==2'b01 ? 1'b1 : 1'b0;
   assign r_up = {r_en,recv_en}==2'b01 ? 1'b1 : 1'b0;
   assign run  = {t_en,send_en}==2'b01 || {r_en,recv_en}==2'b01 ? 1'b1 : 1'b0;
   always @(posedge clk or posedge rst_n) begin 
      if(rst_n) begin 
         send_done <= 'd0; // output reg  [0        :0] |发送数据结束标志
         recv_vld  <= 'd0; // output reg  [0        :0] |接收数据有效标志
         recv_data <= 'd0; // output reg  [BIT_NUM-1:0] |接收数据输出
         recv_done <= 'd0; // output reg  [0        :0] |接收数据结束标志
         comm_done <= 'd0; // output reg  [0        :0] |通信结束标志
         spi_cs_n  <= 'd1; // output reg  [0        :0] |设备片选
         if(CPOL) spi_sclk<='d1;
         else     spi_sclk<='d0;

         T_dio    <= 'd1;
         O_sdi    <= 'd0;
         tr_reg   <= 'd3;
         cnt_a    <= 'd0;
         cnt_b    <= 'd0;
         cnt_c    <= 'd0;
         cnt_d    <= 'd0;
         add_r    <= 'd0;
         t_data_r <= 'd0;
         t_data   <= 'd0;
         r_data_r <= 'd0;
         again    <= 'd0;
         NOW_S0   <= 'd0;
      end 
      else begin 
         case(NOW_S0)
            'd0 : begin //空闲状态
               comm_done<='d0;
               spi_cs_n<='d1;
               if(run) NOW_S0<='d1;
               if(t_up) begin 
                  if(DATA_SIDE=="MSB") t_data_r<={1'b0,send_data};
                  else                 t_data_r<={send_data,1'b0};
               end 
               if     (t_up) tr_reg[0]<='d0;
               else if(r_up) tr_reg[0]<='d1;
            end 
            'd1 : begin //工作状态
               spi_cs_n<='d0;
               if(tr_reg[0]=='d1)   T_dio<='d1;
               else                 T_dio<='d0;
               if(DATA_SIDE=="MSB") O_sdi<=t_data_r[W4];
               else                 O_sdi<=t_data_r[0];
               if(cnt_a==MAX1) begin 
                  cnt_a<='d0;
                  if(cnt_d=='d8&&cnt_c=='d1) begin 
                     if(again) begin cnt_b<='d2; cnt_c<='d2; cnt_d<='d1; t_data_r<=t_data; tr_reg<=tr_reg>>1; end 
                     else      begin cnt_b<='d0; cnt_c<='d0; cnt_d<='d0; NOW_S0<='d0; T_dio<='d1; comm_done<='d1; end 
                  end else begin 
                     cnt_b<=cnt_b+1;
                     cnt_c<=cnt_c+1;
                     if(&cnt_b) spi_sclk<=~spi_sclk;
                     if(cnt_c=='d1) begin 
                        cnt_d<=cnt_d+1;
                        if(~tr_reg[0]) begin 
                           if(DATA_SIDE=="MSB") t_data_r<=t_data_r<<1;
                           else                 t_data_r<=t_data_r>>1;
                        end 
                     end 
                  end 
                  if(tr_reg[0]) begin 
                     if(DATA_SIDE=="MSB") begin 
                        if(cnt_c=='d6) r_data_r<={r_data_r[W1-1:0],add_r[2]};
                     end else begin 
                        if(cnt_c=='d6) r_data_r<={add_r[2],r_data_r[W1:1]};
                     end 
                  end 
                  if(tr_reg[0]&&cnt_d=='d8&&cnt_c=='d7) recv_data<=r_data_r;
               end else cnt_a<=cnt_a+1;

               if(cnt_a==MAX1&&cnt_d=='d8&&cnt_c=='d0) begin 
                  if(tr_reg[0])  recv_done<='d1;
                  else           send_done<='d1;
               end else begin recv_done<='d0; send_done<='d0; end 
               if(cnt_d=='d8&&cnt_c=='d1) begin 
                  if(t_up=='d1) begin 
                     tr_reg[1]<='d0;
                     if(DATA_SIDE=="MSB") t_data<={send_data,1'b0};
                     else                 t_data<={1'b0,send_data};
                  end 
                  if(r_up) tr_reg[1]<='d1;
               end 
               if(cnt_d=='d8&&cnt_c=='d1&&run=='d1) again<='d1;
               else if(cnt_d=='d1)                  again<='d0;
               if(tr_reg[0]) begin 
                  if(cnt_c=='d6&&cnt_a<='d6) add_r<=add_r+I_sdo;
                  else if(cnt_c=='d7)        add_r<='d0;
                  if(cnt_a=='d0 &&cnt_d=='d8&&cnt_c=='d0) recv_vld <='d1;
                  else                                    recv_vld <='d0;
               end 
            end 
         endcase 
      end 
   end 
   IOBUF #(
      .DRIVE         (12         ), // Specify the output drive strength
      .IBUF_LOW_PWR  ("TRUE"     ), // Low Power - "TRUE", High Performance = "FALSE" 
      .IOSTANDARD    ("DEFAULT"  ), // Specify the I/O standard
      .SLEW          ("SLOW"     )  // Specify the output slew rate
   ) IOBUF_sdio (
      .O (I_sdo   ), // Buffer output
      .IO(spi_sdio), // Buffer inout port (connect directly to top-level port)
      .I (O_sdi   ), // Buffer input
      .T (T_dio   )  // 3-state enable input, high=input, low=output
   );   
`endif 
endmodule 
