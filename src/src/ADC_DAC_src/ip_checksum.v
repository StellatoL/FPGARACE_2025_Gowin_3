/////////////////////////////////////////////////////////////////////////////////
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2019/05/01 00:00:00
// Module Name   : ip_checksum
// Description   : ip头部校验模块
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module ip_checksum(
  input           clk            ,
  input           reset_p        ,

  input           cal_en         ,

  input   [3:0]   IP_ver         ,
  input   [3:0]   IP_hdr_len     ,
  input   [7:0]   IP_tos         ,
  input   [15:0]  IP_total_len   ,
  input   [15:0]  IP_id          ,
  input           IP_rsv         ,
  input           IP_df          ,
  input           IP_mf          ,
  input   [12:0]  IP_frag_offset ,
  input   [7:0]   IP_ttl         ,
  input   [7:0]   IP_protocol    ,
  input   [31:0]  src_ip         ,
  input   [31:0]  dst_ip         ,

  output  [15:0]  checksum       
);

  reg  [31:0]suma;
  wire [16:0]sumb;
  wire [15:0]sumc;

  always@(posedge clk or posedge reset_p)
  if(reset_p)
    suma <= 32'd0;
  else if(cal_en)
    suma <= {IP_ver,IP_hdr_len,IP_tos}+IP_total_len+IP_id+
           {IP_rsv,IP_df,IP_mf,IP_frag_offset}+{IP_ttl,IP_protocol}+
           src_ip[31:16]+src_ip[15:0]+dst_ip[31:16]+dst_ip[15:0];
  else
    suma <= suma;

  assign sumb = suma[31:16]+suma[15:0];
  assign sumc = sumb[16]+sumb[15:0];

  assign checksum = ~sumc;

endmodule
