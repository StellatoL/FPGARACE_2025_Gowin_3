/////////////////////////////////////////////////////////////////////////////////
// Company: 武汉芯路恒科技有限公司
// Engineer: Max
// Web: www.corecourse.cn
// 
// Create Date: 2020/07/20 00:00:00
// Design Name: eth_udp_tx_gmii
// Module Name: eth_udp_tx_gmii
// Project Name: eth_udp_tx_gmii
// Target Devices: XC7A35T-2FGG484I
// Tool Versions: Vivado 2018.3
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module eth_udp_tx_gmii(
  clk125m,
  reset_p,

  tx_en_pulse,
  tx_done,

  dst_mac,
  src_mac,
  dst_ip,
  src_ip,
  dst_port,
  src_port,

  data_length,
  
  payload_req_o,
  payload_dat_i,

  gmii_tx_clk,
  gmii_txen,
  gmii_txd
);

  input          clk125m;
  input          reset_p;

  input          tx_en_pulse;
  output reg     tx_done;

  input [47:0]   dst_mac;
  input [47:0]   src_mac;
  input [31:0]   dst_ip;
  input [31:0]   src_ip;
  input [15:0]   dst_port;
  input [15:0]   src_port;

  input [15:0]   data_length;
  
  output         payload_req_o;
  input     [7:0]payload_dat_i;

  output         gmii_tx_clk;
//  (*IOB = "TRUE"*) output  reg[7:0] gmii_txd;
//  (*IOB = "TRUE"*) output  reg      gmii_txen;
  output  reg[7:0] gmii_txd;
  output  reg      gmii_txen;

  parameter ETH_type       = 16'h0800,
            IP_ver         = 4'h4,
            IP_hdr_len     = 4'h5,
            IP_tos         = 8'h00,
            IP_id          = 16'h0000,
            IP_rsv         = 1'b0,
            IP_df          = 1'b0,
            IP_mf          = 1'b0,
            IP_frag_offset = 13'h0000,
            IP_ttl         = 8'h40,
            IP_protocol    = 8'h11;

  localparam
    IDLE          = 8'b00000001,
    TX_PREAMBLE   = 8'b00000010,
    TX_ETH_HEADER = 8'b00000100,
    TX_IP_HEADER  = 8'b00001000,
    TX_UDP_HEADER = 8'b00010000,
    TX_DATA       = 8'b00100000,
    TX_FILL_DATA  = 8'b01000000,
    TX_CRC        = 8'b10000000;

  wire [15:0] IP_total_len;
  wire [15:0] IP_check_sum;
  wire [15:0] udp_length;
  wire [15:0] udp_check_sum = 16'h0000;

  reg  [47:0] dst_mac_reg;
  reg  [47:0] src_mac_reg;
  reg  [15:0] dst_port_reg;
  reg  [15:0] src_port_reg;
  reg  [31:0] dst_ip_reg;
  reg  [31:0] src_ip_reg;
  reg  [15:0] data_length_reg; 
  reg  [15:0] IP_total_len_reg;
  reg  [15:0] udp_length_reg;

  reg  [3:0]  cnt_preamble;
  reg  [3:0]  cnt_eth_header;
  reg  [4:0]  cnt_ip_header;
  reg  [3:0]  cnt_udp_header;
  reg  [15:0] cnt_data;
  reg  [4:0]  cnt_fill_data;
  reg  [1:0]  cnt_crc;
  reg  [7:0]  tx_data;
  reg         tx_en;
  reg  [7:0]  curr_state;
  reg  [7:0]  next_state;

  wire        crc_state;
  reg         crc_state_dly1;
  reg         crc_state_dly2;
  reg  [1:0]  cnt_crc_dly1;
  reg  [1:0]  cnt_crc_dly2;
  reg         tx_en_dly1;
  reg  [7:0]  tx_data_dly1;
  reg         crc_init;
  reg         crc_en_temp;
  reg         crc_en;
  reg  [7:0]  crc_in;
  wire [31:0] crc_result;

  reg  [7:0]  gmii_txd_renewcrc;
  reg         gmii_txen_renewcrc; 

  assign gmii_tx_clk = clk125m;

  assign udp_length = data_length + 8'd8;   //udp_header: 8byte
  assign IP_total_len = udp_length + 8'd20; //ip_header: 20byte

  ip_checksum ip_checksum(
    .clk            (clk125m        ),
    .reset_p        (reset_p        ),

    .cal_en         (tx_en_pulse    ),

    .IP_ver         (IP_ver         ),
    .IP_hdr_len     (IP_hdr_len     ),
    .IP_tos         (IP_tos         ),
    .IP_total_len   (IP_total_len   ),
    .IP_id          (IP_id          ),
    .IP_rsv         (IP_rsv         ),
    .IP_df          (IP_df          ),
    .IP_mf          (IP_mf          ),
    .IP_frag_offset (IP_frag_offset ),
    .IP_ttl         (IP_ttl         ),
    .IP_protocol    (IP_protocol    ),
    .src_ip         (src_ip         ),
    .dst_ip         (dst_ip         ),

    .checksum       (IP_check_sum   )
  );
  
  //����̫������Դ/Ŀ�Ĳ����Ĵ�
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    dst_mac_reg  <= 48'd0;
    src_mac_reg  <= 48'd0;
    dst_ip_reg   <= 32'd0;
    src_ip_reg   <= 32'd0;
    dst_port_reg <= 16'd0;
    src_port_reg <= 16'd0;
  end
  else if(tx_en_pulse)
  begin
    dst_mac_reg  <= dst_mac;
    src_mac_reg  <= src_mac;
    dst_port_reg <= dst_port;
    src_port_reg <= src_port;
    dst_ip_reg   <= dst_ip;
    src_ip_reg   <= src_ip;
  end

  //����̫���������ݲ��ֳ��Ȳ����Ĵ�
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    data_length_reg  <= 16'h0;
    IP_total_len_reg <= 16'h0;
    udp_length_reg   <= 16'h0;
  end
  else if(tx_en_pulse)
  begin
    data_length_reg  <= data_length;
    IP_total_len_reg <= IP_total_len;
    udp_length_reg   <= udp_length; 
  end

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    curr_state <= IDLE;
  else
    curr_state <= next_state;

  always@(*)
  begin
    case(curr_state)
      IDLE:
        if(tx_en_pulse)
          next_state = TX_PREAMBLE;
        else
          next_state = IDLE;

      TX_PREAMBLE:
        if(cnt_preamble == 4'd7)
          next_state = TX_ETH_HEADER;
        else
          next_state = TX_PREAMBLE;

      TX_ETH_HEADER:
        if(cnt_eth_header == 4'd13)
          next_state = TX_IP_HEADER;
        else
          next_state = TX_ETH_HEADER;

      TX_IP_HEADER:
        if(cnt_ip_header == 5'd19)
          next_state = TX_UDP_HEADER;
        else
          next_state = TX_IP_HEADER;

      TX_UDP_HEADER:
        if(cnt_udp_header == 4'd7)
          next_state = TX_DATA;
        else
          next_state = TX_UDP_HEADER;

      TX_DATA:
        if(data_length_reg <5'd18 && cnt_data == data_length_reg - 1'b1)
          next_state = TX_FILL_DATA;
        else if(cnt_data == data_length_reg - 1'b1)
          next_state = TX_CRC;
        else
          next_state = TX_DATA;

      TX_FILL_DATA:
        if(cnt_fill_data == 5'd17 - data_length_reg)
          next_state = TX_CRC;
        else
          next_state = TX_FILL_DATA;
        
      TX_CRC:
        if(cnt_crc == 2'd3)
          next_state = IDLE;
        else
          next_state = TX_CRC;

      default:next_state = IDLE;

    endcase
  end

  //cnt_preamble
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_preamble <= 4'd0;
  else if(curr_state == TX_PREAMBLE)
    cnt_preamble <= cnt_preamble + 1'b1;
  else
    cnt_preamble <= 4'd0;

  //cnt_eth_header
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_eth_header <= 4'd0;
  else if(curr_state == TX_ETH_HEADER)
    cnt_eth_header <= cnt_eth_header + 1'b1;
  else
    cnt_eth_header <= 4'd0;

  //cnt_ip_header
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_ip_header <= 5'd0;
  else if(curr_state == TX_IP_HEADER)
    cnt_ip_header <= cnt_ip_header + 1'b1;
  else
    cnt_ip_header <= 5'd0;

  //cnt_udp_header
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_udp_header <= 4'd0;
  else if(curr_state == TX_UDP_HEADER)
    cnt_udp_header <= cnt_udp_header + 1'b1;
  else
    cnt_udp_header <= 4'd0;

  //cnt_data
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_data <= 16'd0;
  else if(curr_state == TX_DATA)
    cnt_data <= cnt_data + 1'b1;
  else
    cnt_data <= 16'd0;

  //cnt_fill_data
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_fill_data <= 5'd0;
  else if(curr_state == TX_FILL_DATA)
    cnt_fill_data <= cnt_fill_data + 1'b1;
  else
    cnt_fill_data <= 5'd0;

  //cnt_crc
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    cnt_crc <= 5'd0;
  else if(curr_state == TX_CRC)
    cnt_crc <= cnt_crc + 1'b1;
  else
    cnt_crc <= 5'd0;

  //tx_data,tx_en
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    tx_en   <= 1'b0;
    tx_data <= 8'h00;
  end
  else
  begin
    case(curr_state)
      IDLE:
      begin
        tx_en   <= 1'b0;
        tx_data <= 8'h00;
      end

      TX_PREAMBLE:
      begin
        tx_en <= 1'b1;
        if(cnt_preamble <= 4'd6)
          tx_data <= 8'h55;
        else
          tx_data <= 8'hd5;
      end

      TX_ETH_HEADER:
      begin
        tx_en <= 1'b1;
        case(cnt_eth_header)
          4'd0:   tx_data <= dst_mac_reg[47:40];
          4'd1:   tx_data <= dst_mac_reg[39:32];
          4'd2:   tx_data <= dst_mac_reg[31:24];
          4'd3:   tx_data <= dst_mac_reg[23:16];
          4'd4:   tx_data <= dst_mac_reg[15:8];
          4'd5:   tx_data <= dst_mac_reg[7:0];
          4'd6:   tx_data <= src_mac_reg[47:40];
          4'd7:   tx_data <= src_mac_reg[39:32];
          4'd8:   tx_data <= src_mac_reg[31:24];
          4'd9:   tx_data <= src_mac_reg[23:16];
          4'd10:  tx_data <= src_mac_reg[15:8];
          4'd11:  tx_data <= src_mac_reg[7:0];
          4'd12:  tx_data <= ETH_type[15:8];
          4'd13:  tx_data <= ETH_type[7:0];
          default:tx_data <= 8'h00;
        endcase
      end

      TX_IP_HEADER:
      begin
        tx_en <= 1'b1;
        case(cnt_ip_header)
          5'd0:   tx_data <= {IP_ver,IP_hdr_len};
          5'd1:   tx_data <= IP_tos;
          5'd2:   tx_data <= IP_total_len_reg[15:8];
          5'd3:   tx_data <= IP_total_len_reg[7:0];
          5'd4:   tx_data <= IP_id[15:8];
          5'd5:   tx_data <= IP_id[7:0];
          5'd6:   tx_data <= {IP_rsv,IP_df,IP_mf,IP_frag_offset[12:8]};
          5'd7:   tx_data <= IP_frag_offset[7:0];
          5'd8:   tx_data <= IP_ttl;
          5'd9:   tx_data <= IP_protocol;
          5'd10:  tx_data <= IP_check_sum[15:8];
          5'd11:  tx_data <= IP_check_sum[7:0];
          5'd12:  tx_data <= src_ip_reg[31:24];
          5'd13:  tx_data <= src_ip_reg[23:16];
          5'd14:  tx_data <= src_ip_reg[15:8];
          5'd15:  tx_data <= src_ip_reg[7:0];
          5'd16:  tx_data <= dst_ip_reg[31:24];
          5'd17:  tx_data <= dst_ip_reg[23:16];
          5'd18:  tx_data <= dst_ip_reg[15:8];
          5'd19:  tx_data <= dst_ip_reg[7:0];
          default:tx_data <= 8'h00; 
        endcase
      end

      TX_UDP_HEADER:
      begin
        tx_en <= 1'b1;
        case(cnt_udp_header)
          4'd0:   tx_data <= src_port_reg[15:8];
          4'd1:   tx_data <= src_port_reg[7:0];
          4'd2:   tx_data <= dst_port_reg[15:8];
          4'd3:   tx_data <= dst_port_reg[7:0];
          4'd4:   tx_data <= udp_length_reg[15:8];
          4'd5:   tx_data <= udp_length_reg[7:0];
          4'd6:   tx_data <= udp_check_sum[15:8];
          4'd7:   tx_data <= udp_check_sum[7:0];
          default:tx_data <= 8'h00; 
        endcase
      end

      TX_DATA:
      begin
        tx_en <= 1'b1;
        tx_data <= payload_dat_i;
      end

      TX_FILL_DATA:
      begin
        tx_en <= 1'b1;
        tx_data <= 8'h00;
      end

      TX_CRC:
      begin
        tx_en <= 1'b1;
        tx_data <= 8'h00;
      end

      default:
      begin
        tx_en   <= 1'b0;
        tx_data <= 8'h00;
      end
    endcase
  end

  //payload_req_o
  assign payload_req_o = (curr_state == TX_DATA) ? 1'b1 : 1'b0;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    crc_init <= 1'b0;
  else if (tx_en_pulse && (curr_state == IDLE))
    crc_init <= 1'b1;
  else 
    crc_init <= 1'b0;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    crc_en_temp <= 1'b0;
  else if (curr_state == TX_ETH_HEADER || curr_state == TX_IP_HEADER || 
           curr_state == TX_UDP_HEADER || curr_state == TX_FILL_DATA || curr_state == TX_DATA)
    crc_en_temp <= 1'b1;
  else 
    crc_en_temp <= 1'b0;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    crc_en <= 1'b0;
  else if (crc_en_temp)
    crc_en <= 1'b1;
  else 
    crc_en <= 1'b0;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    crc_in <= 8'h00;
  else if(crc_en_temp)
    crc_in <= tx_data;
  else
    crc_in <= crc_in;

  crc32_d8 crc32_d8
  (
    .clk         (clk125m    ),
    .reset_p     (reset_p    ),

    .data        (crc_in     ),
    .crc_init    (crc_init   ),
    .crc_en      (crc_en     ),
    .crc_result  (crc_result )//latency=1
  );

  assign crc_state = curr_state == TX_CRC;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    tx_en_dly1   <= 1'b0;
    tx_data_dly1 <= 8'h00;
  end
  else
  begin
    tx_en_dly1   <= tx_en;
    tx_data_dly1 <= tx_data;
  end

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    crc_state_dly1 <= 1'b0;
    crc_state_dly2 <= 1'b0;
    cnt_crc_dly1   <= 2'd0;
    cnt_crc_dly2   <= 2'd0;
  end
  else
  begin
    crc_state_dly1 <= crc_state;
    crc_state_dly2 <= crc_state_dly1;
    cnt_crc_dly1   <= cnt_crc;
    cnt_crc_dly2   <= cnt_crc_dly1;
  end

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    gmii_txd_renewcrc <= 8'h00;
  else if(crc_state_dly2)
  begin
    case(cnt_crc_dly2)
      2'd0:gmii_txd_renewcrc <= crc_result[7:0];
      2'd1:gmii_txd_renewcrc <= crc_result[15:8];
      2'd2:gmii_txd_renewcrc <= crc_result[23:16];
      2'd3:gmii_txd_renewcrc <= crc_result[31:24];
    endcase
  end
  else
    gmii_txd_renewcrc <= tx_data_dly1;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    gmii_txen_renewcrc <= 1'b0;
  else if(tx_en_dly1)
    gmii_txen_renewcrc <= 1'b1;
  else
    gmii_txen_renewcrc <= 1'b0;

  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
  begin
    gmii_txen <= 1'b0;
    gmii_txd  <= 8'h00;
  end
  else
  begin
    gmii_txen <= gmii_txen_renewcrc;
    gmii_txd  <= gmii_txd_renewcrc;
  end

  //tx_done
  always@(posedge clk125m or posedge reset_p)
  if(reset_p)
    tx_done <= 1'b0;
  else if(curr_state == TX_CRC && cnt_crc == 2'd3)
    tx_done <= 1'b1;
  else
    tx_done <= 1'b0;

endmodule