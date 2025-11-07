module acg525_udp(
    input    wire   clk_50M        , //系统时钟输入，50MHz
    input    wire   reset_n       , //复位信号输入

    //ACM108
    input   wire [7:0]ADC_data,               
	output  wire  ADC_clk,
    output  wire [7:0]DAC_data,               
    output  wire  DAC_clk,

    // 串口调试
    output  uart_tx, 
    input   uart_rx,

    //以太网发送
    output        rgmii_tx_clk,
    output  [3:0] rgmii_txd,
    output        rgmii_txen,

    //ddr3硬件连接    
    output [13:0] O_ddr_addr        ,
    output [2:0] O_ddr_ba           ,
    output O_ddr_cs_n               ,
    output O_ddr_ras_n              ,
    output O_ddr_cas_n              ,
    output O_ddr_we_n               ,
    output O_ddr_clk                ,
    output O_ddr_clk_n              ,
    output O_ddr_cke                ,
    output O_ddr_odt                ,
    output O_ddr_reset_n            ,
    output [1:0] O_ddr_dqm          ,
    inout [15:0] IO_ddr_dq          ,
    inout [1:0] IO_ddr_dqs          ,
    inout [1:0] IO_ddr_dqs_n        ,

    // USB CDC相关端口
    inout [7:0] fx2_fdata,
    input       fx2_flagb,
    input       fx2_flagc,
    input       fx2_ifclk,
    output [1:0] fx2_faddr,
    output fx2_sloe,
    output fx2_slwr,
    output fx2_slrd,
    output fx2_pkt_end,
    output fx2_slcs,

    // 串口相关端口
    output uart_txd,
    input  uart_rxd,

    // I2C相关端口
    output scl,
    inout  sda,

    // SPI相关端口
    output wire spi_cs,
    output wire spi_clk,
    output wire spi_mosi,
    input  wire spi_miso,

    // PWM相关端口
    output [3:0] PWM,

    // CAN相关端口
    input  wire can_rxd,
    output wire can_txd,

    // 数字电路测量端口
    input extern_clk

);
    assign ADC_clk = clk_25M;
    assign DAC_clk = clk_125M;
    assign DAC_data=(510-DAC_data_r1)>>2;
    wire [8:0]DAC_data_r1;

    wire [2:0]uart_wave  ;//波形选择，0为sine，1为square，2为triangular，3为sawtooth， 5为自定义波形，7为输出电压恒为0
    wire [2:0]uart_frequency  ;//频率选择（0~7控制八个频率）
    wire [8:0]w_ad; //写入地址
    wire [7:0]w_data; //写入数据
    wire w_en=1;  //写入使能
    wire [31:0]DataNUM;

    parameter LOCAL_MAC  = 48'h00_0a_35_01_fe_c0;
    parameter DST_MAC = 48'hFF_FF_FF_FF_FF_FF;
    parameter LOCAL_IP   = 32'hc0_a8_00_02;
    parameter DST_IP = 32'hc0_a8_00_03;
    parameter LOCAL_PORT = 16'd5000;
    parameter DST_PORT = 16'd6102;
    parameter MAINDATA_NUM =  1472;

    parameter CLK_FREQ  = 48_000_000;

    parameter UART_BAUD = 115200;
    parameter CDC_MAX_DATA_SIZE = 32;

    parameter I2C_FREQ  = 100_000;
    parameter I2C_DATA_SIZE = 16;

    parameter SPI_FREQ  = 400_000;
    parameter SPI_MAX_BYTES = 16;
    
    //ddr3缓冲串口发送    
    wire [7:0]uart_adc_DATA;

    //ddr3配置    
    wire [27:0] app_addr_max = 256*1024*1024-1;//最大地址长度
    wire [7:0] burst_len = 8'd128;//突发长度 

    //ACM108
    wire [15:0]ad_out;

    //将adc数据与ddr3连接
    wire [15:0] wrfifo_din;

    //ddr3信号，主要与state_ctrl连接，控制ddr3的读写
    wire [15:0]rdfifo_dout;

    //FIFO_TX
    wire [11:0] Rnum;
    wire [7:0] dout;

    //udp
    wire[7:0] gmii_txd;

    //uart_rx
    wire [159:0] uart_rx_packet;
    reg [7:0]uart_tx_packet_reg[0:10];
    wire [3:0]rx_data_size;
    wire [31:0]Frequence;
    wire [7:0]my_order;
    wire [7:0]wave_time;
    Gowin_PLL your_instance_name(
         .clkin(clk_50M), //input  clkin
         .clkout0(clk_12_5M), //output  clkout0
         .clkout2(clk_400M), //output  clkout2
         .lock(lock_ddr3), //output  lock
         .mdclk(clk_50M), //input  mdclk
         .reset(~reset_n) //input  reset
    );

    ADC_DDR3_PLL ADC_DDR3_PLL(
         .clkin(clk_50M), //input  clkin
         .clkout0(clk_25M), //output  clkout0
         .clkout1(), //output  clkout1
         .lock(pll_lock), //output  lock
         .reset(~reset_n), //input  reset
         .mdclk(clk_50M) //input  mdclk
    );

    eth_pll eth_pll(
         .clkin(clk_50M), //input  clkin
         .clkout0(clk_125M), //output  clkout0
         .clkout1(clk_200M),
         .lock(lock), //output  lock
         .reset(~reset_n), //input  reset
         .mdclk(clk_50M) //input  mdclk
    );


 digital_top #(
     .CLK_FREQ(CLK_FREQ),
     .UART_BAUD(UART_BAUD),
     .CDC_MAX_DATA_SIZE(CDC_MAX_DATA_SIZE),
     .I2C_FREQ(I2C_FREQ),
     .I2C_DATA_SIZE(I2C_DATA_SIZE),
     .SPI_FREQ(SPI_FREQ),
     .SPI_MAX_BYTES(SPI_MAX_BYTES),
     .SPI_DATA_SIDE("MSB"),
     .CPOL(0),
     .CPHA(0)
 )digital_top_inst(
     .clk(clk_50M),
     .rst_n(reset_n),

     // usb cdc相关端口
     .fx2_fdata(fx2_fdata),
     .fx2_flagb(fx2_flagb),
     .fx2_flagc(fx2_flagc),
     .fx2_ifclk(fx2_ifclk),
     .fx2_faddr(fx2_faddr),
     .fx2_sloe(fx2_sloe),
     .fx2_slwr(fx2_slwr),
     .fx2_slrd(fx2_slrd),
     .fx2_pkt_end(fx2_pkt_end),
     .fx2_slcs(fx2_slcs),

     // 串口相关端口
     .uart_txd(uart_txd),
     .uart_rxd(uart_rxd),
     // I2C相关端口
     .scl(scl),
     .sda(sda),
     // SPI相关端口
     .spi_cs(spi_cs),
     .spi_clk(spi_clk),
     .spi_mosi(spi_mosi),
     .spi_miso(spi_miso),
     // CAN相关端口
     .can_rxd(can_rxd),
     .can_txd(can_txd),
     // PWM相关端口
     .PWM(PWM),
     // 数字测量端口
     .extern_clk(extern_clk)
 );

    ACM108_DAC ACM108_DAC(
         .clk_50M(clk_50M),
         .clk_125M(clk_125M),
         .pll_locked(lock),

         .uart_frequency(uart_frequency),
         .uart_wave(uart_wave),
         .Frequence(Frequence),
         .my_order(my_order),
         .wave_time(wave_time),

         .w_data(w_data),
         .w_ad(w_ad),
         .w_en(w_en),


         .Data(DAC_data_r1)
    );

    usart_send usart_send( 
        .tx_all_done(tx_all_done),//output
        .clk(clk_200M),          // 时钟引脚 (T9)
        .rst_n(reset_n),       // 复位引脚 (B17)
        .tx(uart_tx),               // 串口发送引脚 (V8)
        .ADC_data(uart_adc_DATA),
        .uart_en(uart_en)
    );


    uart_packet_rx #(
        .CLK_FREQ(200_000_000),
        .UART_BAUD(1000000),
        .MAX_DATA_SIZE(20)
    ) uart_packet_rx_inst (
        .clk(clk_200M),
        .rst_n(reset_n),
        .uart_rxd(uart_rx),
        .uart_rx_done(uart_rx_done),               
        .uart_rx_packet(uart_rx_packet),           
        .rx_data_size(rx_data_size)                
        );
    

    control control(
        .clk(clk_200M),
        .clk_b(clk_12_5M),
        .rst_n(reset_n),
        .uart_rx_done(uart_rx_done),
        .uart_rx_packet(uart_rx_packet),
        .DataNUM(DataNUM),
        .uart_wave(uart_wave),
        .uart_frequency(uart_frequency) ,
        .w_ad(w_ad),  
        .w_data(w_data),
        .w_en(),
        .Frequence(Frequence),
        .my_order(my_order),
        .wave_time(wave_time)
        );  

    data_8to16 data_8to16(
		.clk(clk_25M),
		.rst_n(reset_n),
		.data_en(adc_data_en),//input
		.data_in(ADC_data),  //intput
		.data_out(ad_out),  //output               
		.data_out_valid(ad_out_valid) //output
	);
    speed_ctrl speed_ctrl(
        .clk(clk_12_5M),
        .reset_n(reset_n),
        .ad_sample_en(ad_sample_en),
        .adc_data_en(adc_data_en),
        .div_set(0)
    );


    state_ctrl state_ctrl(
        .clk            (clk_12_5M    ),
        .reset          (!reset_n),
        
        .start_sample   (1),
        .set_sample_num (DataNUM),
        
        .ddr3_init_done (init_calib_complete),
        .uart_tx_done   (tx_all_done  ),
        .rdfifo_empty   (rdfifo_empty  ),
        .rdfifo_dout    (rdfifo_dout   ),
        .wrfifo_full    (wrfifo_full   ),
        .wrfifo_clr     (wr_load),
        .ad_sample_en   (ad_sample_en  ),
        .adc_data_en    (adc_data_en   ),
        .ad_out_valid   (),
        .wrfifo_wren    (),
        .rdfifo_clr     (rd_load    ),
        .rdfifo_rden    (rdfifo_rden   ),
        .uart_send_en   (uart_en  ),
        .uart_tx_data   (uart_adc_DATA  )
    );


    ddr3_ctrl_2port ddr3_ctrl_2port(
        .clk(clk_12_5M)                 ,//ADC采样时钟信号
        .pll_lock(lock_ddr3)            ,
        .clk_400m(clk_400M)            ,//DDR3参考时钟信号
        .sys_rst_n(reset_n)           ,//外部复位信号
        .init_calib_complete(init_calib_complete) ,    //DDR初始化完成信号

       // 用户接口
        .rd_load(rd_load)             ,     //输出源更新信号
        .wr_load(wr_load)             ,     //输入源更新信号
        .app_addr_rd_min(28'd0)     ,       //读DDR3的起始地址
        .app_addr_rd_max(app_addr_max)     ,//读DDR3的结束地址
        .rd_bust_len(burst_len)         ,   //从DDR3中读数据时的突发长度
        .app_addr_wr_min(28'd0)     ,       //写DD3的起始地址
        .app_addr_wr_max(app_addr_max)     ,//写DDR的结束地址
        .wr_bust_len(burst_len)         ,   //向DDR3中写数据时的突发长度

        .wr_clk(clk_12_5M)             ,       //wr_fifo的写时钟信号（ADC采样速率）
        .wfifo_wren(ad_out_valid&&adc_data_en)          , //wr_fifo的写使能信号
        .wfifo_din(ad_out)           ,  //写入到wr_fifo中的数据
        .wrfifo_full(wrfifo_full),
        .rd_clk(clk_12_5M)              ,      //rd_fifo的读时钟信号（ADC采样速率）
        .rfifo_rden(rdfifo_rden)          , //rd_fifo的读使能信号
        .rdfifo_empty(rdfifo_empty),
        .rfifo_dout(rdfifo_dout)          , //rd_fifo读出的数据信号 

       // DDR3硬件连接
        .ddr3_dq(IO_ddr_dq)             ,   //DDR3 数据
        .ddr3_dqs_n(IO_ddr_dqs_n)          ,//DDR3 dqs负
        .ddr3_dqs_p(IO_ddr_dqs)          ,  //DDR3 dqs正  
        .ddr3_addr(O_ddr_addr)           ,  //DDR3 地址   
        .ddr3_ba(O_ddr_ba)             ,    //DDR3 banck 选择
        .ddr3_ras_n(O_ddr_ras_n)          , //DDR3 行选择
        .ddr3_cas_n(O_ddr_cas_n)          , //DDR3 列选择
        .ddr3_we_n(O_ddr_we_n)           ,  //DDR3 读写选择
        .ddr3_reset_n(O_ddr_reset_n)       ,//DDR3 复位
        .ddr3_ck_p(O_ddr_clk)          ,    //DDR3 时钟正
        .ddr3_ck_n(O_ddr_clk_n)           , //DDR3 时钟负
        .ddr3_cke(O_ddr_cke)            ,   //DDR3 时钟使能
        .ddr3_cs_n(O_ddr_cs_n)        ,     //DDR3 片选
        .ddr3_dm(O_ddr_dqm)             ,   //DDR3_dm
        .ddr3_odt(O_ddr_odt)                //DDR3_odt   
    );



    fifo_send fifo_send(
		.Data(ADC_data),
		.Reset(~pll_lock), 
		.WrClk(clk_25M),
		.RdClk(clk_125M),
		.WrEn(1), 
		.RdEn(RdEn),
		.Wnum(Wnum), 
		.Rnum(Rnum), 
		.Q(dout), 
		.Empty(Empty),
		.Full(Full) 
	);

    udp_stream udp_stream(
        .clk_125M(clk_125M),
        .Rnum(Rnum),
        .udp_data_en(udp_data_en)
        );

    eth_udp_tx_gmii eth_udp_tx_gmii(
        .clk125m       (clk_125M),//input
        .reset_p       (~pll_lock),//input

        .tx_en_pulse   (udp_data_en),//input
        .tx_done       (tx_done),

        .dst_mac       (DST_MAC),
        .src_mac       (LOCAL_MAC), 
        .dst_ip        (DST_IP),
        .src_ip        (LOCAL_IP),
        .dst_port      (DST_PORT),
        .src_port      (LOCAL_PORT),

        .data_length   (MAINDATA_NUM), //UDP数据长度

        .payload_req_o (RdEn),//
        .payload_dat_i (dout),//数据填充

        .gmii_tx_clk   (gmii_tx_clk),
        .gmii_txen     (gmii_txen),
        .gmii_txd      (gmii_txd)
    );


     gmii_to_rgmii gmii_to_rgmii(
        .reset_n(pll_lock),

        .gmii_tx_clk(gmii_tx_clk),
        .gmii_txd(gmii_txd),
        .gmii_txen(gmii_txen),
        .gmii_txer(1'b0),

        .rgmii_tx_clk(rgmii_tx_clk),
        .rgmii_txd(rgmii_txd),
        .rgmii_txen(rgmii_txen)
    );





endmodule

