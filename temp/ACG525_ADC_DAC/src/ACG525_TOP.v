module ACG525_TOP(
    input wire clk_50M,
    input wire reset,

    input wire [7:0]ADC_data,
    output wire [7:0]DAC_data,

    output wire ADC_clk,
    output wire DAC_clk,

    output wire uart_tx,
    output wire uart_led
);




wire clk_200M;
wire clk_125M;
wire clk_35M;
wire lock;
assign ADC_clk =clk_35M;
assign DAC_clk =clk_125M;
Gowin_PLL Gowin_PLL(
        .clkin(clk_50M), 
        .clkout0(clk_200M), 
        .clkout1(clk_125M), 
        .clkout2(clk_35M), 
        .lock(lock), 
        .mdclk(clk_50M),
        .reset(!reset) 
);




assign ADC_data=0'hFF-ADC_data_0;
ACM108_ADC ACM108_ADC(

    .clk_50M(clk_50M),
    .clk_35M(clk_35M),
    .clk_200M(clk_200M),

    .reset(reset),
    .ADC_data(ADC_data_0),
    .uart_tx(uart_tx),
    .uart_led(uart_led)
);




wire [2:0]uart_wave =2;//波形选择，0为sine，1为square，2为triangular，3为sawtooth， 5为自定义波形，7为输出电压恒为0
wire [2:0]uart_frequency=0;//频率选择（0~7控制八个频率）
wire [7:0]w_ad; //写入地址
wire [7:0]w_data; //写入数据
wire w_en=1;  //写入使能
ACM108_DAC ACM108_DAC(
    .clk_50M(clk_50M),
    .clk_125M(clk_125M),
    .pll_locked(lock),

    .uart_frequency(uart_frequency),
    .uart_wave(uart_wave),

    .w_ad(w_ad),
    .w_data(w_data),
    .w_en(w_en),

    .Data(DAC_data)
);


endmodule