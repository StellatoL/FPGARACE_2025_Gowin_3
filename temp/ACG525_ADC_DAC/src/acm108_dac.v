module ACM108_DAC(
    input clk_50M,
    input wire clk_125M,
    input wire pll_locked,

    input [2:0]uart_frequency,
    input [2:0]uart_wave,

    input wire [7:0] w_ad,
    input wire [7:0] w_data,
    input wire w_en,

    output wire [7:0]Data
);
    wire [7:0] DAC_Data;  //DDS产生数据
    reg [31:0]frequency_set; //由uart_frequency设置，传入DDS
    assign Data = DAC_Data;


    assign Reset_n = pll_locked;  //pll产生的复位信号


    //DDS例化
    DDS DDS(
        .Clk(clk_125M),
        .Reset_n(Reset_n),  
        .wave_select(uart_wave),
        .frequency_set(frequency_set),
        .Data(DAC_Data),
        .w_ad(w_ad),
        .w_data(w_data),
        .w_en(w_en)
    );

   //选择频率
    always@(posedge clk_125M)
        case(uart_frequency)   
            0:frequency_set = 34;  
            1:frequency_set = 344;
            2:frequency_set = 3436;
            3:frequency_set = 34360;
            4:frequency_set = 343597;
            5:frequency_set = 3435974;
            6:frequency_set = 34359738;
            7:frequency_set = 343597384;
        endcase        
endmodule 