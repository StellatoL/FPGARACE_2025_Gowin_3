module control (
    input clk,
    input clk_b,
    input rst_n,
    input uart_rx_done,
    input [159:0] uart_rx_packet,
    output [31:0] DataNUM,
    output [3:0]uart_wave,
    output [3:0]uart_frequency,
    output [7:0]w_data,
    output [8:0]w_ad,
    output  w_en,
    output [31:0]Frequence,
    output [7:0]my_order,
    output [7:0]wave_time
    
);


wire [159:0]uart_rx_packet_out;
	fifo_uart fifo_uart(
		.Data(uart_rx_packet), //input [159:0] Data
		.WrClk(clk), //input WrClk
		.RdClk(clk_b), //input RdClk
		.WrEn(uart_rx_done), //input WrEn
		.RdEn(1), //input RdEn
		.Q(uart_rx_packet_out), //output [159:0] Q
		.Empty(Empty), //output Empty
		.Full(Full) //output Full
	);

reg uart_rx_done_en;
always@(posedge clk)begin
    if(uart_rx_done)begin
        uart_rx_done_en<=1;
    end
    else
       uart_rx_done_en<=0;
end


       assign DataNUM = uart_rx_packet_out[31:0];
       assign uart_wave = uart_rx_packet_out[35:32];
       assign uart_frequency = uart_rx_packet_out[39:36];
       assign w_data = uart_rx_packet_out[47:40];
       assign w_ad = uart_rx_packet_out[56:48];
       assign w_en = uart_rx_packet_out[57];
       assign Frequence =  uart_rx_packet_out[95:64];
       assign my_order = uart_rx_packet_out[103:96];
       assign wave_time =uart_rx_packet_out[111:104];


//always@(posedge clk or negedge rst_n) begin
//        if(!rst_n) begin
//            DataNUM <= 32'd0;
//            uart_wave <= 4'd0;
//            uart_frequency <= 4'd0;
//            w_ad <= 8'd0;
//            w_data <= 8'd0;
//            w_en <= 1'b0;
//        end
//        else if(uart_rx_done) begin
//            DataNUM <= uart_rx_packet[31:0];
//            uart_wave <= uart_rx_packet[35:32];
//            uart_frequency <= uart_rx_packet[39:36];
//            w_ad <= uart_rx_packet[47:40];
//            w_data <= uart_rx_packet[55:48];
//            w_en <= uart_rx_packet[56];
//        end
//        else begin
//            DataNUM <= DataNUM;
//            uart_wave <= uart_wave;
//            uart_frequency <= uart_frequency;
//            w_ad <= w_ad;
//            w_data <= w_data;
//            w_en <= w_en;
//        end
//    end

endmodule