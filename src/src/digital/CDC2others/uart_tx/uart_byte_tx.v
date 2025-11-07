module uart_byte_tx #(
	parameter CLK_FREQ = 50000000,
    parameter UART_BAUD = 115200
)(
	input clk,   		   //模块全局时钟输入，50M
	input rst_n,    	   //复位信号输入，低有效
	input [7:0]data_byte,  //待传输8bit数据
	input send_en,    	   //发送使能
	
	output reg uart_tx,    //串口输出信号
	output reg Tx_Done,    //1byte数据发送完成标志
	output reg uart_state  //发送数据状态
);
    localparam bps_DR = CLK_FREQ / UART_BAUD;
	localparam START_BIT = 1'b0;
	localparam STOP_BIT = 1'b1; 
	
	reg bps_clk;	     //波特率时钟	
	reg [15:0]div_cnt;      //分频计数器	
	reg [3:0]bps_cnt;      //波特率时钟计数器	
	reg [7:0]data_byte_reg;//data_byte寄存后数据
	
	always@(posedge clk or negedge rst_n)
	if(!rst_n)
		uart_state <= 1'b0;
	else if(send_en)
		uart_state <= 1'b1;
	else if(bps_cnt == 4'd11)
		uart_state <= 1'b0;
	else
		uart_state <= uart_state;
	
	always@(posedge clk or negedge rst_n)
	if(!rst_n)
		data_byte_reg <= 8'd0;
	else if(send_en)
		data_byte_reg <= data_byte;
	else
		data_byte_reg <= data_byte_reg;
	
	//counter
	always@(posedge clk or negedge rst_n)
	if(!rst_n)
		div_cnt <= 16'd0;
	else if(uart_state)begin
		if(div_cnt == bps_DR)
			div_cnt <= 16'd0;
		else
			div_cnt <= div_cnt + 1'b1;
	end
	else
		div_cnt <= 16'd0;
	
	// bps_clk gen
	always@(posedge clk or negedge rst_n)
	if(!rst_n)
		bps_clk <= 1'b0;
	else if(div_cnt == 16'd1)
		bps_clk <= 1'b1;
	else
		bps_clk <= 1'b0;
	
	//bps counter
	always@(posedge clk or negedge rst_n)
	if(!rst_n)	
		bps_cnt <= 4'd0;
	else if(bps_cnt == 4'd11)
		bps_cnt <= 4'd0;
	else if(bps_clk)
		bps_cnt <= bps_cnt + 1'b1;
	else
		bps_cnt <= bps_cnt;
		
	always@(posedge clk or negedge rst_n)
	if(!rst_n)
		Tx_Done <= 1'b0;
	else if(bps_cnt == 4'd11)
		Tx_Done <= 1'b1;
	else
		Tx_Done <= 1'b0;
		
	always@(posedge clk or negedge rst_n)
	if(!rst_n)
		uart_tx <= 1'b1;
	else begin
		case(bps_cnt)
			0:uart_tx <= 1'b1;
			1:uart_tx <= START_BIT;
			2:uart_tx <= data_byte_reg[0];
			3:uart_tx <= data_byte_reg[1];
			4:uart_tx <= data_byte_reg[2];
			5:uart_tx <= data_byte_reg[3];
			6:uart_tx <= data_byte_reg[4];
			7:uart_tx <= data_byte_reg[5];
			8:uart_tx <= data_byte_reg[6];
			9:uart_tx <= data_byte_reg[7];
			10:uart_tx <= STOP_BIT;
			default:uart_tx <= 1'b1;
		endcase
	end	

endmodule