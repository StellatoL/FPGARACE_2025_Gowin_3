module state_ctrl(
	input clk,
	input reset,

	input start_sample,
	input [31:0]set_sample_num,

	input ddr3_init_done,
	input uart_tx_done,
	input rdfifo_empty,
	input [15:0]rdfifo_dout,
	input wrfifo_full,
    input adc_data_en,
    input ad_out_valid,	
	//wrfifo_clr向外打三拍输出，保证wrfifo的清零信号的生效节拍数
	output reg wrfifo_clr,
	output reg ad_sample_en,

	output reg wrfifo_wren,
	output reg rdfifo_clr,
	output reg rdfifo_rden,
	output reg uart_send_en,
	output reg[7:0]uart_tx_data
);
	//*//状态机的状态总位宽为[3:0]也可行
	reg [4:0]state;
	//统计ADC1030向DDR传送的数据，和set_sample_num位深相同
	reg [31:0]adc_sample_cnt;
	//统计向串口发送ADC数据的个数，和set_sample_num位深一致
	reg [31:0]send_data_cnt;
	//将start_sample_rm进行采样，锁定其只在IDLE状态进行工作有效，其余状态均无效
	reg start_sample_rm;
	/*下方信号为写fifo清零的状态计数和保持，当进入写fifo清零状态后，
	首先开始计数，先保证计数完成，再等待wrfifo_full（写端fifo满信号）的信号拉低，
	拉低后，表示可以往fifo里写入数据，此时进入下一个状态。在清空（复位）fifo的时候，
	fifo的full信号会变高，可以认为在复位fifo时是不允许对fifo进行写操作的，即使写也是不可靠的，
	等fifo的复位结束后，full信号会变低，就允许对fifo进行写操作。
	清写端fifo的控制信号是由计数器(在前3个计数值将清除控制信号拉高)产生3个时钟周期的高电平脉冲*/
	reg [4:0]wrfifo_clr_cnt;
/*下方信号为读fifo清零的状态计数和保持，当进入读fifo清零状态后，
首先开始计数，先保证计数完成，再等待rdfifo_empty（读端fifo的空信号）信号拉低，
拉低后，表示fifo里已经有被写入数据，此时进入下一个状态在清空(复位)fifo的时候，
fifo的empty信号会变高，可以认为在复位fifo时是不允许对fifo进行读操作的，即使读也是不可靠的，
等fifo的复位结束后，fifo被写入数据后，empty信号会变低，就允许对fifo进行读操作。
清读端fifo的控制信号是由计数器(在前3个计数值将清除控制信号拉高)产生3个时钟周期的高电平脉冲*/	
	reg [4:0]rdfifo_clr_cnt;
	reg [15:0]fiforddata_rm;

	localparam IDLE                   = 4'd0;
	localparam DDR_WR_FIFO_CLEAR      = 4'd1;
	localparam ADC_SAMPLE             = 4'd2;
	localparam DDR_RD_FIFO_CLEAR      = 4'd3;
	localparam DATA_SEND_DELAY1       = 4'd4;
	localparam DATA_SEND_DELAY2       = 4'd5;
	localparam DATA_SEND_LOW_START    = 4'd6;
	localparam DATA_SEND_LOW_WORKING  = 4'd7;
	localparam DATA_SEND_HIGH_START   = 4'd8;
	localparam DATA_SEND_HIGH_WORKING = 4'd9;
	localparam DATA_SWITCH            = 4'd10;

	always@(posedge clk or posedge reset)
	if(reset)begin
		state<=IDLE;
		rdfifo_rden <= 1'b0;
		uart_send_en <= 1'b0;
		uart_tx_data<=8'd0;
	end
	else 
    case(state)
		IDLE: //0
            begin
            if(start_sample_rm)begin
                state<=DDR_WR_FIFO_CLEAR;  
                end
            else
                state<=state;
            end

		DDR_WR_FIFO_CLEAR: //1
            begin
            if(!wrfifo_full && (wrfifo_clr_cnt==9))
                state<=ADC_SAMPLE;
            else
                state<=DDR_WR_FIFO_CLEAR;
            end

		ADC_SAMPLE: //2
            begin
            if((adc_sample_cnt>=set_sample_num-1'b1 )&& adc_data_en)
                state<=DDR_RD_FIFO_CLEAR;
            else
                state<=state;
            end

		DDR_RD_FIFO_CLEAR: //3
            begin
            if(!rdfifo_empty && (rdfifo_clr_cnt==9))
                state<=DATA_SEND_DELAY1;
            else
                state<=state;
            end

		DATA_SEND_DELAY1: //4
            begin
                state <= DATA_SEND_DELAY2;
                rdfifo_rden <= 1'b1;
            end

		DATA_SEND_DELAY2: //5
            begin
                state <= DATA_SEND_LOW_START;
                rdfifo_rden <= 1'b0;
            end

		DATA_SEND_LOW_START: //6
            begin
                state <= DATA_SEND_LOW_WORKING;
                uart_send_en <= 1'b1;     //*//告诉串口：需要发送的数据已经到位，目前可以向外发送低8位数据了
                uart_tx_data <= fiforddata_rm[7:0];
                rdfifo_rden <= 1'b0;
            end

		DATA_SEND_LOW_WORKING: //7
            begin
            if(uart_tx_done)
                state <= DATA_SEND_HIGH_START;
            else
                state <= DATA_SEND_LOW_WORKING;
            end

		DATA_SEND_HIGH_START: //8
            begin
                state <= DATA_SEND_HIGH_WORKING;
                uart_tx_data <= fiforddata_rm[15:8];
            end

		DATA_SEND_HIGH_WORKING: //9
            begin
            if(uart_tx_done)//*//如果再次收到串口发送完成信号，则进入大循环和小循环的选择状态
                state <= DATA_SWITCH;
            else
                state <= DATA_SEND_HIGH_WORKING;
            end

		DATA_SWITCH: //10
            begin
                rdfifo_rden <= 1'b0;
                if(send_data_cnt>=set_sample_num)//*//如果send_data_cnt（串口发送计数）等于给定的值,则跳转进入IDLE状态//整个数据块发送完成，大循环收口
                    begin
                    state <= IDLE;
                    uart_send_en <= 1'b0;
                    end
                else 
                    state <= DATA_SEND_DELAY1;//*//一个双字节发送完成，进入DATA_SEND_DELAY1状态，小循环收口
            end

		default://默认为空闲状态，串口发送不使能，不读取fifo
            begin
                state <= IDLE;
                uart_send_en <= 1'b0;
                rdfifo_rden <= 1'b0;
            end
    endcase

	always@(posedge clk or posedge reset)begin  //对start_sample采样起始位进行寄存，同时限定其只工作在状态IDLE
	if(reset)
		start_sample_rm <= 1'b0;
	else if(state==IDLE && ddr3_init_done==1'b1)
		start_sample_rm <= start_sample;
	else 
		start_sample_rm <= 1'b0;
	end

	always@(posedge clk or posedge reset)begin//*//清除DDR写FIFO的计数器，10拍后指挥状态的跳转
	if(reset)
		wrfifo_clr_cnt<=0;
	else if(state==DDR_WR_FIFO_CLEAR)//如果进入了清fifo状态
	begin 
		if(wrfifo_clr_cnt==9)
			wrfifo_clr_cnt<=4'd9;
		else
			wrfifo_clr_cnt<=wrfifo_clr_cnt+1'b1;
	end
	else
		wrfifo_clr_cnt<=1'b0;
	end
	
//*//初始化成功后，进行一次清fifo
//*如果进入了DDR_WR_FIFO_CLEAR状态，则在wrfifo_clr_cnt为0,1或2时，清写fifo置1，否则wrfifo_clr为0*//
	always@(posedge clk or posedge reset)begin
	if (reset)
		wrfifo_clr<=0;
	else if(ddr3_init_done==1'b0)
		wrfifo_clr<=1'b1;
	else if(state==DDR_WR_FIFO_CLEAR)
        begin
            if(wrfifo_clr_cnt==0||wrfifo_clr_cnt==1||wrfifo_clr_cnt==2)
                wrfifo_clr<=1'b1;
            else
                wrfifo_clr<=1'b0;
        end
	else 
		wrfifo_clr<=1'b0;
	end

	always@(posedge clk or posedge reset)begin
    if(reset)
		wrfifo_wren<=0;
    else if(state==ADC_SAMPLE)
		wrfifo_wren<=1;
    else
		wrfifo_wren<=0;
	end
	
	always@(posedge clk or posedge reset)begin
    if(reset)
		ad_sample_en<=0;
	else if(state==ADC_SAMPLE)
		ad_sample_en<=1;
	else
		ad_sample_en<=0;
	end
	
//以下//如果adc_sample_cnt在ADC_SAMPLE状态，则每个时钟周期自加1
	always@(posedge clk or posedge reset)begin  
    if(reset)                                  
		adc_sample_cnt<=1'b0;
    else if(state==ADC_SAMPLE)begin
        if(adc_data_en)
		adc_sample_cnt<=adc_sample_cnt+1'b1;
		else
		adc_sample_cnt<=adc_sample_cnt;
	end
    else
		adc_sample_cnt<=1'b0;
	end

//以下//清除DDR读FIFO的计数器，10拍后指挥状态的跳转
	always@(posedge clk or posedge reset)begin
    if(reset)
		rdfifo_clr_cnt<=0;
    else if(state==DDR_RD_FIFO_CLEAR)//如果进入了清fifo状态
    begin 
		if(rdfifo_clr_cnt==9)
			rdfifo_clr_cnt<=4'd9;
		else
			rdfifo_clr_cnt<=rdfifo_clr_cnt+1'b1;
    end
    else
		rdfifo_clr_cnt<=1'b0;
	end

	always@(posedge clk or posedge reset)begin
    if (reset)
		rdfifo_clr<=0;
    else if(ddr3_init_done==1'b0)
		rdfifo_clr<=1'b1;
    else if(state==DDR_RD_FIFO_CLEAR)
    begin
		if(rdfifo_clr_cnt==0||rdfifo_clr_cnt==1||rdfifo_clr_cnt==2)
			rdfifo_clr<=1'b1;
     else
			rdfifo_clr<=1'b0;
    end
    else 
		rdfifo_clr<=1'b0;
	end

	always@(posedge clk or posedge reset)begin
    if(reset)
		fiforddata_rm<=16'b0;
    else if(rdfifo_rden)
		fiforddata_rm<=rdfifo_dout;
    else 
		fiforddata_rm<=fiforddata_rm;
	end
	
/*每个send_data_cnt在rdfifo_rden为1的状态下加1，
由于rdfifo_rden为高每次只持续一拍，
保证了每次读16bit数时send_data_cnt只持续加1*/
	always@(posedge clk or posedge reset)begin  
    if(reset)
		send_data_cnt<=1'b0;
    else if(state==IDLE)
		send_data_cnt<=1'b0;
    else if(rdfifo_rden)
		send_data_cnt<=send_data_cnt+1;
    else 
		send_data_cnt<=send_data_cnt;
	end

endmodule