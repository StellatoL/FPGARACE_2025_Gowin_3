module speed_ctrl(
	clk,
	reset_n,
	ad_sample_en,
	adc_data_en,
	div_set
);

	input clk;
	input reset_n;
	
	input ad_sample_en;
	
	output reg adc_data_en;
	input [31:0]div_set;

	reg [31:0]div_cnt;
	
	always@(posedge clk or negedge reset_n)
	if(!reset_n)
		div_cnt <= 0;
	else if(ad_sample_en)begin
		if(div_cnt >= div_set)
			div_cnt <= 0;
		else
			div_cnt <= div_cnt + 1'd1;
	end
	else
		div_cnt <= 0;
	
	always@(posedge clk or negedge reset_n)
	if(!reset_n)
		adc_data_en <= 0;
	else if(div_cnt == div_set)
		adc_data_en <= 1;
	else
		adc_data_en <= 0;
		
endmodule

