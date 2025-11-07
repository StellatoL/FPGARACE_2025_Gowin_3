module uart_data_tx #(
    parameter CLK_FREQ = 50000000,
    parameter UART_BAUD = 115200,
	parameter MAX_DATA_WIDTH = 25*8,
	parameter MSB_FIRST = 1
)(
	input clk,
	input rst_n,
  
	input [MAX_DATA_WIDTH - 1 : 0]data,
	input send_en,
    input [7:0] tx_data_size,
	
	output uart_tx,
	output reg Tx_Done,
	output uart_state
);
	
	reg [MAX_DATA_WIDTH - 1 : 0]data_r;

	reg [7:0] data_byte;
	reg byte_send_en;
	wire byte_tx_done;
	
	uart_byte_tx
    #(
    .CLK_FREQ(CLK_FREQ),
    .UART_BAUD(UART_BAUD)
)
u_uart_byte_tx(
		.clk(clk),
		.rst_n(rst_n),
		.data_byte(data_byte),
		.send_en(byte_send_en),
		.uart_tx(uart_tx),
		.Tx_Done(byte_tx_done),
		.uart_state(uart_state)
	);
	
	reg [8:0]cnt;
	reg [1:0]state;
	
	localparam S0 = 0;	//等待发送请求
	localparam S1 = 1;	//发起单字节数据发送
	localparam S2 = 2;	//等待单字节数据发送完成
	localparam S3 = 3;	//检查所有数据是否发送完成
	
	always@(posedge clk or negedge rst_n)
	if(!rst_n)begin
		data_byte <= 0;
		byte_send_en <= 0;
		state <= S0;
		cnt <= 0;
	end
	else begin
		case(state)
			S0: 
				begin
					data_byte <= 0;
					cnt <= 0;
					Tx_Done <= 0;
					if(send_en)begin
						state <= S1;
						data_r <= data;
					end
					else begin
						state <= S0;
						data_r <= data_r;
					end
				end
			
			S1:
				begin
					byte_send_en <= 1;
					if(MSB_FIRST == 1)begin
						data_byte <= data_r[tx_data_size*8-1 -:8];
						data_r <= data_r << 8;
					end
					else begin
						data_byte <= data_r[7:0];
						data_r <= data_r >> 8;					
					end
					state <= S2;
				end
				
			S2:
				begin
					byte_send_en <= 0;
					if(byte_tx_done)begin
						state <= S3;
						cnt <= cnt + 9'd8;
					end
					else
						state <= S2;
				end
			
			S3:
				if(cnt >= tx_data_size * 8)begin
					state <= S0;
					cnt <= 0;
					Tx_Done <= 1;
				end
				else begin
					state <= S1;
					Tx_Done <= 0;
				end
			default:state <= S0;
		endcase	
	end

endmodule