module uart_rx_packet_trans
#(
   parameter MAX_RX_DATA_SIZE = 25
)
(
   input clk,
   input rst_n,

   input [MAX_RX_DATA_SIZE * 8 - 1:0] uart_rx_packet,
   input [7:0] rx_data_size,

   output reg [2:0] packet_type
);

// localparameter define
localparam UART_TYPE = 0;
localparam I2C_TYPE  = 1;
localparam SPI_TYPE  = 2;
localparam PWM_TYPE  = 3;
localparam CAN_TYPE  = 4;


//**************************判断待转数据类型*************************//
wire [7:0] packet_head;
assign packet_head = (uart_rx_packet >> (rx_data_size * 8 - 8)) & 8'h07;

always @(posedge clk or negedge rst_n) begin
   if(!rst_n)
       packet_type <= UART_TYPE;
   else begin
       case(packet_head)
           8'h00: packet_type <= UART_TYPE;
           8'h01: packet_type <= I2C_TYPE;
           8'h02: packet_type <= SPI_TYPE;
           8'h03: packet_type <= PWM_TYPE;
           8'h04: packet_type <= CAN_TYPE;
           default: packet_type <= UART_TYPE;
       endcase
   end
end

endmodule
