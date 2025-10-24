module uart_byte_rx(
    input CLK,
    input RESET,

    input UART_RXD,
    output reg UART_RX_done,
    output reg [7:0] UART_RX_data
);

// 宏定义
parameter CLK_FREQ = 50000000;      // 系统时钟
parameter UART_BAUD = 115200;       // 波特率
localparam UART_BAUD_CNT_MAX = CLK_FREQ / UART_BAUD;    // 当前时钟下波特率对应的计数值

// 变量定义
reg         UART_RX_d0;
reg         UART_RX_d1;
reg         UART_RX_d2;
reg         UART_RX_FLAG;
reg  [3:0]  UART_RX_CNT;
reg  [15:0] UART_BAUD_CNT;
reg  [7:0]  UART_RX_data_t;

wire        start_en;

assign start_en = UART_RX_d2 & (~UART_RX_d1) & (~UART_RX_FLAG);

always @(posedge CLK or negedge RESET) begin
    if(!RESET) begin
        UART_RX_d0 <= 1'b0;
        UART_RX_d1 <= 1'b0;
        UART_RX_d2 <= 1'b0;
    end
    else begin
        UART_RX_d0 <= UART_RXD;
        UART_RX_d1 <= UART_RX_d0;
        UART_RX_d2 <= UART_RX_d1;
    end
end

always @(posedge CLK or negedge RESET) begin
    if(!RESET)
        UART_RX_FLAG <= 1'b0;
    else if(start_en)
        UART_RX_FLAG <=1'b1;
    else if((UART_RX_CNT == 4'd9) && (UART_BAUD_CNT == UART_BAUD_CNT_MAX / 2 - 1'b1))
        UART_RX_FLAG <= 1'b0;   //接收完成，接收标志位置零
    else
        UART_RX_FLAG <= UART_RX_FLAG;
end

always @(posedge CLK or negedge RESET) begin
    if(!RESET)
        UART_BAUD_CNT <= 16'd0;
    else if(UART_RX_FLAG) begin
        if(UART_BAUD_CNT < UART_BAUD_CNT_MAX - 1)
            UART_BAUD_CNT <= UART_BAUD_CNT + 16'd1;
        else
            UART_BAUD_CNT <= 16'd0;
    end
    else
        UART_BAUD_CNT <= 16'd0;
end

always @(posedge CLK or negedge RESET) begin
    if(!RESET)
        UART_RX_CNT <= 4'd0;
    else if(UART_RX_FLAG) begin
        if(UART_BAUD_CNT == UART_BAUD_CNT_MAX - 1'b1)
            UART_RX_CNT <= UART_RX_CNT + 1'b1;
        else
            UART_RX_CNT <= UART_RX_CNT;
    end
    else
        UART_RX_CNT <= 4'd0;
end

always @(posedge CLK or negedge RESET) begin
    if(!RESET)
        UART_RX_data_t <= 8'b0;
    else if(UART_RX_FLAG) begin
        if(UART_BAUD_CNT == UART_BAUD_CNT_MAX / 2 - 1'b1) begin
            case(UART_RX_CNT)
                4'd1 : UART_RX_data_t[0] <= UART_RX_d2;
                4'd2 : UART_RX_data_t[1] <= UART_RX_d2;
                4'd3 : UART_RX_data_t[2] <= UART_RX_d2;
                4'd4 : UART_RX_data_t[3] <= UART_RX_d2;
                4'd5 : UART_RX_data_t[4] <= UART_RX_d2;
                4'd6 : UART_RX_data_t[5] <= UART_RX_d2;
                4'd7 : UART_RX_data_t[6] <= UART_RX_d2;
                4'd8 : UART_RX_data_t[7] <= UART_RX_d2;
                default : ;
            endcase
        end
        else
            UART_RX_data_t <= UART_RX_data_t;
    end
    else
        UART_RX_data_t <= 8'b0;
end

always @(posedge CLK or negedge RESET) begin
    if(!RESET) begin
        UART_RX_done <= 1'b0;
        UART_RX_data <= 8'b0;
    end
    else if((UART_RX_CNT == 4'd9) && (UART_BAUD_CNT == UART_BAUD_CNT_MAX / 2 -1'b1))begin
        UART_RX_done <= 1'b1;
        UART_RX_data <= UART_RX_data_t;
    end
    else begin
        UART_RX_done <=1'b0;
        UART_RX_data <= UART_RX_data;
    end
end

endmodule
