module order_wave (
    input  clk,
    input  rst_n,
    input  [7:0]my_order,
    input  [7:0]wave_time,
    output reg[1:0] order
);

   

    reg [26:0] cnt;
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            cnt <= 27'd0;
        else if (cnt < 125000000-1)
            cnt <= cnt + 1'b1;
        else
            cnt <= 27'd0;
    end

    reg [1:0]change_order;
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            change_order <= 2'b0;
        else if (cnt == 125000000-1)
            change_order <= change_order+1'b1;
        else
        change_order <= change_order;
    end

    always @ (posedge clk)begin
        case (change_order)
            2'b00: order <= my_order[1:0];
            2'b01: order <= my_order[3:2];
            2'b10: order <= my_order[5:4];
            2'b11: order <= my_order[7:6];
            default: order <= 0;
        endcase
    end




endmodule