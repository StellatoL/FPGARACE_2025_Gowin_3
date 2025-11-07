module udp_stream(
    input           clk_125M,
    input  [15:0]   Rnum,
    output reg      udp_data_en

    );

reg pkt_tx_en;
always @(posedge clk_125M )begin
                if(Rnum >= (1472-2)) begin
                    udp_data_en <= 1'd1;
                end
                else begin
                    udp_data_en <= 1'd0;
                end
            end
 endmodule