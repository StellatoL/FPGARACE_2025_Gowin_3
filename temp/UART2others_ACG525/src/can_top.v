
//--------------------------------------------------------------------------------------------------------
// Module  : can_top
// Type    : synthesizable, IP's top
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: CAN bus controller,
//           CAN-TX: buffer input data and send them to CAN bus,
//           CAN-RX: get CAN bus data and output to user
//--------------------------------------------------------------------------------------------------------

module can_top #(
    // recieve ID filter parameters
    parameter [10:0] RX_ID_SHORT_FILTER = 11'h123,
    parameter [10:0] RX_ID_SHORT_MASK   = 11'h7ff,
    parameter [28:0] RX_ID_LONG_FILTER  = 29'h12345678,
    parameter [28:0] RX_ID_LONG_MASK    = 29'h1fffffff,
    
    // CAN timing parameters
    parameter [15:0] default_c_PTS  = 16'd34,
    parameter [15:0] default_c_PBS1 = 16'd5,
    parameter [15:0] default_c_PBS2 = 16'd10
) (
    input  wire        rstn,      // set to 1 while working
    input  wire        clk,       // system clock
    
    // CAN TX and RX, connect to external CAN phy (e.g., TJA1050)
    input  wire        can_rx,
    output wire        can_tx,
    
    // user tx-buffer write interface
    input  wire [10:0] LOCAL_ID,
    input  wire        tx_valid,  // when tx_valid=1 and tx_ready=1, push a data to tx fifo
    output wire        tx_ready,  // whether the tx fifo is available
    input  wire [63:0] tx_data,   // the data to push to tx fifo (64-bit for 8 bytes)
    
    // user rx packet interface (frame-at-once) - handshake style
    output reg         rx_valid,  // whether a whole packet is valid (held until rx_ready)
    output reg  [63:0] rx_data,   // the whole data payload of the packet
    output reg  [ 3:0] rx_len,    // number of valid bytes in rx_data
    output reg  [28:0] rx_id,     // the ID of a packet
    output reg         rx_ide,    // whether the ID is LONG or SHORT
    input wire         rx_ready   // handshake from user: set to 1 when user accepts packet
);



initial begin
    rx_valid = 1'b0;
    rx_data  = 64'h0;
    rx_len   = 4'd0;
    rx_id    = 29'd0;
    rx_ide   = 1'b0;
end

reg         buff_valid = 1'b0;
reg         buff_ready = 1'b0;
wire [63:0] buff_data;

reg         pkt_txing = 1'b0;
reg  [63:0] pkt_tx_data = 0;
wire        pkt_tx_done;
wire        pkt_tx_acked;
wire        pkt_rx_valid;
wire [28:0] pkt_rx_id;
wire        pkt_rx_ide;
wire        pkt_rx_rtr;
wire [ 3:0] pkt_rx_len;
wire [63:0] pkt_rx_data;
reg         pkt_rx_ack = 1'b0;

reg         t_rtr_req = 1'b0;
reg         r_rtr_req = 1'b0;
reg  [ 3:0] r_len = 4'd0;
reg  [63:0] r_data = 64'h0;
reg  [ 1:0] t_retry_cnt = 2'h0;



// ---------------------------------------------------------------------------------------------------------------------------------------
//  TX buffer
// ---------------------------------------------------------------------------------------------------------------------------------------
localparam DSIZE = 64;
localparam ASIZE = 10;

reg [DSIZE-1:0] buffer [0:((1<<ASIZE)-1)];  // may automatically synthesize to BRAM

reg [ASIZE:0] wptr=0, rptr=0;

wire full  = wptr == {~rptr[ASIZE], rptr[ASIZE-1:0]};
wire empty = wptr == rptr;

assign tx_ready = ~full;

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        wptr <= 0;
    end else begin
        if(tx_valid & ~full)
            wptr <= wptr + {{ASIZE{1'b0}}, 1'b1};
    end

always @ (posedge clk)
    if(tx_valid & ~full)
        buffer[wptr[ASIZE-1:0]] <= tx_data;

wire            rdready = ~buff_valid | buff_ready;
reg             rdack = 1'b0;
reg [DSIZE-1:0] rddata;
reg [DSIZE-1:0] keepdata = 0;
assign buff_data = rdack ? rddata : keepdata;

always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        buff_valid <= 1'b0;
        rdack <= 1'b0;
        rptr <= 0;
        keepdata <= 0;
    end else begin
        buff_valid <= ~empty | ~rdready;
        rdack <= ~empty & rdready;
        if(~empty & rdready)
            rptr <= rptr + {{ASIZE{1'b0}}, 1'b1};
        if(rdack)
            keepdata <= rddata;
    end

always @ (posedge clk)
    rddata <= buffer[rptr[ASIZE-1:0]];



// ---------------------------------------------------------------------------------------------------------------------------------------
//  CAN packet level controller
// ---------------------------------------------------------------------------------------------------------------------------------------
can_level_packet #(
    .default_c_PTS   ( default_c_PTS    ),
    .default_c_PBS1  ( default_c_PBS1   ),
    .default_c_PBS2  ( default_c_PBS2   )
) u_can_level_packet (
    .rstn            ( rstn             ),
    .clk             ( clk              ),
    
    .can_rx          ( can_rx           ),
    .can_tx          ( can_tx           ),
    
    .TX_ID           ( LOCAL_ID         ),
    .tx_start        ( pkt_txing        ),
    .tx_data         ( pkt_tx_data      ),
    .tx_done         ( pkt_tx_done      ),
    .tx_acked        ( pkt_tx_acked     ),
    
    .rx_valid        ( pkt_rx_valid     ),
    .rx_id           ( pkt_rx_id        ),
    .rx_ide          ( pkt_rx_ide       ),
    .rx_rtr          ( pkt_rx_rtr       ),
    .rx_len          ( pkt_rx_len       ),
    .rx_data         ( pkt_rx_data      ),
    .rx_ack          ( pkt_rx_ack       )
);



// ---------------------------------------------------------------------------------------------------------------------------------------
//  RX action (forward whole packet)
// ---------------------------------------------------------------------------------------------------------------------------------------
always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        pkt_rx_ack <= 1'b0;
        r_rtr_req <= 1'b0;
        r_len <= 4'd0;
        r_data <= 64'h0;
        rx_valid <= 1'b0;
        rx_data  <= 64'h0;
        rx_len   <= 4'd0;
        rx_id    <= 29'd0;
        rx_ide   <= 1'b0;
    end else begin
        // default
        pkt_rx_ack <= 1'b0;
        r_rtr_req <= 1'b0;

        // clear rx_valid when user accepts the packet
        if(rx_valid && rx_ready) begin
            rx_valid <= 1'b0;
        end

        // only accept/latch a new incoming packet when no pending packet is held by user
        if(pkt_rx_valid && ~rx_valid) begin
            // latch incoming packet
            r_len <= pkt_rx_len;
            r_data <= pkt_rx_data;
            rx_id <= pkt_rx_id;
            rx_ide <= pkt_rx_ide;

            if(pkt_rx_rtr) begin
                // remote frame handling: if short ID matches LOCAL_ID, ack and request RTR
                if(~pkt_rx_ide && pkt_rx_id[10:0]==LOCAL_ID) begin
                    pkt_rx_ack <= 1'b1;
                    r_rtr_req <= 1'b1;
                end
            end else if(~pkt_rx_ide) begin
                // short ID data packet
                if( (pkt_rx_id[10:0] & RX_ID_SHORT_MASK) == (RX_ID_SHORT_FILTER & RX_ID_SHORT_MASK) ) begin
                    pkt_rx_ack <= 1'b1;
                    rx_data <= pkt_rx_data;
                    rx_len  <= pkt_rx_len;
                    rx_valid <= 1'b1; // hold until rx_ready
                end
            end else begin
                // long ID data packet
                if( (pkt_rx_id & RX_ID_LONG_MASK) == (RX_ID_LONG_FILTER & RX_ID_LONG_MASK) ) begin
                    pkt_rx_ack <= 1'b1;
                    rx_data <= pkt_rx_data;
                    rx_len  <= pkt_rx_len;
                    rx_valid <= 1'b1; // hold until rx_ready
                end
            end
        end
    end



// ---------------------------------------------------------------------------------------------------------------------------------------
//  TX action
// ---------------------------------------------------------------------------------------------------------------------------------------
always @ (posedge clk or negedge rstn)
    if(~rstn) begin
        buff_ready <= 1'b0;
        pkt_tx_data <= 0;
        t_rtr_req <= 1'b0;
        pkt_txing <= 1'b0;
        t_retry_cnt <= 2'd0;
    end else begin
        buff_ready <= 1'b0;
        
        if(r_rtr_req)
            t_rtr_req <= 1'b1;                   // set t_rtr_req 
        
        if(~pkt_txing) begin
            t_retry_cnt <= 2'd0;
            if(t_rtr_req | buff_valid) begin     // if recieved a remote packet, or tx-buffer data available
                buff_ready <= buff_valid;        // tx-buffer buffer pop, if tx-buffer is available
                t_rtr_req <= 1'b0;               // reset t_rtr_req 
                if(buff_valid)                   // update data from tx-buffer , if tx-buffer is available
                    pkt_tx_data <= buff_data;
                pkt_txing <= 1'b1;
            end
        end else if(pkt_tx_done) begin
            if(pkt_tx_acked || t_retry_cnt==2'd3) begin
                pkt_txing <= 1'b0;
            end else begin
                t_retry_cnt <= t_retry_cnt + 2'd1;
            end
        end
    end


endmodule
