module FX2_CDC_TOP #(
    parameter MAX_DATA_SIZE = 32, // 最大数据长度 (以字节为单位)
    parameter MSB_FIRST = 1,     // 字节序：1 MSB first, 0 LSB first
    parameter RX_TIMEOUT = 10000  // 接收超时计数（周期数），无新字节则判定包结束
)(
    input       reset_n,

    // FX2 物理接口
    inout [7:0] fx2_fdata,       // FX2 SlaveFIFO data bus
    input       fx2_flagb,       // endpoint2 empty flag (read available)
    input       fx2_flagc,       // endpoint6 full flag (write available)
    input       fx2_ifclk,       // FX2 interface clock
    output [1:0] fx2_faddr,
    output fx2_sloe,
    output fx2_slwr,
    output fx2_slrd,
    output fx2_pkt_end,
    output fx2_slcs,

    // 不定字节收发口
    input       tx_send_en,                         // 启动发送（在 fx2_ifclk 域给出）
    input [MAX_DATA_SIZE*8-1:0] tx_data,            // 要发送的数据（高位或低位先取决于 MSB_FIRST）
    input [7:0] tx_data_size,                       // 有效字节数
    output reg  tx_done,                            // 发送完成脉冲（1 个周期）
    output reg  rx_valid,                           // 接收完成脉冲（包完成）
    output reg [MAX_DATA_SIZE*8-1:0] rx_data,      // 接收到的数据（按接收顺序打包）
    output reg [7:0] rx_data_size                   // 接收到的字节数
);

// wires to/from core
wire core_rx_valid;
wire [7:0] core_rx_byte;
wire core_tx_ready;

// internal control for sending single-byte to core
reg core_tx_pulse;
reg [7:0] core_tx_data;

// instantiate core
FX2_CDC_BYTE FX2_CDC_BYTE_inst(
    .reset_n(reset_n),
    .fx2_fdata(fx2_fdata),
    .fx2_flagb(fx2_flagb),
    .fx2_flagc(fx2_flagc),
    .fx2_ifclk(fx2_ifclk),
    .fx2_faddr(fx2_faddr),
    .fx2_sloe(fx2_sloe),
    .fx2_slwr(fx2_slwr),
    .fx2_slrd(fx2_slrd),
    .fx2_pkt_end(fx2_pkt_end),
    .fx2_slcs(fx2_slcs),

    .tx_send_en(core_tx_pulse),
    .tx_byte(core_tx_data),
    .rx_valid(core_rx_valid),
    .rx_byte(core_rx_byte),
    .tx_ready(core_tx_ready)
);

// --------------------- TX controller ---------------------
// Serialize tx_data into bytes and feed core when requested
reg [MAX_DATA_SIZE*8-1:0] tx_data_r;
reg [7:0] tx_bytes_remaining;
reg tx_active;
reg tx_send_en_d;

localparam TX_IDLE = 2'd0;

// tx FSM and byte steering (clocked by fx2_ifclk)
always @(posedge fx2_ifclk or negedge reset_n) begin
    if (!reset_n) begin
        tx_data_r <= {MAX_DATA_SIZE*8{1'b0}};
        tx_bytes_remaining <= 8'd0;
        tx_active <= 1'b0;
        core_tx_pulse <= 1'b0;
        core_tx_data <= 8'h00;
        tx_done <= 1'b0;
        tx_send_en_d <= 1'b0;
    end else begin
        tx_done <= 1'b0;
        core_tx_pulse <= 1'b0;
        tx_send_en_d <= tx_send_en;
        if (!tx_active) begin
            // start send on rising edge of tx_send_en
            if (tx_send_en && ~tx_send_en_d) begin
                // capture data and start
                tx_data_r <= tx_data;
                tx_bytes_remaining <= tx_data_size;
                tx_active <= 1'b1;
            end
        end else begin
            // active sending: if bytes remaining and core ready, send next byte
            if (tx_bytes_remaining != 0) begin
                if (core_tx_ready) begin
                    // choose byte depending on MSB_FIRST
                    if (MSB_FIRST) begin
                        core_tx_data <= tx_data_r[(tx_bytes_remaining*8-1) -: 8];
                        // no need to shift tx_data_r; just decrease counter
                    end else begin
                        core_tx_data <= tx_data_r[7:0];
                        tx_data_r <= tx_data_r >> 8;
                    end
                    core_tx_pulse <= 1'b1; // pulse to core to send byte
                    tx_bytes_remaining <= tx_bytes_remaining - 1'b1;
                end
            end else begin
                // finished
                tx_active <= 1'b0;
                tx_done <= 1'b1;
            end
        end
    end
end

// --------------------- RX controller ---------------------
// Collect incoming bytes into rx_data using frame format:
// Frame header: 0x20 0x25
// Frame tail:   0x0D 0x0A (\r\n)
// Behavior mirrors uart_packet_rx.v: only data between header and \r\n are stored.
reg [MAX_DATA_SIZE*8-1:0] rx_data_r;
reg [7:0] rx_count;
reg [2:0] rx_state;
reg [31:0] rx_timer;

localparam R_IDLE = 3'd0;
localparam R_SOF1 = 3'd1; // saw 0x20
localparam R_DATA = 3'd2; // collecting
localparam R_CR   = 3'd3; // saw 0x0D, expect 0x0A

always @(posedge fx2_ifclk or negedge reset_n) begin
    if (!reset_n) begin
        rx_data_r <= {MAX_DATA_SIZE*8{1'b0}};
        rx_count <= 8'd0;
        rx_data_size <= 8'd0;
        rx_valid <= 1'b0;
        rx_data <= {MAX_DATA_SIZE*8{1'b0}};
        rx_state <= R_IDLE;
    end else begin
        rx_valid <= 1'b0; // default low
        // update timer: reset on incoming byte, increment while inside a frame
        if (core_rx_valid) begin
            rx_timer <= 32'd0;
        end else if (rx_state != R_IDLE) begin
            rx_timer <= rx_timer + 1'b1;
        end else begin
            rx_timer <= 32'd0;
        end

        // if timer exceeds threshold, abort current frame and resync
        if (rx_timer >= RX_TIMEOUT && rx_state != R_IDLE) begin
            rx_state <= R_IDLE;
            rx_count <= 8'd0;
            rx_data_r <= {MAX_DATA_SIZE*8{1'b0}};
        end

        if (core_rx_valid) begin
            // process incoming byte according to state
            case (rx_state)
                R_IDLE: begin
                    if (core_rx_byte == 8'h20) begin
                        rx_state <= R_SOF1; // got first header byte
                    end else begin
                        rx_state <= R_IDLE;
                    end
                end
                R_SOF1: begin
                    if (core_rx_byte == 8'h25) begin
                        // header complete, move to data collection
                        rx_state <= R_DATA;
                        rx_count <= 8'd0;
                        rx_data_r <= {MAX_DATA_SIZE*8{1'b0}};
                    end else if (core_rx_byte == 8'h20) begin
                        // stay in SOF1 (possible repeated 0x20)
                        rx_state <= R_SOF1;
                    end else begin
                        // invalid, go back to idle
                        rx_state <= R_IDLE;
                    end
                end
                R_DATA: begin
                    if (core_rx_byte == 8'h0D) begin
                        rx_state <= R_CR; // possible end
                    end else begin
                        if (rx_count < MAX_DATA_SIZE) begin
                            rx_data_r <= (rx_data_r << 8) | core_rx_byte;
                            rx_count <= rx_count + 1'b1;
                        end
                        rx_state <= R_DATA;
                    end
                end
                R_CR: begin
                    if (core_rx_byte == 8'h0A) begin
                        // end of frame
                        rx_data <= rx_data_r;
                        rx_data_size <= rx_count;
                        rx_valid <= 1'b1;
                        // reset state
                        rx_state <= R_IDLE;
                        rx_count <= 8'd0;
                        rx_data_r <= {MAX_DATA_SIZE*8{1'b0}};
                    end else begin
                        // false alarm: previous 0x0D is real data, save it
                        if (rx_count < MAX_DATA_SIZE) begin
                            rx_data_r <= (rx_data_r << 8) | 8'h0D;
                            rx_count <= rx_count + 1'b1;
                        end
                        // now handle current byte as normal data
                        if (rx_count < MAX_DATA_SIZE) begin
                            rx_data_r <= (rx_data_r << 8) | core_rx_byte;
                            rx_count <= rx_count + 1'b1;
                        end
                        rx_state <= R_DATA;
                    end
                end
                default: rx_state <= R_IDLE;
            endcase
        end
    end
end

endmodule
