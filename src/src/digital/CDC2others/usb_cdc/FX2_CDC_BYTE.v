`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Simple FX2 FIFO loopback top module
// This version removes the 7-seg display and hc595 driver and provides
// a minimal loopback between FX2 SlaveFIFO and an internal FIFO for testing.
//
// Usage: use this file as your top-level for simple data loopback tests.
//////////////////////////////////////////////////////////////////////////////////
module FX2_CDC_BYTE (
  input       reset_n,

  // FX2 物理接口
  inout [7:0] fx2_fdata,      // FX2 SlaveFIFO data bus
  input       fx2_flagb,      // endpoint2 empty flag (read available)
  input       fx2_flagc,      // endpoint6 full flag (write available)
  input       fx2_ifclk,      // FX2 interface clock
  output [1:0] fx2_faddr,
  output fx2_sloe,
  output fx2_slwr,
  output fx2_slrd,
  output fx2_pkt_end,
  output fx2_slcs,

  // 单字节收发口
  input        tx_send_en,    // pulse to send tx_byte
  input  [7:0] tx_byte,
  output reg   rx_valid,      // pulse when a byte received from FX2
  output reg [7:0] rx_byte,
  output        tx_ready      // core can accept another tx byte

  
);

  // Simple loopback states
  localparam [1:0] LOOP_IDLE = 2'd0;
  localparam [1:0] LOOP_READ = 2'd1;
  localparam [1:0] LOOP_WRITE = 2'd2;

  reg [1:0] state;
  reg [1:0] next_state;

  reg slrd_n;
  reg slwr_n;
  reg sloe_n;
  reg [1:0] faddr_n;

  // rx path: data coming from FX2 -> rx_fifo -> output rx_byte/rx_valid
  // rx path: data coming directly from FX2 -> output rx_byte/rx_valid
  // (removed internal rx FIFO because it was never popped and could fill)

  // tx path: data to send -> tx_fifo -> FX2
  reg  [7:0] tx_fifo_din;
  wire [7:0] tx_fifo_dout;
  wire       tx_fifo_full;
  wire       tx_fifo_empty;
  wire       tx_fifo_push;
  wire       tx_fifo_pop;

  // FX2 control outputs
  assign fx2_slwr  = slwr_n;
  assign fx2_slrd  = slrd_n;
  assign fx2_sloe  = sloe_n;
  assign fx2_faddr = faddr_n;
  assign fx2_slcs  = 1'b0; // always enabled

  // Drive FX2 data bus when writing to FX2, otherwise high-Z
  reg [7:0] data_out;
  assign fx2_fdata = (slwr_n == 1'b0) ? data_out : 8'hzz;

  // Choose address: when reading from FX2 use addr 0, otherwise addr 2 for write
  always @(*) begin
    if (state == LOOP_READ) faddr_n = 2'b00;
    else faddr_n = 2'b10;
  end

  // Read control: assert when in READ state and FX2 indicates data available and rx_fifo not full
  always @(*) begin
    if ((state == LOOP_READ) && (fx2_flagb == 1'b1)) begin
      slrd_n = 1'b0;
      sloe_n = 1'b0;
    end else begin
      slrd_n = 1'b1;
      sloe_n = 1'b1;
    end
  end

  // Latch data from fx2 bus when reading: sample on clock edge when slrd asserted

  // Write control: assert when in WRITE state and FX2 can accept data and tx_fifo not empty
  always @(*) begin
    if ((state == LOOP_WRITE) && (fx2_flagc == 1'b1) && (tx_fifo_empty == 1'b0))
      slwr_n = 1'b0;
    else
      slwr_n = 1'b1;
  end

  assign tx_fifo_pop = ((slwr_n == 1'b0) & (tx_fifo_empty == 1'b0));

  // Output data to FX2 when writing
  always @(*) begin
    if (slwr_n == 1'b1) data_out = 8'dz;
    else data_out = tx_fifo_dout;
  end

  // Simple FSM for loopback (clocked by fx2_ifclk)
  always @(posedge fx2_ifclk or negedge reset_n) begin
    if (!reset_n) state <= LOOP_IDLE;
    else state <= next_state;
  end

  always @(*) begin
    next_state = state;
    case (state)
      LOOP_IDLE: begin
        // If data available from FX2 and rx FIFO has space, read it.
        // Otherwise if there is tx data pending and FX2 can accept, write it.
        if (fx2_flagb == 1'b1) begin
          next_state = LOOP_READ;
        end else if ((tx_fifo_empty == 1'b0) && (fx2_flagc == 1'b1)) begin
          next_state = LOOP_WRITE;
        end else begin
          next_state = LOOP_IDLE;
        end
      end
      LOOP_READ: begin
        // continue reading as long as FX2 has data and rx_fifo not full
        if (fx2_flagb == 1'b1) begin
          next_state = LOOP_READ;
        end else begin
          // finish read stage, give write priority if tx data pending
          if ((tx_fifo_empty == 1'b0) && (fx2_flagc == 1'b1)) next_state = LOOP_WRITE;
          else next_state = LOOP_IDLE;
        end
      end
      LOOP_WRITE: begin
        // write until TX FIFO empty or FX2 can't accept
        if ((fx2_flagc == 1'b1) && (~tx_fifo_empty)) next_state = LOOP_WRITE;
        else next_state = LOOP_IDLE;
      end
      default: next_state = LOOP_IDLE;
    endcase
  end

  // Simple packet end signal: assert 1 when not in idle (keeps FX2 happy)
  assign fx2_pkt_end = (state == LOOP_IDLE) ? 1'b0 : 1'b1;

  // Hook external TX input into tx_fifo push (assumes tx_send_en is synchronous to fx2_ifclk)
  assign tx_fifo_push = tx_send_en & (~tx_fifo_full);
  always @(*) tx_fifo_din = tx_byte;

  // When we read from FX2, also pulse rx_valid and present rx_byte
  // rx_valid is asserted for one fx2_ifclk cycle when a byte is read
  always @(posedge fx2_ifclk or negedge reset_n) begin
    if (!reset_n) begin
      rx_valid <= 1'b0;
      rx_byte <= 8'h00;
    end else begin
      rx_valid <= (slrd_n == 1'b0); // one-cycle pulse when read asserted
      if (slrd_n == 1'b0) rx_byte <= fx2_fdata;
    end
  end

  // rx FIFO removed: direct read from FX2 bus is used and presented via rx_valid/rx_byte

  // Instantiate tx_fifo (data to be sent to FX2)
  fifo_1024x8 tx_fifo_inst (
      .fifo_clk  (fx2_ifclk),
      .rst_n     (reset_n),
      .din       (tx_fifo_din),
      .write_busy(tx_fifo_push),
      .read_busy (tx_fifo_pop),
      .fifo_flush(1'b0),
      .dout      (tx_fifo_dout),
      .fifo_empty(tx_fifo_empty),
      .fifo_full (tx_fifo_full)
  );

  // expose tx_ready (core can accept a byte) -- active when tx fifo not full
  assign tx_ready = ~tx_fifo_full;

endmodule
