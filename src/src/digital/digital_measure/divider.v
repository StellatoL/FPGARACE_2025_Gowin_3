`timescale 1ns / 1ps

module divider#(
  parameter WIDTH    = 32,
  parameter CACHING  =  0,
  parameter INIT_VLD =  0
)(
  input clk, 
  input rst, 
  input start, 
  input      [WIDTH-1:0] dividend, 
  input      [WIDTH-1:0] divisor, 
  output reg [WIDTH-1:0] quotient, 
  output     [WIDTH-1:0] remainder, 
  output reg zeroErr, 
  output valid
);
  localparam IDLE = 0,
             PREP = 1,
             CALC = 2;
  reg [1:0] state;


  //Operands
  reg [WIDTH-1:0] dividendOp; 
  reg [WIDTH  :0] divisorOp; 


  //Shift counter
  localparam C_WIDTH = $clog2(WIDTH)+1;
  reg [C_WIDTH-1:0] shiftCounter;

  always@(posedge clk) begin
    if(state == IDLE) begin
      shiftCounter <= 0;
    end case(state)
      PREP: shiftCounter <= shiftCounter+1;
      CALC: shiftCounter <= shiftCounter-1;
    endcase
  end


  //Flags
  wire zeroDivide = divisor == 0;
  wire useCache; //needed for caching
  wire startCalculation = !zeroDivide && start && !(CACHING && useCache);

  wire subtract  = {1'b0,dividendOp} >= divisorOp && state == CALC;
  wire stopShift = dividendOp < (divisorOp<<1);
  
  generate //Reset state valid or valid only enabled after receiving a calculation
    if(INIT_VLD) begin
      assign valid = state == IDLE && !start;
    end else begin
      reg valid_en;
      assign valid = valid_en && !start && state == IDLE;
      always@(posedge clk) begin
        if(rst) begin
          valid_en <= 0;
        end begin
          valid_en <= valid_en || start;
        end
      end
    end
  endgenerate
  
  always@(posedge clk) begin
    if(rst) begin
      zeroErr <= 0;
    end else begin
      zeroErr <= start && state == IDLE ? zeroDivide : zeroErr;
    end
  end


  //Operand Logic
  always@(posedge clk) begin
    if(startCalculation) begin
      divisorOp <= divisor;
    end else case(state)
      PREP: divisorOp <= divisorOp<<1;
      CALC: divisorOp <= divisorOp>>1;
    endcase
  end
  always@(posedge clk) begin
    if(startCalculation) begin
      dividendOp <= dividend;
    end else begin
      dividendOp <= subtract ? dividendOp-divisorOp : dividendOp; 
    end
  end
  always@(posedge clk) begin
    if(state == PREP) begin
      quotient <= 0;
    end else begin
      quotient <= state == CALC ? (quotient<<1) | subtract : quotient;
    end
  end
  assign remainder = dividendOp;

  //State Transactions
  always@(posedge clk) begin
    if(rst) begin
      state <= IDLE;
    end else case(state)
      IDLE: state <=  startCalculation ? PREP : state;
      PREP: state <=         stopShift ? CALC : state;
      CALC: state <= shiftCounter == 0 ? IDLE : state;
    endcase
  end


  //Optional Logic
  generate //Caching functionality
    if(CACHING) begin
      reg [WIDTH-1:0] dividend_cached, divisor_cached;
      assign useCache = (dividend == dividend_cached) && (divisor == divisor_cached);
      always@(posedge clk) begin
        if (rst) begin
          dividend_cached <= 0;
          divisor_cached  <= 0;
        end else begin
          dividend_cached <= startCalculation && state == IDLE ? dividend : dividend_cached;
          divisor_cached  <= startCalculation && state == IDLE ? divisor  :  divisor_cached;
        end
      end
    end
  endgenerate
endmodule
