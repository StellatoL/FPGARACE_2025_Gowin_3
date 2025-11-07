module DDS(
    input Clk,
    input Reset_n,          
    input [2:0] wave_select,
    input [31:0] frequency_set,

    input  wire [8:0] w_ad,
    input  wire [7:0] w_data,
    input  wire w_en,
    input [1:0]wave_set,
    output reg [7:0] Data
);

    reg [31:0] phase_accumulator;
    

    //累加
    always @(posedge Clk or negedge Reset_n) begin
        if (!Reset_n) begin
            phase_accumulator <= 32'd0;
        end else begin
            phase_accumulator <= phase_accumulator + frequency_set;
        end
    end

    wire [10:0] Rom_Addr = phase_accumulator[31:21];//提取高shier位
    wire  [8:0] Ram_Addr = phase_accumulator[24:16];
    wire [7:0] sine, square, triangular, sawtooth, wave_ram_out;

    reg [7:0] wave_data;

    always @(posedge Clk) begin
        case (wave_select)
            3'd0: wave_data = sine;
            3'd1: wave_data = square;
            3'd2: wave_data = triangular; 
            3'd3: wave_data = sawtooth;
            3'd4: wave_data = rule_data;
            3'd5: wave_data = wave_ram_out;
            3'd7: wave_data =  8'h0;
            default: wave_data = 8'd0;
        endcase
    end

    reg [7:0]rule_data;
    always @(posedge Clk) begin
        case (wave_set)
            3'd0: rule_data = sine;
            3'd1: rule_data = square;
            3'd2: rule_data = triangular; 
            3'd3: rule_data = sawtooth;
            default: rule_data = 8'd0;
        endcase
    end

    wave_ram wave_ram(
        .dout(wave_ram_out),
        .clka(Clk), 
        .cea(w_en), 
        .clkb(Clk),
        .ceb(1), 
        .oce(1),
        .reset(!Reset_n), 
        .ada(w_ad), 
        .din(w_data), 
        .adb(Ram_Addr) 
    );

    sine sine_rom (
        .dout(sine),
        .clk(Clk),
        .oce(1'b1),
        .ce(1'b1),
        .reset(!Reset_n),    
        .ad(Rom_Addr)
    );

    square square_rom (
        .dout(square),
        .clk(Clk),
        .oce(1'b1),
        .ce(1'b1),
        .reset(!Reset_n),
        .ad(Rom_Addr)
    );

    triangular triangular_rom (
        .dout(triangular),
        .clk(Clk),
        .oce(1'b1),
        .ce(1'b1),
        .reset(!Reset_n),
        .ad(Rom_Addr)
    );

    sawtooth sawtooth_rom (
        .dout(sawtooth),
        .clk(Clk),
        .oce(1'b1),
        .ce(1'b1),
        .reset(!Reset_n),
        .ad(Rom_Addr)
    );

    always @(posedge Clk) begin
        Data <= wave_data;
    end

endmodule