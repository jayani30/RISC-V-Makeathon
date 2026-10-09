`timescale 1ns / 1ps
// dsin_lut: quarter-wave sine ROM + symmetry, registered output (1-cycle latency)
// NOTE: table is generated for PHASE_WIDTH=8, LUT_SIZE=256, AMPLITUDE=32767, OUTPUT_WIDTH>=16.
module dsin_lut #(
    parameter integer PHASE_WIDTH  = 8,
    parameter integer OUTPUT_WIDTH = 16,
    parameter integer LUT_SIZE     = 256,
    parameter integer AMPLITUDE    = 32767
)(
    input  wire                           clk,
    input  wire                           rst,
    input  wire [PHASE_WIDTH-1:0]         phase,
    input  wire                           enable,
    output reg signed [OUTPUT_WIDTH-1:0]  sin_out,
    output reg                            out_valid
);
    wire [1:0] quad = phase[7:6];                       // quadrant
    wire [5:0] idx  = phase[5:0];                       // position in quadrant
    // odd quadrants run the quarter table backwards
    wire [6:0] rom_addr = quad[0] ? (7'd64 - {1'b0, idx}) : {1'b0, idx};

    reg [14:0] rom_mag;                                 // 65 entries: sin 0..90 deg
    always @(*) begin
        case (rom_addr)
            7'd0 : rom_mag = 15'd0;
            7'd1 : rom_mag = 15'd804;
            7'd2 : rom_mag = 15'd1608;
            7'd3 : rom_mag = 15'd2410;
            7'd4 : rom_mag = 15'd3212;
            7'd5 : rom_mag = 15'd4011;
            7'd6 : rom_mag = 15'd4808;
            7'd7 : rom_mag = 15'd5602;
            7'd8 : rom_mag = 15'd6393;
            7'd9 : rom_mag = 15'd7179;
            7'd10: rom_mag = 15'd7962;
            7'd11: rom_mag = 15'd8739;
            7'd12: rom_mag = 15'd9512;
            7'd13: rom_mag = 15'd10278;
            7'd14: rom_mag = 15'd11039;
            7'd15: rom_mag = 15'd11793;
            7'd16: rom_mag = 15'd12539;
            7'd17: rom_mag = 15'd13279;
            7'd18: rom_mag = 15'd14010;
            7'd19: rom_mag = 15'd14732;
            7'd20: rom_mag = 15'd15446;
            7'd21: rom_mag = 15'd16151;
            7'd22: rom_mag = 15'd16846;
            7'd23: rom_mag = 15'd17530;
            7'd24: rom_mag = 15'd18204;
            7'd25: rom_mag = 15'd18868;
            7'd26: rom_mag = 15'd19519;
            7'd27: rom_mag = 15'd20159;
            7'd28: rom_mag = 15'd20787;
            7'd29: rom_mag = 15'd21403;
            7'd30: rom_mag = 15'd22005;
            7'd31: rom_mag = 15'd22594;
            7'd32: rom_mag = 15'd23170;
            7'd33: rom_mag = 15'd23731;
            7'd34: rom_mag = 15'd24279;
            7'd35: rom_mag = 15'd24811;
            7'd36: rom_mag = 15'd25329;
            7'd37: rom_mag = 15'd25832;
            7'd38: rom_mag = 15'd26319;
            7'd39: rom_mag = 15'd26790;
            7'd40: rom_mag = 15'd27245;
            7'd41: rom_mag = 15'd27683;
            7'd42: rom_mag = 15'd28105;
            7'd43: rom_mag = 15'd28510;
            7'd44: rom_mag = 15'd28898;
            7'd45: rom_mag = 15'd29268;
            7'd46: rom_mag = 15'd29621;
            7'd47: rom_mag = 15'd29956;
            7'd48: rom_mag = 15'd30273;
            7'd49: rom_mag = 15'd30571;
            7'd50: rom_mag = 15'd30852;
            7'd51: rom_mag = 15'd31113;
            7'd52: rom_mag = 15'd31356;
            7'd53: rom_mag = 15'd31580;
            7'd54: rom_mag = 15'd31785;
            7'd55: rom_mag = 15'd31971;
            7'd56: rom_mag = 15'd32137;
            7'd57: rom_mag = 15'd32285;
            7'd58: rom_mag = 15'd32412;
            7'd59: rom_mag = 15'd32521;
            7'd60: rom_mag = 15'd32609;
            7'd61: rom_mag = 15'd32678;
            7'd62: rom_mag = 15'd32728;
            7'd63: rom_mag = 15'd32757;
            7'd64: rom_mag = 15'd32767;
            default: rom_mag = 15'd0;
        endcase
    end

    wire signed [15:0] mag_s    = {1'b0, rom_mag};
    wire signed [15:0] lut_data = quad[1] ? -mag_s : mag_s;   // negative half cycle

    always @(posedge clk) begin
       if (rst) begin
            sin_out   <= {OUTPUT_WIDTH{1'b0}};
            out_valid <= 1'b0;
        end else if (enable) begin
            sin_out   <= lut_data;
            out_valid <= 1'b1;
        end else begin
            sin_out   <= {OUTPUT_WIDTH{1'b0}};
            out_valid <= 1'b0;
        end
    end

endmodule