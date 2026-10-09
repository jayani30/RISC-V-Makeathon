`timescale 1ns / 1ps
//==============================================================================
// bpsk_sample_generator : turns a BPSK bit + sine sample into a signed 8-bit sample
//
//   bpsk_bit = 1 : bpsk_sample = +sine
//   bpsk_bit = 0 : bpsk_sample = -sine     (180-degree inverted)
//
// SINE_SIGNED = 0 (default): sine_sample is an UNSIGNED magnitude, 0..255.
//                            Values above 127 are clamped to 127.
// SINE_SIGNED = 1          : sine_sample is a two's-complement signed 8-bit sine
//                            (-128..+127), e.g. the top byte of a signed sine LUT.
//
// Signed conversion: sine_sample is widened to a SIGNED 9-bit value s9
//   unsigned mode: s9 = {2'b00, magnitude[6:0]}      ( 0 .. +127)
//   signed mode  : s9 = {sine[7], sine}              (-128 .. +127, sign extend)
// then v = bpsk_bit ? s9 : -s9 is computed in 9 bits (range -128..+128, so
// negating -128 cannot overflow), and the result is saturated to -127..+127
// before being truncated to 8 bits. The output range is therefore symmetric.
//
// Timing (one clock of latency, fully registered):
//   edge N   : enable, bpsk_bit, sine_sample are sampled
//   after N  : bpsk_sample = result, sample_valid = enable (1 if enable was 1)
//   enable=0 : sample_valid goes 0 after the edge, bpsk_sample HOLDS its value
//   reset    : active-low, synchronous; clears bpsk_sample and sample_valid
//==============================================================================
module bpsk_sample_generator #(
    parameter integer SINE_SIGNED = 0
)(
    input  wire                     clk,
    input  wire                     resetn,

    input  wire                     enable,
    input  wire                     bpsk_bit,
    input  wire signed [7:0]        sine_sample,

    output reg  signed [7:0]        bpsk_sample,
    output reg                      sample_valid
);

    // unsigned mode: clamp to 127
    wire [6:0] mag7 = (sine_sample > 8'd127) ? 7'd127 : sine_sample[6:0];

    // widen to signed 9 bits
    wire signed [8:0] s9 = SINE_SIGNED ? {sine_sample[7], sine_sample}
                                       : {2'b00, mag7};

    // apply BPSK sign (9 bits: -(-128) = +128 still fits)
    wire signed [8:0] v = bpsk_bit ? s9 : -s9;

    always @(posedge clk) begin
        if (!resetn) begin
            bpsk_sample  <= 8'sd0;
            sample_valid <= 1'b0;
        end else begin
            sample_valid <= enable;
            if (enable) begin
                if (v > 9'sd127)
                    bpsk_sample <= 8'sd127;
                else if (v < -9'sd127)
                    bpsk_sample <= -8'sd127;
                else
                    bpsk_sample <= v[7:0];
            end
        end
    end

endmodule