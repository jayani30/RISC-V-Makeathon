`timescale 1ns / 1ps
//==============================================================================
// bpsk_modulator : baseband BPSK sample generator
//
//   bit 0 -> +AMPLITUDE
//   bit 1 -> -AMPLITUDE
//
// Behaviour:
//   * data_valid = 1 : data_in is latched into an internal register.
//   * sample_en  = 1 : on that clock edge bpsk_out is updated from the LATCHED
//                      bit and out_valid goes high for one cycle.
//   * A bit latched on edge N is used by a sample_en seen on edge N+1 or later
//     (if both occur on the same edge, the OLD latched bit is used).
//   * When sample_en = 0, out_valid = 0 and bpsk_out holds its last value.
//   * After reset the latched bit is 0, so the first sample is +AMPLITUDE.
//   * Output is registered, so it appears one clock after sample_en.
//
// Reset: active-high, synchronous.
//==============================================================================
module bpsk_modulator #(
    parameter integer OUTPUT_WIDTH = 16,
    parameter integer AMPLITUDE    = 1000   // must fit in OUTPUT_WIDTH-1 bits
)(
    input  wire                           clk,
    input  wire                           rst,
    input  wire                           data_in,
    input  wire                           data_valid,
    input  wire                           sample_en,
    output reg  signed [OUTPUT_WIDTH-1:0] bpsk_out,
    output reg                            out_valid
);

    localparam signed [OUTPUT_WIDTH-1:0] POS_AMP =  AMPLITUDE;
    localparam signed [OUTPUT_WIDTH-1:0] NEG_AMP = -AMPLITUDE;

    reg bit_reg;   // latched input bit

    always @(posedge clk) begin
        if (rst) begin
            bit_reg   <= 1'b0;
            bpsk_out  <= {OUTPUT_WIDTH{1'b0}};
            out_valid <= 1'b0;
        end else begin
            out_valid <= 1'b0;

            if (data_valid)
                bit_reg <= data_in;

            if (sample_en) begin
                bpsk_out  <= bit_reg ? NEG_AMP : POS_AMP;
                out_valid <= 1'b1;
            end
        end
    end

endmodule