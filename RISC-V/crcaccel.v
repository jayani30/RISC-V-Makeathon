`timescale 1ns / 1ps
//==============================================================================
// crc8_accelerator : memory-mapped CRC-8/ITU (I-432-1) accelerator
//
// Parameters : poly = 0x07, init = 0x00, RefIn = false, RefOut = false,
//              XorOut = 0x55   (check value for "123456789" = 0xA1)
//
// Algorithm (MSB-first, bit-by-bit, per byte):
//   1. c = crc ^ data_byte
//   2. repeat 8 times:
//        if c[7] == 1 : c = (c << 1) ^ 0x07
//        else         : c =  c << 1
//   3. crc = c
//   The accumulator holds the RAW crc. XorOut (0x55) is applied only when
//   RESULT is read, so the accumulator can keep running across bytes.
//
// Register map (word offsets, only addr[3:2] is decoded):
//   0x00 DATA    W : write byte in wdata[7:0] -> fed into CRC. Reads return 0.
//   0x04 CONTROL W : bit0 = 1 resets CRC accumulator to init (0x00).
//   0x08 RESULT  R : {24'b0, crc ^ 0x55}
//   0x0C STATUS  R : bit0 = DONE
//
// Timing:
//   * Single-cycle peripheral. When valid=1 and ready=0, the access is
//     performed on that clock edge and ready goes high on the SAME edge,
//     for exactly one cycle.
//   * A DATA write updates the CRC on the edge where ready rises, so the
//     new RESULT is readable by the very next bus transaction.
//   * rdata is registered and valid while ready=1.
//   * Keep valid asserted until ready is seen (PicoRV32 does this).
//     The access executes once only, because it needs valid && !ready.
//   * DONE = 1 after a DATA byte has been processed. It is cleared by
//     reset or a CONTROL reset write. Because processing takes one cycle,
//     it is already 1 when the next transaction reads STATUS.
//==============================================================================
module crc8_accelerator (
    input  wire        clk,
    input  wire        resetn,

    input  wire        valid,
    input  wire        write_en,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,

    output reg  [31:0] rdata,
    output reg         ready
);

    localparam [7:0] CRC_POLY   = 8'h07;
    localparam [7:0] CRC_INIT   = 8'h00;
    localparam [7:0] CRC_XOROUT = 8'h55;

    reg [7:0] crc_reg;   // raw CRC accumulator
    reg       done;

    // Process one byte, MSB first
    function [7:0] crc8_byte;
        input [7:0] crc_in;
        input [7:0] data;
        integer     i;
        reg [7:0]   c;
        begin
            c = crc_in ^ data;
            for (i = 0; i < 8; i = i + 1) begin
                if (c[7])
                    c = {c[6:0], 1'b0} ^ CRC_POLY;
                else
                    c = {c[6:0], 1'b0};
            end
            crc8_byte = c;
        end
    endfunction

    always @(posedge clk) begin
        if (!resetn) begin
            crc_reg <= CRC_INIT;
            done    <= 1'b0;
            ready   <= 1'b0;
            rdata   <= 32'b0;
        end else begin
            ready <= 1'b0;                 // default: one-cycle pulse

            if (valid && !ready) begin
                ready <= 1'b1;
                rdata <= 32'b0;

                if (write_en) begin
                    case (addr[3:2])
                        2'd0: begin        // DATA
                            if (wstrb[0]) begin
                                crc_reg <= crc8_byte(crc_reg, wdata[7:0]);
                                done    <= 1'b1;
                            end
                        end
                        2'd1: begin        // CONTROL
                            if (wstrb[0] && wdata[0]) begin
                                crc_reg <= CRC_INIT;
                                done    <= 1'b0;
                            end
                        end
                        default: ;         // RESULT/STATUS are read-only
                    endcase
                end else begin
                    case (addr[3:2])
                        2'd2: rdata <= {24'b0, crc_reg ^ CRC_XOROUT}; // RESULT
                        2'd3: rdata <= {31'b0, done};                 // STATUS
                        default: rdata <= 32'b0;
                    endcase
                end
            end
        end
    end

endmodule