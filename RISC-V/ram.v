`timescale 1ns / 1ps
//==============================================================================
// ram : 64 KB single-port byte-addressable synchronous RAM for PicoRV32
//==============================================================================
module ram #(
    parameter integer WORDS = 16384  // 64 KB (16384 x 32-bit words)
)(
    input  wire        clk,
    input  wire        resetn,
    input  wire        valid,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,
    output reg  [31:0] rdata,
    output reg         ready
);

    reg [31:0] mem [0:WORDS-1];
    wire [13:0] word_addr = addr[15:2];

    integer i;
    initial begin
        for (i = 0; i < WORDS; i = i + 1)
            mem[i] = 32'h0000_0000;
    end

    always @(posedge clk) begin
        if (!resetn) begin
            ready <= 1'b0;
            rdata <= 32'h0000_0000;
        end else begin
            ready <= 1'b0;
            if (valid && !ready) begin
                ready <= 1'b1;
                rdata <= mem[word_addr];
                if (wstrb[0]) mem[word_addr][ 7: 0] <= wdata[ 7: 0];
                if (wstrb[1]) mem[word_addr][15: 8] <= wdata[15: 8];
                if (wstrb[2]) mem[word_addr][23:16] <= wdata[23:16];
                if (wstrb[3]) mem[word_addr][31:24] <= wdata[31:24];
            end
        end
    end

endmodule
