`timescale 1ns / 1ps
//==============================================================================
// rom : 64 KB synchronous read-only memory for PicoRV32 (loads hex file)
//==============================================================================
module rom #(
    parameter integer WORDS     = 16384,          // 64 KB (16384 x 32-bit words)
    parameter         INIT_FILE = "rom_test.hex"
)(
    input  wire        clk,
    input  wire        resetn,
    input  wire        valid,
    input  wire [31:0] addr,
    output reg  [31:0] rdata,
    output reg         ready
);

    reg [31:0] mem [0:WORDS-1];
    wire [13:0] word_addr = addr[15:2];

    integer i;
    initial begin
        for (i = 0; i < WORDS; i = i + 1)
            mem[i] = 32'h0000_0013; // default to NOP (addi x0, x0, 0)
        if (INIT_FILE != "") begin
            $readmemh(INIT_FILE, mem);
        end
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
            end
        end
    end

endmodule
