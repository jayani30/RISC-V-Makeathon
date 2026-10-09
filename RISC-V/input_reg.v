`timescale 1ns / 1ps
//==============================================================================
// digital_input_register : memory-mapped 8-bit input register (Verilog-2001)
//
// Register map (offset = addr[7:0], word aligned):
//   0x00 DATA    RW : write wdata[7:0] (needs wstrb[0]=1) stores the byte.
//                     read returns {24'b0, stored byte}.
//   0x04 CONTROL W  : bit0 = LOAD. Writing 1 generates a one-clock data_valid
//                     pulse and sets STATUS.DATA_VALID. Reads return 0.
//   0x08 STATUS  R  : bit0 = DATA_VALID (sticky). Set by a LOAD write, cleared
//                     by a new DATA write or by reset. Upper bits read 0.
//   Any other offset (0x0C, 0x10 ..., or addr[1:0] != 0) is INVALID:
//     writes are ignored, reads return 0, but ready is still given so the
//     bus never hangs.
//   addr[31:8] are ignored: the SoC address decoder does the base-address select.
//
// Bus timing (same style as the CRC-8 block):
//   * valid && !ready : the access happens on that clock edge; ready goes high
//     on the SAME edge for exactly one cycle. Keep valid high until ready is seen.
//   * Reads: rdata is registered and valid while ready = 1.
//   * A DATA write updates data_out on the same edge ready rises.
//   * A LOAD write sets data_valid on the same edge ready rises; data_valid
//     lasts exactly one clock cycle.
//==============================================================================
module digital_input_register (
    input  wire        clk,
    input  wire        resetn,      // active-low, synchronous

    input  wire        valid,
    input  wire        write_en,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,

    output reg  [31:0] rdata,
    output reg         ready,

    output wire [7:0]  data_out,
    output wire        data_valid
);

    reg [7:0] data_reg;       // stored byte
    reg       load_pulse;     // one-cycle data_valid
    reg       status_valid;   // sticky STATUS.DATA_VALID

    wire aligned     = (addr[1:0] == 2'b00);
    wire sel_data    = aligned && (addr[7:2] == 6'd0);   // 0x00
    wire sel_control = aligned && (addr[7:2] == 6'd1);   // 0x04
    wire sel_status  = aligned && (addr[7:2] == 6'd2);   // 0x08

    assign data_out   = data_reg;
    assign data_valid = load_pulse;

    always @(posedge clk) begin
        if (!resetn) begin
            data_reg     <= 8'h00;
            load_pulse   <= 1'b0;
            status_valid <= 1'b0;
            ready        <= 1'b0;
            rdata        <= 32'h0000_0000;
        end else begin
            ready      <= 1'b0;      // defaults: one-cycle pulses
            load_pulse <= 1'b0;

            if (valid && !ready) begin
                ready <= 1'b1;
                rdata <= 32'h0000_0000;

                if (write_en) begin
                    if (sel_data) begin
                        if (wstrb[0]) begin
                            data_reg     <= wdata[7:0];
                            status_valid <= 1'b0;      // new data not loaded yet
                        end
                    end else if (sel_control) begin
                        if (wstrb[0] && wdata[0]) begin
                            load_pulse   <= 1'b1;
                            status_valid <= 1'b1;
                        end
                    end
                    // STATUS and invalid offsets: write ignored
                end else begin
                    if (sel_data)        rdata <= {24'b0, data_reg};
                    else if (sel_status) rdata <= {31'b0, status_valid};
                    // CONTROL and invalid offsets read 0
                end
            end
        end
    end

endmodule