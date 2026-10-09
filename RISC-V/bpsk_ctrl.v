`timescale 1ns / 1ps
//==============================================================================
// bpsk_ctrl : Memory-Mapped Control Register block for Sonar TX & BPSK
//
// Address offset: 0x0200_2000 - 0x0200_2FFF
//
// Register Map (offset = addr[7:0], word aligned):
//   0x00 CONTROL    (RW):
//          bit 0 = START      (Write 1: pulses sonar_start for 1 clock)
//          bit 1 = SRC_SEL    (0: Input Register [1 byte], 1: RAM)
//          bit 2 = RESET_TX   (soft reset request)
//   0x04 CONFIG     (RW):
//          bits [7:0]   = PAYLOAD_LEN   (1 .. MAX_LEN bytes)
//          bits [31:16] = RAM_BASE_ADDR (RAM byte offset, e.g. 16'h0000)
//   0x08 STATUS     (R):
//          bit 0 = BUSY       (from sonar_tx_top)
//          bit 1 = DONE       (sticky, set on done pulse, cleared on START)
//          bit 2 = LEN_ERROR  (sticky, set on len_error pulse, cleared on START)
//   0x0C SAMPLE     (R):
//          bits [7:0]   = LAST_SAMPLE (signed 8-bit bpsk_sample)
//          bit 8        = SAMPLE_VALID (current cycle valid)
//          bits [31:16] = SAMPLE_COUNT (total valid samples in current/last tx)
//   0x10 DIRECT_MOD (RW):
//          bit 0  = DIRECT_DATA_IN  (drives bpsk_modulator.data_in)
//          bit 1  = DIRECT_DATA_VAL (drives bpsk_modulator.data_valid)
//          bit 2  = DIRECT_SMPL_EN  (drives bpsk_modulator.sample_en)
//          bit 16 = DIRECT_OUT_VAL  (from bpsk_modulator.out_valid)
//   0x14 DIRECT_OUT (R):
//          bits [15:0] = DIRECT_BPSK_OUT (signed 16-bit output from bpsk_modulator)
//==============================================================================
module bpsk_ctrl (
    input  wire        clk,
    input  wire        resetn,

    // MMIO bus interface (from mmio_address_decoder)
    input  wire        valid,
    input  wire        write_en,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,
    output reg  [31:0] rdata,
    output reg         ready,

    // Interface to sonar_tx_top
    output reg         sonar_start,
    output wire [7:0]  sonar_payload_len,
    output wire        sonar_src_sel,
    output wire [15:0] sonar_ram_base_addr,
    input  wire        sonar_busy,
    input  wire        sonar_done,
    input  wire        sonar_len_error,
    input  wire signed [7:0] sonar_bpsk_sample,
    input  wire        sonar_sample_valid,

    // Interface to standalone bpsk_modulator
    output reg         direct_data_in,
    output reg         direct_data_valid,
    output reg         direct_sample_en,
    input  wire signed [15:0] direct_bpsk_out,
    input  wire        direct_out_valid
);

    reg        src_sel_reg;
    reg [7:0]  len_reg;
    reg [15:0] base_addr_reg;
    reg        done_sticky;
    reg        len_err_sticky;

    reg signed [7:0] last_sample;
    reg [15:0]       sample_count;

    assign sonar_payload_len   = len_reg;
    assign sonar_src_sel       = src_sel_reg;
    assign sonar_ram_base_addr = base_addr_reg;

    wire aligned = (addr[1:0] == 2'b00);

    always @(posedge clk) begin
        if (!resetn) begin
            ready             <= 1'b0;
            rdata             <= 32'h0000_0000;
            sonar_start       <= 1'b0;
            src_sel_reg       <= 1'b0;
            len_reg           <= 8'd1;
            base_addr_reg     <= 16'd0;
            done_sticky       <= 1'b0;
            len_err_sticky    <= 1'b0;
            last_sample       <= 8'sd0;
            sample_count      <= 16'd0;
            direct_data_in    <= 1'b0;
            direct_data_valid <= 1'b0;
            direct_sample_en  <= 1'b0;
        end else begin
            ready       <= 1'b0;
            sonar_start <= 1'b0;

            // Monitor sonar_tx events
            if (sonar_done)
                done_sticky <= 1'b1;
            if (sonar_len_error)
                len_err_sticky <= 1'b1;

            if (sonar_sample_valid) begin
                last_sample  <= sonar_bpsk_sample;
                sample_count <= sample_count + 16'd1;
            end

            // Bus access
            if (valid && !ready) begin
                ready <= 1'b1;
                rdata <= 32'h0000_0000;

                if (write_en && aligned) begin
                    case (addr[7:2])
                        6'd0: begin // 0x00 CONTROL
                            if (wstrb[0]) begin
                                if (wdata[0]) begin
                                    sonar_start    <= 1'b1;
                                    done_sticky    <= 1'b0;
                                    len_err_sticky <= 1'b0;
                                    sample_count   <= 16'd0;
                                end
                                src_sel_reg <= wdata[1];
                            end
                        end

                        6'd1: begin // 0x04 CONFIG
                            if (wstrb[0]) len_reg           <= wdata[7:0];
                            if (wstrb[2]) base_addr_reg[7:0]  <= wdata[23:16];
                            if (wstrb[3]) base_addr_reg[15:8] <= wdata[31:24];
                        end

                        6'd4: begin // 0x10 DIRECT_MOD
                            if (wstrb[0]) begin
                                direct_data_in    <= wdata[0];
                                direct_data_valid <= wdata[1];
                                direct_sample_en  <= wdata[2];
                            end
                        end

                        default: ;
                    endcase
                end else if (!write_en && aligned) begin
                    case (addr[7:2])
                        6'd0: rdata <= {30'b0, src_sel_reg, 1'b0};
                        6'd1: rdata <= {base_addr_reg, 8'b0, len_reg};
                        6'd2: rdata <= {29'b0, len_err_sticky, done_sticky, sonar_busy};
                        6'd3: rdata <= {sample_count, 7'b0, sonar_sample_valid, last_sample};
                        6'd4: rdata <= {15'b0, direct_out_valid, 13'b0, direct_sample_en, direct_data_valid, direct_data_in};
                        6'd5: rdata <= {{16{direct_bpsk_out[15]}}, direct_bpsk_out};
                        default: rdata <= 32'h0000_0000;
                    endcase
                end
            end
        end
    end

endmodule
