`timescale 1ns / 1ps
//==============================================================================
// sonar_controller : payload -> CRC-8 -> serial BPSK bit stream controller
//
// Sequence:
//   IDLE -> CRC_INIT -> [READ_PAYLOAD -> CRC_START -> CRC_WAIT] x len
//        -> SEND_PAYLOAD (len*8 bits) -> SEND_CRC (8 bits) -> DONE -> IDLE
//
// Bit order: MSB first for every payload byte and for the CRC byte.
//
// Start / length rules:
//   * start is accepted only in IDLE. payload_len is sampled on that edge.
//   * payload_len == 0 or > MAX_LEN : start is rejected, len_error pulses for
//     one clock, state stays IDLE, no request / no transmission.
//   * start in any other state is ignored.
//
// Payload source handshake (request / response):
//   READ_PAYLOAD: pl_req = 1 and pl_addr = byte index. The source answers with
//   a ONE-cycle pl_valid + pl_data (any latency >= 1 clock). The controller
//   captures pl_data on the clock where pl_req && pl_valid, then drops pl_req.
//   The source must not answer unless pl_req was high.
//
// CRC engine handshake (explicit interface):
//   CRC_INIT : crc_clear = 1 for one clock -> engine resets to init (0x00).
//   CRC_START: crc_valid = 1, crc_data = byte, held until crc_ready = 1.
//              The byte is accepted on the clock where crc_valid && crc_ready.
//   CRC_WAIT : wait for crc_done = 1 (byte processed, crc_result valid).
//              The engine MUST drive crc_done low by the clock after it
//              accepts a byte (same edge), otherwise the controller would
//              see a stale done. After the LAST byte, crc_result is captured
//              once into crc_reg and is never read again (sent exactly once).
//   crc_result must already include the XOR-out (0x55): it is the final CRC.
//
// Bit stream handshake (valid / ready):
//   SEND_PAYLOAD / SEND_CRC: bit_valid = 1 with bit_data. A bit is transferred
//   on the clock where bit_valid && bit_ready. While bit_valid && !bit_ready,
//   bit_valid and bit_data stay unchanged. bit_last = 1 on the final CRC bit.
//
// Status:
//   busy = 1 from the clock after an accepted start until the final CRC bit is
//          accepted (it is 0 in the DONE cycle).
//   done = 1 for exactly one clock (state DONE), the clock AFTER the last CRC
//          bit was accepted.
//
// Reset: active-low, synchronous, highest priority. Any state -> IDLE,
//   counters cleared, busy/done/bit_valid/pl_req/crc_valid all low.
//
// state_dbg encoding: 0 IDLE, 1 CRC_INIT, 2 READ_PAYLOAD, 3 CRC_START,
//   4 CRC_WAIT, 5 SEND_PAYLOAD, 6 SEND_CRC, 7 DONE.
//==============================================================================
module sonar_controller #(
    parameter MAX_LEN = 32            // maximum payload bytes (1..255)
)(
    input  wire        clk,
    input  wire        resetn,

    // command
    input  wire        start,
    input  wire [7:0]  payload_len,
    output wire        busy,
    output wire        done,
    output reg         len_error,

    // payload source
    output wire        pl_req,
    output wire [7:0]  pl_addr,
    input  wire        pl_valid,
    input  wire [7:0]  pl_data,

    // CRC engine
    output wire        crc_clear,
    output wire        crc_valid,
    output wire [7:0]  crc_data,
    input  wire        crc_ready,
    input  wire        crc_done,
    input  wire [7:0]  crc_result,

    // serial bit output (to BPSK datapath)
    output wire        bit_valid,
    output wire        bit_data,
    output wire        bit_last,
    input  wire        bit_ready,

    // progress / debug
    output wire [7:0]  byte_cnt,
    output wire [2:0]  bit_cnt,
    output wire [3:0]  state_dbg
);

    localparam [3:0] IDLE         = 4'd0,
                     CRC_INIT     = 4'd1,
                     READ_PAYLOAD = 4'd2,
                     CRC_START    = 4'd3,
                     CRC_WAIT     = 4'd4,
                     SEND_PAYLOAD = 4'd5,
                     SEND_CRC     = 4'd6,
                     DONE         = 4'd7;

    reg [3:0] state;
    reg [7:0] len_reg;                // latched payload length
    reg [7:0] byte_idx;               // current byte
    reg [2:0] bit_idx;                // current bit within the byte (0 = MSB)
    reg [7:0] data_reg;               // byte just read (goes to the CRC engine)
    reg [7:0] crc_reg;                // final CRC, captured once

    reg [7:0] buffer [0:MAX_LEN-1];   // payload storage (no reset needed)

    wire len_ok     = (payload_len != 8'd0) && (payload_len <= MAX_LEN);
    wire last_byte  = (byte_idx == (len_reg - 8'd1));
    wire sending    = (state == SEND_PAYLOAD) || (state == SEND_CRC);
    wire [7:0] cur_byte = (state == SEND_CRC) ? crc_reg : buffer[byte_idx];

    // ---------------- outputs ----------------
    assign busy      = (state != IDLE) && (state != DONE);
    assign done      = (state == DONE);
    assign pl_req    = (state == READ_PAYLOAD);
    assign pl_addr   = byte_idx;
    assign crc_clear = (state == CRC_INIT);
    assign crc_valid = (state == CRC_START);
    assign crc_data  = data_reg;
    assign bit_valid = sending;
    assign bit_data  = sending ? cur_byte[3'd7 - bit_idx] : 1'b0;   // MSB first
    assign bit_last  = (state == SEND_CRC) && (bit_idx == 3'd7);
    assign byte_cnt  = byte_idx;
    assign bit_cnt   = bit_idx;
    assign state_dbg = state;

    // ---------------- FSM ----------------
    always @(posedge clk) begin
        if (!resetn) begin
            state     <= IDLE;
            len_reg   <= 8'd0;
            byte_idx  <= 8'd0;
            bit_idx   <= 3'd0;
            data_reg  <= 8'd0;
            crc_reg   <= 8'd0;
            len_error <= 1'b0;
        end else begin
            len_error <= 1'b0;                       // default: one-cycle pulse

            case (state)
                IDLE: begin
                    if (start) begin
                        if (len_ok) begin
                            len_reg  <= payload_len;
                            byte_idx <= 8'd0;
                            bit_idx  <= 3'd0;
                            state    <= CRC_INIT;
                        end else begin
                            len_error <= 1'b1;       // rejected, stay IDLE
                        end
                    end
                end

                CRC_INIT: begin                      // crc_clear = 1 this clock
                    state <= READ_PAYLOAD;
                end

                READ_PAYLOAD: begin                  // pl_req = 1
                    if (pl_valid) begin
                        buffer[byte_idx] <= pl_data;
                        data_reg         <= pl_data;
                        state            <= CRC_START;
                    end
                end

                CRC_START: begin                     // crc_valid = 1
                    if (crc_ready)
                        state <= CRC_WAIT;
                end

                CRC_WAIT: begin
                    if (crc_done) begin
                        if (last_byte) begin
                            crc_reg  <= crc_result;  // captured exactly once
                            byte_idx <= 8'd0;
                            bit_idx  <= 3'd0;
                            state    <= SEND_PAYLOAD;
                        end else begin
                            byte_idx <= byte_idx + 8'd1;
                            state    <= READ_PAYLOAD;
                        end
                    end
                end

                SEND_PAYLOAD: begin                  // bit_valid = 1
                    if (bit_ready) begin
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            if (last_byte) begin
                                byte_idx <= 8'd0;
                                state    <= SEND_CRC;
                            end else begin
                                byte_idx <= byte_idx + 8'd1;
                            end
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end
                end

                SEND_CRC: begin                      // bit_valid = 1
                    if (bit_ready) begin
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= DONE;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end
                end

                DONE: begin                          // done = 1 for one clock
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule