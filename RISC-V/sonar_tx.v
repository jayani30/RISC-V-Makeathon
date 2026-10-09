`timescale 1ns / 1ps
//==============================================================================
// sonar_tx_top : payload (input register | RAM) -> sonar_controller -> CRC-8
//                -> serial bits (MSB first) -> BPSK samples
//
// src_sel = 0 : input register (ONE byte only; payload_len must be 1)
// src_sel = 1 : RAM, bytes ram_base_addr .. ram_base_addr+payload_len-1
//               (little-endian lanes: byte a = word[a>>2] bits [8*(a&3)+7 : 8*(a&3)])
// start       : one-clock pulse, accepted only when idle and the length is valid.
//               Invalid -> len_error pulses for one clock, nothing starts.
//               Start while busy is ignored.
// busy        : high from an accepted start until the LAST SAMPLE has been output
// done        : one clock, the cycle after the last sample_valid; busy is low then
//
// BPSK: bit 1 -> +sine, bit 0 -> -sine (BIT1_NEGATIVE = 1 flips it).
// Samples per bit = (256/PHASE_STEP)*CYCLES_PER_BIT; PHASE_STEP = power of two.
// Pipeline: sequencer (cycle k) -> dsin_lut (k+1) -> bpsk_sample_generator (k+2).
//==============================================================================
module sonar_tx_top #(
    parameter MAX_LEN        = 32,
    parameter PHASE_STEP     = 16,
    parameter CYCLES_PER_BIT = 1,
    parameter BIT1_NEGATIVE  = 0,
    parameter [31:0] RAM_BASE = 32'h0100_0000
)(
    input  wire        clk,
    input  wire        resetn,

    input  wire        start,
    input  wire [7:0]  payload_len,
    input  wire        src_sel,
    input  wire [15:0] ram_base_addr,
    output wire        busy,
    output wire        done,
    output wire        len_error,

    output wire signed [7:0] bpsk_sample,
    output wire              sample_valid,

    // digital_input_register bus slave (connect to the address decoder)
    input  wire        in_valid,
    input  wire        in_write_en,
    input  wire [31:0] in_addr,
    input  wire [31:0] in_wdata,
    input  wire [3:0]  in_wstrb,
    output wire [31:0] in_rdata,
    output wire        in_ready,
    output wire        in_load,

    // RAM read master (connect through sonar_ram_arbiter)
    output wire        rm_valid,
    output wire [31:0] rm_addr,
    input  wire [31:0] rm_rdata,
    input  wire        rm_ready,
    output wire        rm_busy,         // 1 while the controller owns the RAM

    // debug / monitoring
    output wire        dbg_bit_valid,
    output wire        dbg_bit_data,
    output wire        dbg_bit_ready,
    output wire        dbg_bit_last,
    output wire [3:0]  dbg_state
);

    localparam SAMPLES_PER_BIT = (256 / PHASE_STEP) * CYCLES_PER_BIT;
    localparam [7:0] PSTEP     = PHASE_STEP;

    // ---------------- wires ----------------
    wire        c_start;
    wire        pl_req;
    wire [7:0]  pl_addr;
    wire        pl_valid;
    wire [7:0]  pl_data;
    wire        crc_clear, crc_valid, crc_ready, crc_done;
    wire [7:0]  crc_data, crc_result;
    wire        bit_valid, bit_data, bit_last, bit_ready;
    wire        c_busy, c_done, c_len_error;
    wire [7:0]  c_byte_cnt;
    wire [2:0]  c_bit_cnt;
    wire [3:0]  state_dbg;
    wire        m0;                      // last sample of the last bit (sequencer)

    wire [7:0]  in_data;

    //==========================================================================
    // Start validation and top-level status
    //==========================================================================
    reg        busy_r, done_r, len_err_r;
    reg        src_lat;
    reg [15:0] base_lat;
    reg        m1, m2;

    wire        ctrl_idle     = (state_dbg == 4'd0) && !busy_r;
    wire [16:0] ram_end       = {1'b0, ram_base_addr} + {9'b0, payload_len};
    wire        len_in_range  = (payload_len != 8'd0) && (payload_len <= MAX_LEN);
    wire        len_ok        = src_sel ? (len_in_range && (ram_end <= 17'd65536))
                                        : (payload_len == 8'd1);
    wire        start_ok      = start && ctrl_idle && len_ok;
    assign c_start = start_ok;

    always @(posedge clk) begin
        if (!resetn) begin
            busy_r    <= 1'b0;
            done_r    <= 1'b0;
            len_err_r <= 1'b0;
            src_lat   <= 1'b0;
            base_lat  <= 16'd0;
            m1        <= 1'b0;
            m2        <= 1'b0;
        end else begin
            len_err_r <= start && ctrl_idle && !len_ok;
            m1        <= m0;
            m2        <= m1;
            done_r    <= m2;
            if (start_ok) begin
                busy_r   <= 1'b1;
                src_lat  <= src_sel;
                base_lat <= ram_base_addr;
            end else if (m2) begin
                busy_r   <= 1'b0;
            end
        end
    end

    assign busy      = busy_r;
    assign done      = done_r;
    assign len_error = len_err_r;

    assign dbg_bit_valid = bit_valid;
    assign dbg_bit_data  = bit_data;
    assign dbg_bit_ready = bit_ready;
    assign dbg_bit_last  = bit_last;
    assign dbg_state     = state_dbg;

    //==========================================================================
    // Sonar controller
    //==========================================================================
    sonar_controller #(.MAX_LEN(MAX_LEN)) u_ctrl (
        .clk(clk), .resetn(resetn),
        .start(c_start), .payload_len(payload_len),
        .busy(c_busy), .done(c_done), .len_error(c_len_error),
        .pl_req(pl_req), .pl_addr(pl_addr), .pl_valid(pl_valid), .pl_data(pl_data),
        .crc_clear(crc_clear), .crc_valid(crc_valid), .crc_data(crc_data),
        .crc_ready(crc_ready), .crc_done(crc_done), .crc_result(crc_result),
        .bit_valid(bit_valid), .bit_data(bit_data), .bit_last(bit_last),
        .bit_ready(bit_ready),
        .byte_cnt(c_byte_cnt), .bit_cnt(c_bit_cnt), .state_dbg(state_dbg)
    );

    //==========================================================================
    // Payload adapter: answers pl_req from the input register or from RAM
    //==========================================================================
    localparam [1:0] A_IDLE = 2'd0, A_REQ = 2'd1, A_RESP = 2'd2;
    reg [1:0] as;
    reg [7:0] pl_data_r;

    wire [15:0] baddr = base_lat + {8'b0, pl_addr};
    wire [7:0]  lane_byte = (baddr[1:0] == 2'd0) ? rm_rdata[7:0]   :
                            (baddr[1:0] == 2'd1) ? rm_rdata[15:8]  :
                            (baddr[1:0] == 2'd2) ? rm_rdata[23:16] :
                                                   rm_rdata[31:24];

    always @(posedge clk) begin
        if (!resetn) begin
            as        <= A_IDLE;
            pl_data_r <= 8'd0;
        end else begin
            case (as)
                A_IDLE: if (pl_req) begin
                    if (src_lat) as <= A_REQ;
                    else begin
                        pl_data_r <= in_data;
                        as        <= A_RESP;
                    end
                end
                A_REQ: if (rm_ready) begin
                    pl_data_r <= lane_byte;
                    as        <= A_RESP;
                end
                A_RESP:  as <= A_IDLE;          // pl_valid lasts exactly one clock
                default: as <= A_IDLE;
            endcase
        end
    end

    assign pl_valid = (as == A_RESP);
    assign pl_data  = pl_data_r;
    assign rm_valid = (as == A_REQ);
    assign rm_addr  = RAM_BASE | {16'b0, baddr[15:2], 2'b00};
    // RAM is owned only during the read phase (CRC_INIT .. CRC_WAIT)
    assign rm_busy  = src_lat && (state_dbg >= 4'd1) && (state_dbg <= 4'd4);

    //==========================================================================
    // Digital input register (single byte source)
    //==========================================================================
    digital_input_register u_inreg (
        .clk(clk), .resetn(resetn),
        .valid(in_valid), .write_en(in_write_en),
        .addr(in_addr), .wdata(in_wdata), .wstrb(in_wstrb),
        .rdata(in_rdata), .ready(in_ready),
        .data_out(in_data), .data_valid(in_load)
    );

    //==========================================================================
    // CRC adapter: sonar_controller CRC interface -> crc8_accelerator bus
    //   crc_clear  : CONTROL (0x04) <= 1
    //   crc_valid  : DATA    (0x00) <= byte, then RESULT (0x08) is read
    //   crc_done   : one clock after RESULT was captured (XOR-out included)
    //==========================================================================
    localparam [2:0] C_IDLE = 3'd0, C_CLR = 3'd1, C_WR = 3'd2, C_RD = 3'd3, C_DONE = 3'd4;
    reg [2:0] cs;
    reg [7:0] cdat, cres;

    wire        cb_valid = (cs == C_CLR) || (cs == C_WR) || (cs == C_RD);
    wire        cb_we    = (cs == C_CLR) || (cs == C_WR);
    wire [31:0] cb_addr  = (cs == C_WR)  ? 32'h0000_0000 :
                           (cs == C_CLR) ? 32'h0000_0004 : 32'h0000_0008;
    wire [31:0] cb_wdata = (cs == C_CLR) ? 32'h0000_0001 : {24'b0, cdat};
    wire [3:0]  cb_wstrb = cb_we ? 4'hF : 4'h0;
    wire [31:0] cb_rdata;
    wire        cb_ready;

    always @(posedge clk) begin
        if (!resetn) begin
            cs   <= C_IDLE;
            cdat <= 8'd0;
            cres <= 8'd0;
        end else begin
            case (cs)
                C_IDLE: begin
                    if (crc_clear) cs <= C_CLR;
                    else if (crc_valid) begin
                        cdat <= crc_data;
                        cs   <= C_WR;
                    end
                end
                C_CLR:  if (cb_ready) cs <= C_IDLE;
                C_WR:   if (cb_ready) cs <= C_RD;
                C_RD:   if (cb_ready) begin
                            cres <= cb_rdata[7:0];
                            cs   <= C_DONE;
                        end
                C_DONE: cs <= C_IDLE;
                default: cs <= C_IDLE;
            endcase
        end
    end

    assign crc_ready  = (cs == C_IDLE);     // byte accepted in the clock where crc_valid && crc_ready
    assign crc_done   = (cs == C_DONE);
    assign crc_result = cres;

    crc8_accelerator u_crc (
        .clk(clk), .resetn(resetn),
        .valid(cb_valid), .write_en(cb_we),
        .addr(cb_addr), .wdata(cb_wdata), .wstrb(cb_wstrb),
        .rdata(cb_rdata), .ready(cb_ready)
    );

    //==========================================================================
    // Bit sequencer: one serial bit -> SAMPLES_PER_BIT samples
    //==========================================================================
    reg        seq_run;
    reg        cur_bit, cur_last, bit_d;
    reg [7:0]  phase;
    reg [15:0] scnt;

    wire last_sample = (scnt == (SAMPLES_PER_BIT - 1));
    assign bit_ready = !seq_run || last_sample;
    wire   bit_take  = bit_valid && bit_ready;
    assign m0        = seq_run && last_sample && cur_last;

    always @(posedge clk) begin
        if (!resetn) begin
            seq_run  <= 1'b0;
            cur_bit  <= 1'b0;
            cur_last <= 1'b0;
            bit_d    <= 1'b0;
            phase    <= 8'd0;
            scnt     <= 16'd0;
        end else begin
            bit_d <= cur_bit;                       // aligns the bit with dsin_lut latency
            if (bit_take) begin
                cur_bit  <= bit_data;
                cur_last <= bit_last;
                phase    <= 8'd0;
                scnt     <= 16'd0;
                seq_run  <= 1'b1;
            end else if (seq_run) begin
                if (last_sample) seq_run <= 1'b0;
                else begin
                    scnt  <= scnt + 16'd1;
                    phase <= phase + PSTEP;
                end
            end
        end
    end

    //==========================================================================
    // Sine LUT and BPSK sample generator
    //==========================================================================
    wire signed [15:0] sin_out;
    wire               sin_valid;
    wire               gen_bit = bit_d ^ (BIT1_NEGATIVE != 0);

    dsin_lut #(.PHASE_WIDTH(8), .OUTPUT_WIDTH(16), .LUT_SIZE(256), .AMPLITUDE(32767)) u_sin (
        .clk(clk), .rst(!resetn),
        .phase(phase), .enable(seq_run),
        .sin_out(sin_out), .out_valid(sin_valid)
    );

    bpsk_sample_generator #(.SINE_SIGNED(1)) u_gen (
        .clk(clk), .resetn(resetn),
        .enable(sin_valid), .bpsk_bit(gen_bit),
        .sine_sample(sin_out[15:8]),
        .bpsk_sample(bpsk_sample), .sample_valid(sample_valid)
    );

endmodule


//==============================================================================
// sonar_ram_arbiter : lets the CPU and sonar_tx_top share the single-port ram.
//   sel_sonar = 1 (connect to sonar_tx_top.rm_busy): the sonar master owns the
//   RAM, CPU accesses are held off (cpu_ready = 0). The sonar master only reads.
//   Do not start a CPU RAM access in the same clock as the sonar start.
//==============================================================================
module sonar_ram_arbiter (
    input  wire        sel_sonar,

    input  wire        cpu_valid,
    input  wire [31:0] cpu_addr,
    input  wire [31:0] cpu_wdata,
    input  wire [3:0]  cpu_wstrb,
    output wire [31:0] cpu_rdata,
    output wire        cpu_ready,

    input  wire        son_valid,
    input  wire [31:0] son_addr,
    output wire [31:0] son_rdata,
    output wire        son_ready,

    output wire        ram_valid,
    output wire [31:0] ram_addr,
    output wire [31:0] ram_wdata,
    output wire [3:0]  ram_wstrb,
    input  wire [31:0] ram_rdata,
    input  wire        ram_ready
);
    assign ram_valid = sel_sonar ? son_valid : cpu_valid;
    assign ram_addr  = sel_sonar ? son_addr  : cpu_addr;
    assign ram_wdata = sel_sonar ? 32'h0     : cpu_wdata;
    assign ram_wstrb = sel_sonar ? 4'h0      : cpu_wstrb;     // sonar never writes

    assign son_rdata = ram_rdata;
    assign son_ready = ram_ready &  sel_sonar;
    assign cpu_rdata = ram_rdata;
    assign cpu_ready = ram_ready & ~sel_sonar;
endmodule