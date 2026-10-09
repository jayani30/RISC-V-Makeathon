`timescale 1ns / 1ps
module tb_sonar_controller;

    localparam MAX_LEN     = 32;
    localparam TIMEOUT_CYC = 30000;

    reg         clk;
    reg         resetn;
    reg         start;
    reg  [7:0]  payload_len;
    wire        busy, done, len_error;
    wire        pl_req;
    wire [7:0]  pl_addr;
    reg         pl_valid;
    reg  [7:0]  pl_data;
    wire        crc_clear, crc_valid;
    wire [7:0]  crc_data;
    wire        crc_ready, crc_done;
    wire [7:0]  crc_result;
    wire        bit_valid, bit_data, bit_last;
    reg         bit_ready;
    wire [7:0]  byte_cnt;
    wire [2:0]  bit_cnt;
    wire [3:0]  state_dbg;

    sonar_controller #(.MAX_LEN(MAX_LEN)) dut (
        .clk(clk), .resetn(resetn),
        .start(start), .payload_len(payload_len),
        .busy(busy), .done(done), .len_error(len_error),
        .pl_req(pl_req), .pl_addr(pl_addr), .pl_valid(pl_valid), .pl_data(pl_data),
        .crc_clear(crc_clear), .crc_valid(crc_valid), .crc_data(crc_data),
        .crc_ready(crc_ready), .crc_done(crc_done), .crc_result(crc_result),
        .bit_valid(bit_valid), .bit_data(bit_data), .bit_last(bit_last),
        .bit_ready(bit_ready),
        .byte_cnt(byte_cnt), .bit_cnt(bit_cnt), .state_dbg(state_dbg)
    );

    initial begin
        $dumpfile("tb_sonar_controller.vcd");
        $dumpvars(1, tb_sonar_controller);
        $dumpvars(1, tb_sonar_controller.dut);
    end

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        #100000000;
        $display("FAIL: simulation timeout");
        $display("====================================");
        $display("SONAR CONTROLLER TEST FAILED");
        $display("====================================");
        $finish;
    end

    //--------------------------------------------------------------
    // bookkeeping
    //--------------------------------------------------------------
    integer errors, tests;
    integer rx_count, crc_push_cnt, pl_cnt, done_count, busy_cycles, len_err_cnt;
    integer exp_total;
    reg     in_tx;
    reg     rx_bits [0:511];
    reg [7:0] mem [0:63];                 // payload source contents
    reg [7:0] sb  [0:5];

    integer seed1;
    reg [31:0] rnd;
    reg        stress;                    // random stalls in source + CRC
    integer    rx_mode;                   // 0 always ready, 1 random, 2 mostly stalled, 3 held low
    reg        src_gate, crc_gate;

    //--------------------------------------------------------------
    // Reference functions (independent of the DUT)
    //--------------------------------------------------------------
    // bit-serial LFSR form, used by the CRC accelerator MODEL
    function [7:0] ref_update;
        input [7:0] crc_in;
        input [7:0] d;
        integer b;
        reg [7:0] c;
        reg fb;
        begin
            c = crc_in;
            for (b = 7; b >= 0; b = b - 1) begin
                fb = c[7] ^ d[b];
                c  = {c[6:0], 1'b0};
                if (fb) c = c ^ 8'h07;
            end
            ref_update = c;
        end
    endfunction

    // XOR-then-shift byte form, used to compute the EXPECTED CRC (poly 0x07,
    // init 0x00, no reflection, XOR-out 0x55)
    function [7:0] exp_crc;
        input integer n;
        integer i, j;
        reg [7:0] c;
        begin
            c = 8'h00;
            for (i = 0; i < n; i = i + 1) begin
                c = c ^ mem[i];
                for (j = 0; j < 8; j = j + 1) begin
                    if (c[7]) c = {c[6:0], 1'b0} ^ 8'h07;
                    else      c = {c[6:0], 1'b0};
                end
            end
            exp_crc = c ^ 8'h55;
        end
    endfunction

    //--------------------------------------------------------------
    // Behavioral model: payload source (1-clock latency, optional stalls)
    //--------------------------------------------------------------
    always @(posedge clk) begin
        rnd = $random(seed1);
        src_gate <= stress ? rnd[0] : 1'b1;
        crc_gate <= stress ? rnd[1] : 1'b1;
        case (rx_mode)
            0: bit_ready <= 1'b1;
            1: bit_ready <= rnd[2];
            2: bit_ready <= (rnd[4:3] == 2'b00);
            default: bit_ready <= 1'b0;
        endcase
        pl_valid <= pl_req && !pl_valid && src_gate;
        pl_data  <= mem[pl_addr];
    end

    //--------------------------------------------------------------
    // Behavioral model: CRC-8 accelerator (3 clocks of processing per byte)
    //--------------------------------------------------------------
    reg [7:0] acc;
    reg       model_done;
    integer   dly;

    assign crc_ready  = model_done && crc_gate;
    assign crc_done   = model_done;
    assign crc_result = acc ^ 8'h55;

    always @(posedge clk) begin
        if (crc_clear) begin
            acc <= 8'h00; model_done <= 1'b1; dly <= 0;
        end else if (crc_valid && crc_ready) begin
            acc <= ref_update(acc, crc_data);
            model_done <= 1'b0;
            dly <= 2;
            crc_push_cnt = crc_push_cnt + 1;
        end else if (!model_done) begin
            if (dly == 0) model_done <= 1'b1;
            else          dly <= dly - 1;
        end
    end

    //--------------------------------------------------------------
    // Monitor: bit receiver + protocol checks (samples pre-edge values)
    //--------------------------------------------------------------
    reg prev_stall, prev_bit, last_acc_prev;

    always @(posedge clk) begin
        // valid/data must not change while stalled
        if (prev_stall) begin
            if (!(bit_valid && (bit_data === prev_bit))) begin
                errors = errors + 1;
                $display("FAIL: time=%0t bit_valid/bit_data changed while bit_ready was low", $time);
            end
        end
        // receiver: accept a bit
        if (bit_valid && bit_ready && in_tx) begin
            if (rx_count < 512) rx_bits[rx_count] = bit_data;
            rx_count = rx_count + 1;
        end
        if (bit_valid && !in_tx) begin
            errors = errors + 1;
            $display("FAIL: time=%0t bit_valid asserted outside a transmission", $time);
        end
        if (in_tx && !done && !busy) begin
            errors = errors + 1;
            $display("FAIL: time=%0t busy low during a transmission", $time);
        end
        if (done) begin
            done_count = done_count + 1;
            if (busy) begin
                errors = errors + 1;
                $display("FAIL: time=%0t busy still high with done", $time);
            end
            if (!in_tx) begin
                errors = errors + 1;
                $display("FAIL: time=%0t done asserted with no transmission", $time);
            end else begin
                if (!last_acc_prev) begin
                    errors = errors + 1;
                    $display("FAIL: time=%0t done without final CRC bit accepted on previous clock", $time);
                end
                if (rx_count != exp_total) begin
                    errors = errors + 1;
                    $display("FAIL: time=%0t done early/late: bits received=%0d expected=%0d",
                             $time, rx_count, exp_total);
                end
                in_tx = 1'b0;
            end
        end
        if (busy)      busy_cycles = busy_cycles + 1;
        if (len_error) len_err_cnt = len_err_cnt + 1;
        if (pl_req && pl_valid) pl_cnt = pl_cnt + 1;

        last_acc_prev = bit_valid && bit_ready && bit_last && resetn;
        prev_stall    = bit_valid && !bit_ready && resetn;
        prev_bit      = bit_data;
    end

    //--------------------------------------------------------------
    // Check helper
    //--------------------------------------------------------------
    task check;
        input [255:0] name;
        input [31:0]  expected;
        input [31:0]  actual;
        begin
            tests = tests + 1;
            if (expected === actual) begin
                // quiet on pass; section summaries are printed by the callers
            end else begin
                errors = errors + 1;
                $display("FAIL: time=%0t %0s  expected=%0d actual=%0d", $time, name, expected, actual);
            end
        end
    endtask

    task clear_counters;
        begin
            rx_count = 0; crc_push_cnt = 0; pl_cnt = 0;
            done_count = 0; busy_cycles = 0; len_err_cnt = 0;
        end
    endtask

    task fill_pattern;
        input integer n;
        input integer sv;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1)
                mem[i] = (i * 37 + sv) % 256;
        end
    endtask

    //--------------------------------------------------------------
    // One complete transmission with all checks
    //   hold  : >0 keeps bit_ready low for that many clocks after start
    //   rxm   : receiver mode after the hold
    //   str   : 1 = random stalls in payload source and CRC engine
    //   poke  : 1 = pulse start again while busy (must be ignored)
    //--------------------------------------------------------------
    task run_tx;
        input [255:0] name;
        input integer len;
        input integer hold;
        input integer rxm;
        input integer str;
        input integer poke;
        integer t, k, errs0, bsel;
        reg [7:0] ec, tb_byte;
        reg exp_bit;
        begin
            errs0 = errors;
            ec = exp_crc(len);
            exp_total = (len + 1) * 8;
            clear_counters;
            stress  = str;
            rx_mode = (hold > 0) ? 3 : rxm;

            @(posedge clk); #1;
            payload_len = len; start = 1'b1;
            @(posedge clk); #1;
            start = 1'b0; in_tx = 1'b1;

            if (hold > 0) begin
                repeat (hold) @(posedge clk);
                #1;
                check("hold: bit_valid stays high", 1, bit_valid);
                check("hold: no bit accepted",      0, rx_count);
                check("hold: busy high",            1, busy);
                check("hold: done low",             0, done);
                rx_mode = rxm;
            end

            t = 0;
            while (done_count == 0 && t < TIMEOUT_CYC) begin
                @(posedge clk); #1;
                t = t + 1;
                if (poke && t == 15) begin
                    start = 1'b1; payload_len = 8'd1;
                    @(posedge clk); #1;
                    start = 1'b0; payload_len = len;
                end
            end
            if (done_count == 0) in_tx = 1'b0;

            repeat (12) @(posedge clk);
            #1;

            check("transmission completed (done seen once)", 1, done_count);
            check("total bits received = (len+1)*8", exp_total, rx_count);
            check("CRC engine received len bytes",   len, crc_push_cnt);
            check("payload source read len bytes",   len, pl_cnt);
            check("busy low after done",             0, busy);
            check("state IDLE after done",           0, state_dbg);
            check("busy was asserted",               1, (busy_cycles > 0));
            check("no len_error for valid length",   0, len_err_cnt);

            for (k = 0; k < exp_total; k = k + 1) begin
                bsel = 7 - (k % 8);                       // MSB first
                if ((k / 8) < len) begin
                    tb_byte = mem[k / 8];
                    exp_bit = tb_byte[bsel];
                end else
                    exp_bit = ec[bsel];                   // CRC follows payload
                tests = tests + 1;
                if (rx_bits[k] !== exp_bit) begin
                    errors = errors + 1;
                    $display("FAIL: %0s bit %0d (byte %0d, bit %0d) expected=%b actual=%b",
                             name, k, k / 8, bsel, exp_bit, rx_bits[k]);
                end
            end

            if (errors == errs0)
                $display("PASS: %0s  len=%0d crc=0x%02h bits=%0d", name, len, ec, exp_total);
            else
                $display("FAIL: %0s had errors", name);

            stress = 1'b0; rx_mode = 0;
            repeat (3) @(posedge clk);
            #1;
        end
    endtask

    // start with an invalid length: must be rejected, no activity
    task run_bad_len;
        input integer len;
        integer errs0;
        begin
            errs0 = errors;
            exp_total = 0; clear_counters; in_tx = 1'b0; stress = 1'b0; rx_mode = 0;
            @(posedge clk); #1;
            payload_len = len; start = 1'b1;
            @(posedge clk); #1;
            start = 1'b0;
            repeat (15) @(posedge clk);
            #1;
            check("bad len: len_error pulsed once", 1, len_err_cnt);
            check("bad len: busy never asserted",   0, busy_cycles);
            check("bad len: no payload request",    0, pl_cnt);
            check("bad len: no CRC activity",       0, crc_push_cnt);
            check("bad len: no bits sent",          0, rx_count);
            check("bad len: no done",               0, done_count);
            check("bad len: state IDLE",            0, state_dbg);
            if (errors == errs0) $display("PASS: invalid payload_len=%0d rejected", len);
            else                 $display("FAIL: invalid payload_len=%0d", len);
        end
    endtask

    // reset during a transmission
    //   until_bits > 0 : wait until that many bits were accepted
    //   otherwise      : wait wcyc clocks
    task reset_mid;
        input [255:0] name;
        input integer len;
        input integer wcyc;
        input integer until_bits;
        integer t, errs0, snap;
        begin
            errs0 = errors;
            exp_total = (len + 1) * 8;
            clear_counters;
            stress = 1'b1; rx_mode = 1;
            @(posedge clk); #1;
            payload_len = len; start = 1'b1;
            @(posedge clk); #1;
            start = 1'b0; in_tx = 1'b1;

            if (until_bits > 0) begin
                t = 0;
                while (rx_count < until_bits && t < TIMEOUT_CYC) begin
                    @(posedge clk); #1; t = t + 1;
                end
            end else begin
                repeat (wcyc) @(posedge clk);
                #1;
            end
            check("reset test: busy before reset", 1, busy);

            resetn = 1'b0;
            @(posedge clk); #1;             // reset edge: DUT is IDLE now
            in_tx = 1'b0;
            @(posedge clk); #1;
            resetn = 1'b1;
            check("after reset: state IDLE",  0, state_dbg);
            check("after reset: busy low",    0, busy);
            check("after reset: done low",    0, done);
            check("after reset: bit_valid low", 0, bit_valid);
            check("after reset: pl_req low",  0, pl_req);
            check("after reset: crc_valid low", 0, crc_valid);
            snap = rx_count;
            repeat (40) @(posedge clk);
            #1;
            check("after reset: no more bits sent", snap, rx_count);
            check("after reset: no done pulse",     0, done_count);
            check("after reset: still IDLE",        0, state_dbg);
            if (errors == errs0) $display("PASS: %0s", name);
            else                 $display("FAIL: %0s", name);
            stress = 1'b0; rx_mode = 0;
            repeat (10) @(posedge clk);
            #1;
        end
    endtask

    integer i, cyc, errs_sec;

    //--------------------------------------------------------------
    // main sequence
    //--------------------------------------------------------------
    initial begin
        errors = 0; tests = 0; seed1 = 4242;
        rx_count = 0; crc_push_cnt = 0; pl_cnt = 0;
        done_count = 0; busy_cycles = 0; len_err_cnt = 0; exp_total = 0;
        in_tx = 1'b0; stress = 1'b0; rx_mode = 0;
        prev_stall = 1'b0; prev_bit = 1'b0; last_acc_prev = 1'b0;
        src_gate = 1'b1; crc_gate = 1'b1;
        pl_valid = 1'b0; pl_data = 8'h00; bit_ready = 1'b0;
        acc = 8'h00; model_done = 1'b1; dly = 0;
        start = 1'b0; payload_len = 8'd0;
        for (i = 0; i < 64; i = i + 1) mem[i] = 8'h00;
        sb[0] = 8'h00; sb[1] = 8'h01; sb[2] = 8'h80;
        sb[3] = 8'h55; sb[4] = 8'hAA; sb[5] = 8'hFF;

        resetn = 1'b0;
        repeat (4) @(posedge clk);
        #1;

        // 1. reset state
        $display("--- 1: reset places controller in IDLE ---");
        check("reset: state IDLE",    0, state_dbg);
        check("reset: busy low",      0, busy);
        check("reset: done low",      0, done);
        check("reset: bit_valid low", 0, bit_valid);
        check("reset: pl_req low",    0, pl_req);
        check("reset: crc_valid low", 0, crc_valid);
        resetn = 1'b1;
        @(posedge clk); #1;

        // 2. no start -> nothing happens
        $display("--- 2: no transmission without start ---");
        payload_len = 8'd5;
        clear_counters;
        repeat (30) begin
            @(posedge clk); #1;
            check("idle: busy low",      0, busy);
            check("idle: bit_valid low", 0, bit_valid);
            check("idle: pl_req low",    0, pl_req);
            check("idle: crc_valid low", 0, crc_valid);
            check("idle: state IDLE",    0, state_dbg);
        end
        check("idle: no bits",  0, rx_count);
        check("idle: no done",  0, done_count);

        // 3-4. one-byte payloads (MSB-first corner values)
        $display("--- 3: one-byte payloads ---");
        for (i = 0; i < 6; i = i + 1) begin
            mem[0] = sb[i];
            run_tx("one byte", 1, 0, 0, 0, 0);
        end

        // 5. multi-byte payloads
        $display("--- 4: multi-byte payloads ---");
        mem[0] = 8'hDE; mem[1] = 8'hAD; mem[2] = 8'hBE;
        run_tx("3 bytes DEADBE", 3, 0, 0, 0, 0);
        fill_pattern(7, 3);
        run_tx("7 bytes pattern", 7, 0, 0, 0, 0);

        // "123456789" must give 0xA1
        $display("--- 5: ASCII 123456789 ---");
        mem[0]="1"; mem[1]="2"; mem[2]="3"; mem[3]="4"; mem[4]="5";
        mem[5]="6"; mem[6]="7"; mem[7]="8"; mem[8]="9";
        check("expected CRC of 123456789 is 0xA1", 32'hA1, exp_crc(9));
        run_tx("123456789", 9, 0, 0, 0, 0);

        // maximum length
        $display("--- 6: maximum payload length (%0d) ---", MAX_LEN);
        fill_pattern(MAX_LEN, 11);
        run_tx("max length", MAX_LEN, 0, 0, 0, 0);

        // 12. backpressure: receiver held low, then released
        $display("--- 7: backpressure (receiver not ready) ---");
        mem[0] = 8'hC3; mem[1] = 8'h3C; mem[2] = 8'h81;
        run_tx("held backpressure", 3, 200, 0, 0, 0);

        $display("--- 8: random backpressure + source/CRC stalls ---");
        fill_pattern(9, 5);
        run_tx("random ready 50%",    9, 0, 1, 1, 0);
        run_tx("mostly stalled 25%",  5, 0, 2, 1, 0);
        fill_pattern(MAX_LEN, 21);
        run_tx("max length stressed", MAX_LEN, 0, 1, 1, 0);

        // start while busy is ignored
        $display("--- 9: start while busy is ignored ---");
        fill_pattern(9, 77);
        run_tx("start ignored while busy", 9, 0, 0, 0, 1);

        // 15. invalid lengths
        $display("--- 10: invalid payload length ---");
        run_bad_len(0);
        run_bad_len(MAX_LEN + 1);
        run_bad_len(255);

        // 13. reset during transmission
        $display("--- 11: reset during operation ---");
        fill_pattern(9, 9);
        reset_mid("reset during CRC phase",  9, 6, 0);
        fill_pattern(3, 99);
        reset_mid("reset during bit stream", 3, 0, 12);

        // normal operation after reset, then back-to-back transmissions
        $display("--- 12: recovery and back-to-back ---");
        mem[0]="1"; mem[1]="2"; mem[2]="3"; mem[3]="4"; mem[4]="5";
        mem[5]="6"; mem[6]="7"; mem[7]="8"; mem[8]="9";
        run_tx("after reset", 9, 0, 0, 0, 0);
        fill_pattern(4, 200);
        run_tx("back-to-back A", 4, 0, 1, 0, 0);
        fill_pattern(2, 17);
        run_tx("back-to-back B", 2, 0, 0, 0, 0);

        $display("-----------------------------");
        $display("Checks run: %0d, errors: %0d", tests, errors);
        if (errors == 0) begin
            $display("====================================");
            $display("SONAR CONTROLLER TEST PASSED");
            $display("====================================");
        end else begin
            $display("====================================");
            $display("SONAR CONTROLLER TEST FAILED");
            $display("====================================");
        end
        $finish;
    end

endmodule