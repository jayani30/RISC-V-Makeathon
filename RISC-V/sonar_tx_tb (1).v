`timescale 1ns / 1ps
module tb_sonar_tx_top;

    localparam MAX_LEN         = 16;
    localparam PHASE_STEP      = 32;                                  // 8 samples per sine cycle
    localparam CYCLES_PER_BIT  = 1;
    localparam SPB             = (256 / PHASE_STEP) * CYCLES_PER_BIT; // samples per bit = 8
    localparam [31:0] RAM_BASE = 32'h0100_0000;
    localparam TIMEOUT_CYC     = 20000;

    reg         clk, resetn;
    reg         start;
    reg  [7:0]  payload_len;
    reg         src_sel;
    reg  [15:0] ram_base_addr;
    wire        busy, done, len_error;
    wire signed [7:0] bpsk_sample;
    wire        sample_valid;

    // input register bus
    reg         in_valid, in_write_en;
    reg  [31:0] in_addr, in_wdata;
    reg  [3:0]  in_wstrb;
    wire [31:0] in_rdata;
    wire        in_ready, in_load;

    // sonar RAM master
    wire        rm_valid, rm_ready, rm_busy;
    wire [31:0] rm_addr, rm_rdata;

    // CPU side of the RAM
    reg         cpu_valid;
    reg  [31:0] cpu_addr, cpu_wdata;
    reg  [3:0]  cpu_wstrb;
    wire [31:0] cpu_rdata;
    wire        cpu_ready;

    // RAM
    wire        ram_valid, ram_ready;
    wire [31:0] ram_addr, ram_wdata, ram_rdata;
    wire [3:0]  ram_wstrb;

    // debug
    wire        dbg_bit_valid, dbg_bit_data, dbg_bit_ready, dbg_bit_last;
    wire [3:0]  dbg_state;

    sonar_tx_top #(
        .MAX_LEN(MAX_LEN), .PHASE_STEP(PHASE_STEP),
        .CYCLES_PER_BIT(CYCLES_PER_BIT), .BIT1_NEGATIVE(0), .RAM_BASE(RAM_BASE)
    ) dut (
        .clk(clk), .resetn(resetn),
        .start(start), .payload_len(payload_len), .src_sel(src_sel),
        .ram_base_addr(ram_base_addr),
        .busy(busy), .done(done), .len_error(len_error),
        .bpsk_sample(bpsk_sample), .sample_valid(sample_valid),
        .in_valid(in_valid), .in_write_en(in_write_en), .in_addr(in_addr),
        .in_wdata(in_wdata), .in_wstrb(in_wstrb), .in_rdata(in_rdata),
        .in_ready(in_ready), .in_load(in_load),
        .rm_valid(rm_valid), .rm_addr(rm_addr), .rm_rdata(rm_rdata),
        .rm_ready(rm_ready), .rm_busy(rm_busy),
        .dbg_bit_valid(dbg_bit_valid), .dbg_bit_data(dbg_bit_data),
        .dbg_bit_ready(dbg_bit_ready), .dbg_bit_last(dbg_bit_last),
        .dbg_state(dbg_state)
    );

    sonar_ram_arbiter u_arb (
        .sel_sonar(rm_busy),
        .cpu_valid(cpu_valid), .cpu_addr(cpu_addr), .cpu_wdata(cpu_wdata),
        .cpu_wstrb(cpu_wstrb), .cpu_rdata(cpu_rdata), .cpu_ready(cpu_ready),
        .son_valid(rm_valid), .son_addr(rm_addr), .son_rdata(rm_rdata),
        .son_ready(rm_ready),
        .ram_valid(ram_valid), .ram_addr(ram_addr), .ram_wdata(ram_wdata),
        .ram_wstrb(ram_wstrb), .ram_rdata(ram_rdata), .ram_ready(ram_ready)
    );

    ram u_ram (
        .clk(clk), .resetn(resetn),
        .valid(ram_valid), .addr(ram_addr), .wdata(ram_wdata), .wstrb(ram_wstrb),
        .rdata(ram_rdata), .ready(ram_ready)
    );

    initial begin
        $dumpfile("tb_sonar_tx_top.vcd");
        $dumpvars(1, tb_sonar_tx_top);
        $dumpvars(1, tb_sonar_tx_top.dut);
    end

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        #200000000;
        $display("FAIL: simulation timeout");
        $display("====================================");
        $display("SONAR TX TOP TEST FAILED");
        $display("====================================");
        $finish;
    end

    //--------------------------------------------------------------
    // bookkeeping and monitors
    //--------------------------------------------------------------
    integer errors, tests;
    integer sc, bc, done_cnt, len_err_cnt, busy_cyc, rm_acc, sc_at_done;
    reg signed [7:0] smp [0:4095];
    reg              sbit [0:1023];
    reg              eb   [0:1023];
    reg [7:0]        tb_pay [0:63];
    reg [7:0]        sb [0:5];
    reg [7:0]        lv [0:3];
    reg prev_stall, prev_bd, prev_done, prev_le;

    always @(posedge clk) begin
        if (sample_valid === 1'b1) begin
            if (sc < 4096) smp[sc] = bpsk_sample;
            sc = sc + 1;
        end
        if (dbg_bit_valid && dbg_bit_ready) begin
            if (bc < 1024) sbit[bc] = dbg_bit_data;
            bc = bc + 1;
        end
        if (prev_stall) begin
            if (!(dbg_bit_valid && (dbg_bit_data === prev_bd))) begin
                errors = errors + 1;
                $display("FAIL: time=%0t serial bit changed while bit_ready was low", $time);
            end
        end
        prev_stall = dbg_bit_valid && !dbg_bit_ready && resetn;
        prev_bd    = dbg_bit_data;

        if (done) begin
            done_cnt   = done_cnt + 1;
            sc_at_done = sc;
            if (busy) begin errors = errors + 1; $display("FAIL: time=%0t busy high while done", $time); end
            if (sample_valid) begin errors = errors + 1; $display("FAIL: time=%0t sample_valid high while done", $time); end
            if (prev_done) begin errors = errors + 1; $display("FAIL: time=%0t done wider than one clock", $time); end
        end
        prev_done = done;

        if (len_error) begin
            len_err_cnt = len_err_cnt + 1;
            if (prev_le) begin errors = errors + 1; $display("FAIL: time=%0t len_error wider than one clock", $time); end
        end
        prev_le = len_error;

        if (busy) busy_cyc = busy_cyc + 1;
        if (rm_valid && rm_ready) rm_acc = rm_acc + 1;
    end

    task clear_counters;
        begin
            sc = 0; bc = 0; done_cnt = 0; len_err_cnt = 0;
            busy_cyc = 0; rm_acc = 0; sc_at_done = -1;
        end
    endtask

    task check;
        input [255:0] name;
        input [31:0]  expected;
        input [31:0]  actual;
        begin
            tests = tests + 1;
            if (expected !== actual) begin
                errors = errors + 1;
                $display("FAIL: time=%0t %0s expected=%0d actual=%0d", $time, name, expected, actual);
            end
        end
    endtask

    //--------------------------------------------------------------
    // Independent reference model
    //--------------------------------------------------------------
    // CRC-8: poly 0x07, init 0x00, no reflection, xorout 0x55 (XOR-then-shift form)
    function [7:0] exp_crc;
        input integer n;
        integer i, j;
        reg [7:0] c;
        begin
            c = 8'h00;
            for (i = 0; i < n; i = i + 1) begin
                c = c ^ tb_pay[i];
                for (j = 0; j < 8; j = j + 1) begin
                    if (c[7]) c = {c[6:0], 1'b0} ^ 8'h07;
                    else      c = {c[6:0], 1'b0};
                end
            end
            exp_crc = c ^ 8'h55;
        end
    endfunction

    // expected serial bits: payload bytes MSB first, then CRC MSB first
    task build_expected;
        input integer len;
        integer i;
        reg [7:0] t, c;
        begin
            for (i = 0; i < len * 8; i = i + 1) begin
                t = tb_pay[i / 8];
                eb[i] = t[7 - (i % 8)];
            end
            c = exp_crc(len);
            for (i = 0; i < 8; i = i + 1) eb[len * 8 + i] = c[7 - i];
        end
    endtask

    // expected output sample n: bit 1 -> +sine, bit 0 -> -sine, sine = top byte of
    // round(32767*sin()), saturated to +-127
    function integer exp_sample;
        input integer n;
        integer b, k, p, v16, sv, r;
        real a;
        begin
            b = n / SPB;
            k = n % SPB;
            p = (k * PHASE_STEP) % 256;
            a = 32767.0 * $sin(6.283185307179586 * p / 256.0);
            if (a >= 0.0) v16 = $rtoi(a + 0.5);
            else          v16 = $rtoi(a - 0.5);
            sv = v16 >>> 8;                       // arithmetic shift = top byte
            if (eb[b]) r = sv; else r = -sv;
            if (r > 127)  r = 127;
            if (r < -127) r = -127;
            exp_sample = r;
        end
    endfunction

    //--------------------------------------------------------------
    // Bus tasks
    //--------------------------------------------------------------
    task inreg_write;
        input [31:0] a;
        input [31:0] d;
        begin
            in_valid = 1'b1; in_write_en = 1'b1;
            in_addr = a; in_wdata = d; in_wstrb = 4'hF;
            @(posedge clk); #1;
            while (!in_ready) begin @(posedge clk); #1; end
            in_valid = 1'b0; in_write_en = 1'b0;
            in_addr = 32'b0; in_wdata = 32'b0; in_wstrb = 4'h0;
            @(posedge clk); #1;
        end
    endtask

    task cpu_wr;
        input [31:0] a;
        input [31:0] d;
        input [3:0]  s;
        begin
            cpu_valid = 1'b1; cpu_addr = a; cpu_wdata = d; cpu_wstrb = s;
            @(posedge clk); #1;
            while (!cpu_ready) begin @(posedge clk); #1; end
            cpu_valid = 1'b0; cpu_wstrb = 4'h0; cpu_addr = 32'b0; cpu_wdata = 32'b0;
            @(posedge clk); #1;
        end
    endtask

    task cpu_rd;
        input  [31:0] a;
        output [31:0] d;
        begin
            cpu_valid = 1'b1; cpu_addr = a; cpu_wdata = 32'b0; cpu_wstrb = 4'h0;
            @(posedge clk); #1;
            while (!cpu_ready) begin @(posedge clk); #1; end
            d = cpu_rdata;
            cpu_valid = 1'b0; cpu_addr = 32'b0;
            @(posedge clk); #1;
        end
    endtask

    // write tb_pay[0..n-1] into RAM at byte address base, one byte lane at a time
    task load_ram;
        input integer base;
        input integer n;
        integer i, ba, lane;
        begin
            for (i = 0; i < n; i = i + 1) begin
                ba   = base + i;
                lane = ba % 4;
                cpu_wr(RAM_BASE + ((ba / 4) * 4),
                       ({24'b0, tb_pay[i]} << (8 * lane)),
                       (4'b0001 << lane));
            end
        end
    endtask

    task set_str9;
        begin
            tb_pay[0]="1"; tb_pay[1]="2"; tb_pay[2]="3"; tb_pay[3]="4"; tb_pay[4]="5";
            tb_pay[5]="6"; tb_pay[6]="7"; tb_pay[7]="8"; tb_pay[8]="9";
        end
    endtask

    //--------------------------------------------------------------
    // One complete transmission and all checks
    //--------------------------------------------------------------
    task run_tx;
        input [255:0] name;
        input integer len;
        input integer srcsel;
        input integer base;
        input integer poke;
        integer t, k, errs0, exp_bits_n, exp_samples, e, d, diff;
        begin
            errs0 = errors;
            build_expected(len);
            exp_bits_n  = (len + 1) * 8;
            exp_samples = exp_bits_n * SPB;
            clear_counters;

            @(posedge clk); #1;
            payload_len = len; src_sel = srcsel; ram_base_addr = base; start = 1'b1;
            @(posedge clk); #1;
            start = 1'b0;
            check("busy high after start", 1, busy);

            t = 0;
            while (done_cnt == 0 && t < TIMEOUT_CYC) begin
                @(posedge clk); #1;
                t = t + 1;
                if (poke && t == 20) begin          // start while busy must be ignored
                    start = 1'b1;
                    @(posedge clk); #1;
                    start = 1'b0;
                end
            end
            repeat (12) @(posedge clk);
            #1;

            check("done seen exactly once",       1, done_cnt);
            check("samples at done = expected",   exp_samples, sc_at_done);
            check("total samples",                exp_samples, sc);
            check("serial bits (payload+CRC)",    exp_bits_n, bc);
            check("busy low after done",          0, busy);
            check("controller back in IDLE",      0, dbg_state);
            check("no len_error",                 0, len_err_cnt);
            check("busy was asserted",            1, (busy_cyc > 0));
            check("RAM released after transfer",  0, rm_busy);
            if (srcsel == 1) check("RAM reads = payload length", len, rm_acc);
            else             check("no RAM reads for input reg",   0, rm_acc);

            for (k = 0; k < exp_bits_n; k = k + 1) begin
                tests = tests + 1;
                if (sbit[k] !== eb[k]) begin
                    errors = errors + 1;
                    $display("FAIL: %0s bit %0d expected=%b actual=%b", name, k, eb[k], sbit[k]);
                end
            end
            for (k = 0; k < exp_samples; k = k + 1) begin
                e = exp_sample(k);
                d = $signed(smp[k]);
                diff = d - e;
                if (diff < 0) diff = -diff;
                tests = tests + 1;
                if (diff > 1) begin
                    errors = errors + 1;
                    if (errors < 30)
                        $display("FAIL: %0s sample %0d expected=%0d actual=%0d", name, k, e, d);
                end
            end

            if (errors == errs0) $display("PASS: %0s  len=%0d crc=0x%02h samples=%0d", name, len, exp_crc(len), exp_samples);
            else                 $display("FAIL: %0s had errors", name);
            repeat (3) @(posedge clk);
            #1;
        end
    endtask

    // start that must be rejected
    task bad_start;
        input [255:0] name;
        input integer len;
        input integer srcsel;
        input integer base;
        integer errs0;
        begin
            errs0 = errors;
            clear_counters;
            @(posedge clk); #1;
            payload_len = len; src_sel = srcsel; ram_base_addr = base; start = 1'b1;
            @(posedge clk); #1;
            start = 1'b0;
            repeat (30) @(posedge clk);
            #1;
            check("rejected: len_error pulsed once", 1, len_err_cnt);
            check("rejected: busy never asserted",   0, busy_cyc);
            check("rejected: no samples",            0, sc);
            check("rejected: no serial bits",        0, bc);
            check("rejected: no RAM access",         0, rm_acc);
            check("rejected: no done",               0, done_cnt);
            check("rejected: controller IDLE",       0, dbg_state);
            if (errors == errs0) $display("PASS: rejected %0s", name);
            else                 $display("FAIL: rejected %0s", name);
        end
    endtask

    // reset during a transmission
    task reset_mid;
        input [255:0] name;
        input integer len;
        input integer srcsel;
        input integer base;
        input integer wcyc;
        input integer until_sc;
        integer t, errs0, snap;
        begin
            errs0 = errors;
            build_expected(len);
            clear_counters;
            @(posedge clk); #1;
            payload_len = len; src_sel = srcsel; ram_base_addr = base; start = 1'b1;
            @(posedge clk); #1;
            start = 1'b0;
            if (until_sc > 0) begin
                t = 0;
                while (sc < until_sc && t < TIMEOUT_CYC) begin @(posedge clk); #1; t = t + 1; end
            end else begin
                repeat (wcyc) @(posedge clk);
                #1;
            end
            check("reset test: busy before reset", 1, busy);
            resetn = 1'b0;
            @(posedge clk); #1;
            @(posedge clk); #1;
            resetn = 1'b1;
            check("after reset: busy low",         0, busy);
            check("after reset: done low",         0, done);
            check("after reset: sample_valid low", 0, sample_valid);
            check("after reset: controller IDLE",  0, dbg_state);
            check("after reset: RAM released",     0, rm_busy);
            check("after reset: no RAM request",   0, rm_valid);
            check("after reset: bit_valid low",    0, dbg_bit_valid);
            snap = sc;
            repeat (80) @(posedge clk);
            #1;
            check("after reset: no more samples",  snap, sc);
            check("after reset: no done pulse",    0, done_cnt);
            check("after reset: still IDLE",       0, dbg_state);
            if (errors == errs0) $display("PASS: %0s", name);
            else                 $display("FAIL: %0s", name);
        end
    endtask

    integer i, base_i;
    reg [31:0] rd;

    //--------------------------------------------------------------
    // main sequence
    //--------------------------------------------------------------
    initial begin
        errors = 0; tests = 0;
        clear_counters;
        prev_stall = 1'b0; prev_bd = 1'b0; prev_done = 1'b0; prev_le = 1'b0;
        start = 1'b0; payload_len = 8'd0; src_sel = 1'b0; ram_base_addr = 16'd0;
        in_valid = 1'b0; in_write_en = 1'b0; in_addr = 32'b0; in_wdata = 32'b0; in_wstrb = 4'h0;
        cpu_valid = 1'b0; cpu_addr = 32'b0; cpu_wdata = 32'b0; cpu_wstrb = 4'h0;
        for (i = 0; i < 64; i = i + 1) tb_pay[i] = 8'h00;
        sb[0] = 8'hB2; sb[1] = 8'h00; sb[2] = 8'hFF; sb[3] = 8'h55; sb[4] = 8'h80; sb[5] = 8'h01;
        lv[0] = 8'hA5; lv[1] = 8'h5A; lv[2] = 8'hC3; lv[3] = 8'h3C;

        resetn = 1'b0;
        repeat (4) @(posedge clk);
        #1;

        // 1. reset and idle state
        $display("--- 1: reset / idle ---");
        check("reset: busy low",        0, busy);
        check("reset: done low",        0, done);
        check("reset: len_error low",   0, len_error);
        check("reset: sample_valid low",0, sample_valid);
        check("reset: bpsk_sample zero",0, {24'b0, bpsk_sample});
        check("reset: rm_busy low",     0, rm_busy);
        check("reset: rm_valid low",    0, rm_valid);
        check("reset: controller IDLE", 0, dbg_state);
        resetn = 1'b1;
        @(posedge clk); #1;

        // 2. nothing happens without start
        $display("--- 2: no transmission without start ---");
        payload_len = 8'd5; src_sel = 1'b1;
        clear_counters;
        repeat (40) begin
            @(posedge clk); #1;
            check("idle: busy low",         0, busy);
            check("idle: sample_valid low", 0, sample_valid);
            check("idle: bit_valid low",    0, dbg_bit_valid);
            check("idle: rm_valid low",     0, rm_valid);
            check("idle: controller IDLE",  0, dbg_state);
        end
        check("idle: no samples", 0, sc);
        check("idle: no bits",    0, bc);
        check("idle: no done",    0, done_cnt);

        // 3. input register source (single byte)
        $display("--- 3: input register source ---");
        for (i = 0; i < 6; i = i + 1) begin
            tb_pay[0] = sb[i];
            inreg_write(32'h00, {24'b0, sb[i]});
            inreg_write(32'h04, 32'h1);
            run_tx("input reg byte", 1, 0, 0, 0);
        end

        // 4. RAM source, one byte in each byte lane
        $display("--- 4: RAM source, byte lanes ---");
        for (i = 0; i < 4; i = i + 1) begin
            tb_pay[0] = lv[i];
            load_ram(i, 1);
            run_tx("RAM 1 byte (lane test)", 1, 1, i, 0);
        end

        // 5. "123456789" -> CRC 0xA1
        $display("--- 5: ASCII 123456789 ---");
        set_str9;
        check("expected CRC of 123456789 is 0xA1", 32'hA1, exp_crc(9));
        load_ram(0, 9);
        run_tx("123456789 aligned", 9, 1, 0, 0);
        cpu_rd(RAM_BASE, rd);
        check("CPU read word0 after tx (RAM released)", 32'h34333231, rd);
        cpu_rd(RAM_BASE + 32'h4, rd);
        check("CPU read word1 after tx", 32'h38373635, rd);
        for (base_i = 1; base_i < 4; base_i = base_i + 1) begin
            load_ram(base_i, 9);
            run_tx("123456789 unaligned", 9, 1, base_i, 0);
        end

        // 6. maximum length
        $display("--- 6: maximum length %0d ---", MAX_LEN);
        for (i = 0; i < MAX_LEN; i = i + 1) tb_pay[i] = (i * 37 + 11) % 256;
        load_ram(5, MAX_LEN);
        run_tx("max length", MAX_LEN, 1, 5, 0);

        // 7. end of RAM (bytes up to 0x0100FFFF)
        $display("--- 7: end of RAM ---");
        for (i = 0; i < 16; i = i + 1) tb_pay[i] = (i * 29 + 3) % 256;
        load_ram(16'hFFF0, 16);
        run_tx("RAM end boundary", 16, 1, 16'hFFF0, 0);
        tb_pay[0] = 8'h9D;
        load_ram(16'hFFFF, 1);
        run_tx("last RAM byte", 1, 1, 16'hFFFF, 0);

        // 8. start while busy is ignored
        $display("--- 8: start while busy ---");
        set_str9;
        load_ram(0, 9);
        run_tx("start ignored while busy", 9, 1, 0, 1);

        // 9. invalid lengths / addresses
        $display("--- 9: invalid requests ---");
        bad_start("reg source len 0",       0,   0, 0);
        bad_start("reg source len 2",       2,   0, 0);
        bad_start("RAM len 0",              0,   1, 0);
        bad_start("RAM len MAX_LEN+1",      MAX_LEN + 1, 1, 0);
        bad_start("RAM len 255",            255, 1, 0);
        bad_start("RAM past end (FFFF+2)",  2,   1, 16'hFFFF);
        bad_start("RAM past end (FFF1+16)", 16,  1, 16'hFFF1);

        // 10. reset during operation
        $display("--- 10: reset during operation ---");
        set_str9;
        load_ram(0, 9);
        reset_mid("reset during payload/CRC phase", 9, 1, 0, 12, 0);
        cpu_rd(RAM_BASE, rd);
        check("CPU can read RAM after reset", 32'h34333231, rd);
        reset_mid("reset during waveform", 9, 1, 0, 0, 40);

        // 11. recovery and back-to-back
        $display("--- 11: recovery / back-to-back ---");
        run_tx("after reset", 9, 1, 0, 0);
        tb_pay[0] = 8'h3C;
        inreg_write(32'h00, 32'h3C);
        inreg_write(32'h04, 32'h1);
        run_tx("input reg after reset", 1, 0, 0, 0);
        run_tx("back-to-back RAM", 9, 1, 0, 0);

        $display("-----------------------------");
        $display("Checks run: %0d, errors: %0d", tests, errors);
        if (errors == 0) begin
            $display("====================================");
            $display("SONAR TX TOP TEST PASSED");
            $display("====================================");
        end else begin
            $display("====================================");
            $display("SONAR TX TOP TEST FAILED");
            $display("====================================");
        end
        $finish;
    end

endmodule