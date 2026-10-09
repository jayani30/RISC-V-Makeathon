`timescale 1ns / 1ps
module tb_bpsk_sample_generator;

    reg        clk;
    reg        resetn;
    reg        enable;
    reg        bpsk_bit;
    reg signed [7:0] sine_sample;

    wire signed [7:0] out_u, out_s;      // unsigned-mode / signed-mode DUTs
    wire              valid_u, valid_s;

    integer errors;
    integer tests;
    integer seed;
    integer i, k;
    reg     verbose;
    reg [7:0] vals [0:5];
    reg [7:0] rs;
    reg       rb;
    reg signed [7:0] held;

    bpsk_sample_generator #(.SINE_SIGNED(0)) dut_u (
        .clk(clk), .resetn(resetn), .enable(enable), .bpsk_bit(bpsk_bit),
        .sine_sample(sine_sample), .bpsk_sample(out_u), .sample_valid(valid_u));

    bpsk_sample_generator #(.SINE_SIGNED(1)) dut_s (
        .clk(clk), .resetn(resetn), .enable(enable), .bpsk_bit(bpsk_bit),
        .sine_sample(sine_sample), .bpsk_sample(out_s), .sample_valid(valid_s));

    initial begin
        $dumpfile("tb_bpsk_sample_generator.vcd");
        $dumpvars(0, tb_bpsk_sample_generator);
    end

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        #2000000;
        $display("FAIL: simulation timeout");
        $display("====================================");
        $display("BPSK SAMPLE GENERATOR TEST FAILED");
        $display("====================================");
        $finish;
    end

    //--------------------------------------------------------------
    // Independent reference models (integer arithmetic, no shared code)
    //--------------------------------------------------------------
    function integer exp_u;               // unsigned magnitude, clamp to 127
        input b;
        input [7:0] s;
        integer m;
        begin
            m = s;
            if (m > 127) m = 127;
            if (b) exp_u = m; else exp_u = -m;
        end
    endfunction

    function integer exp_s;               // two's-complement sine, saturate +-127
        input b;
        input [7:0] s;
        integer sv, r;
        begin
            sv = s;
            if (sv >= 128) sv = sv - 256;
            if (b) r = sv; else r = -sv;
            if (r > 127)  r = 127;
            if (r < -127) r = -127;
            exp_s = r;
        end
    endfunction

    task check_val;
        input [255:0] name;
        input integer expected;
        input integer actual;
        begin
            tests = tests + 1;
            if (expected == actual) begin
                if (verbose)
                    $display("PASS: %0s  expected=%0d actual=%0d", name, expected, actual);
            end else begin
                $display("FAIL: time=%0t %0s  expected=%0d actual=%0d", $time, name, expected, actual);
                errors = errors + 1;
            end
        end
    endtask

    // Drive inputs 1 ns after a clock edge, wait for the next edge, then settle
    task drive_one;
        input b;
        input [7:0] s;
        begin
            bpsk_bit = b; sine_sample = s; enable = 1'b1;
            @(posedge clk); #1;
        end
    endtask

    // Directed test: both modes must give expv (inputs are 0..127)
    task directed;
        input [255:0] name;
        input b;
        input [7:0] s;
        input integer expv;
        begin
            drive_one(b, s);
            check_val(name,                expv, out_u);
            check_val("signed mode, same", expv, out_s);
            check_val("sample_valid (u)",  1, valid_u);
            check_val("sample_valid (s)",  1, valid_s);
        end
    endtask

    // Model-based test for any input
    task auto_check;
        input b;
        input [7:0] s;
        begin
            drive_one(b, s);
            check_val("unsigned-mode sample", exp_u(b, s), out_u);
            check_val("signed-mode sample",   exp_s(b, s), out_s);
            check_val("sample_valid (u)",     1, valid_u);
            check_val("sample_valid (s)",     1, valid_s);
        end
    endtask

    initial begin
        errors = 0; tests = 0; seed = 1; verbose = 1'b1;
        enable = 1'b0; bpsk_bit = 1'b0; sine_sample = 8'd0;
        vals[0] = 8'd0;   vals[1] = 8'd1;   vals[2] = 8'd50;
        vals[3] = 8'd100; vals[4] = 8'd126; vals[5] = 8'd127;
        resetn = 1'b0;
        repeat (3) @(posedge clk);
        #1;

        $display("--- reset state ---");
        check_val("out_u after reset",   0, out_u);
        check_val("out_s after reset",   0, out_s);
        check_val("valid_u after reset", 0, valid_u);
        check_val("valid_s after reset", 0, valid_s);
        resetn = 1'b1;
        @(posedge clk); #1;
        check_val("idle: valid_u low",   0, valid_u);
        check_val("idle: out_u still 0", 0, out_u);

        $display("--- directed tests 1..6 ---");
        directed("TEST1 bit=1 s=100", 1'b1, 8'd100,  100);
        directed("TEST2 bit=0 s=100", 1'b0, 8'd100, -100);
        directed("TEST3 bit=1 s=0",   1'b1, 8'd0,      0);
        directed("TEST4 bit=0 s=0",   1'b0, 8'd0,      0);
        directed("TEST5 bit=1 s=127", 1'b1, 8'd127,  127);
        directed("TEST6 bit=0 s=127", 1'b0, 8'd127, -127);

        $display("--- alternating bits 1,0,1,0,1,0 ---");
        for (i = 0; i < 6; i = i + 1)
            for (k = 0; k < 6; k = k + 1)
                auto_check((k % 2) == 0, vals[i]);

        $display("--- clamp / signed-mode corner cases ---");
        auto_check(1'b1, 8'd200);
        auto_check(1'b0, 8'd200);
        auto_check(1'b1, 8'd255);
        auto_check(1'b0, 8'd128);
        drive_one(1'b1, 8'h80);                       // -128 in signed mode
        check_val("signed -128, bit=1 -> -127", -127, out_s);
        drive_one(1'b0, 8'h80);
        check_val("signed -128, bit=0 -> +127",  127, out_s);
        drive_one(1'b1, 8'hFF);
        check_val("signed -1, bit=1 -> -1",       -1, out_s);
        drive_one(1'b0, 8'hFF);
        check_val("signed -1, bit=0 -> +1",        1, out_s);
        drive_one(1'b0, 8'h9C);                       // -100 in signed mode
        check_val("signed -100, bit=0 -> +100",  100, out_s);
        drive_one(1'b1, 8'h9C);
        check_val("signed -100, bit=1 -> -100", -100, out_s);

        $display("--- enable = 0 ---");
        drive_one(1'b1, 8'd100);
        held = out_u;
        enable = 1'b0; bpsk_bit = 1'b0; sine_sample = 8'd50;
        @(posedge clk); #1;
        check_val("enable=0: valid_u low", 0, valid_u);
        check_val("enable=0: valid_s low", 0, valid_s);
        check_val("enable=0: sample holds", held, out_u);
        @(posedge clk); #1;
        check_val("enable=0 (2nd cycle): valid_u low", 0, valid_u);
        check_val("enable=0 (2nd cycle): sample holds", held, out_u);
        auto_check(1'b0, 8'd50);                       // resumes

        $display("--- reset during operation ---");
        drive_one(1'b1, 8'd100);
        resetn = 1'b0;                                  // enable still 1
        @(posedge clk); #1;
        check_val("mid reset: out_u cleared",   0, out_u);
        check_val("mid reset: valid_u cleared", 0, valid_u);
        check_val("mid reset: out_s cleared",   0, out_s);
        resetn = 1'b1;
        directed("after reset bit=0 s=100", 1'b0, 8'd100, -100);

        $display("--- 200 random samples (only failures printed) ---");
        verbose = 1'b0;
        for (i = 0; i < 200; i = i + 1) begin
            rs = $random(seed);
            rb = $random(seed);
            auto_check(rb, rs);
        end
        verbose = 1'b1;

        $display("-----------------------------");
        $display("Tests run: %0d, errors: %0d", tests, errors);
        if (errors == 0) begin
            $display("====================================");
            $display("BPSK SAMPLE GENERATOR TEST PASSED");
            $display("====================================");
        end else begin
            $display("====================================");
            $display("BPSK SAMPLE GENERATOR TEST FAILED");
            $display("====================================");
        end
        $finish;
    end
initial begin
    $dumpfile("tb_bpsk_sample_generator.vcd");
    $dumpvars(0, tb_bpsk_sample_generator);
end
endmodule