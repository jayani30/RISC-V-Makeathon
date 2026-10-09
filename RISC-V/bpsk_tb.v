`timescale 1ns / 1ps
module tb_bpsk_modulator;

    localparam integer OUTPUT_WIDTH = 16;
    localparam integer AMPLITUDE    = 1000;

    reg                            clk;
    reg                            rst;
    reg                            data_in;
    reg                            data_valid;
    reg                            sample_en;
    wire signed [OUTPUT_WIDTH-1:0] bpsk_out;
    wire                           out_valid;

    integer errors;
    integer tests;
    reg     exp_bit;      // testbench's own model of the latched bit

    bpsk_modulator #(
        .OUTPUT_WIDTH(OUTPUT_WIDTH),
        .AMPLITUDE(AMPLITUDE)
    ) dut (
        .clk(clk), .rst(rst),
        .data_in(data_in), .data_valid(data_valid), .sample_en(sample_en),
        .bpsk_out(bpsk_out), .out_valid(out_valid)
    );

    // VCD dump
    initial begin
        $dumpfile("tb_bpsk_modulator.vcd");
        $dumpvars(0, tb_bpsk_modulator);
    end

    // 100 MHz clock
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // watchdog
    initial begin
        #1000000;
        $display("FAIL: simulation timeout");
        $display("TEST FAILED");
        $finish;
    end

    //--------------------------------------------------------------
    // Reference: expected sample for a bit (independent of the DUT)
    //--------------------------------------------------------------
    function integer exp_sample;
        input b;
        begin
            if (b) exp_sample = -AMPLITUDE;
            else   exp_sample =  AMPLITUDE;
        end
    endfunction

    task check_val;
        input [255:0] name;
        input integer expected;
        input integer actual;
        begin
            tests = tests + 1;
            if (expected == actual)
                $display("PASS: %0s  expected=%0d actual=%0d", name, expected, actual);
            else begin
                $display("FAIL: %0s  expected=%0d actual=%0d", name, expected, actual);
                errors = errors + 1;
            end
        end
    endtask

    // Latch one bit (data_in then changes, to prove it is only latched once)
    task send_bit;
        input b;
        begin
            data_in = b; data_valid = 1'b1;
            @(posedge clk); #1;
            data_valid = 1'b0;
            data_in = ~b;
            exp_bit = b;
        end
    endtask

    // Request one sample and check output, then check hold behaviour
    task take_sample;
        input [255:0] name;
        begin
            sample_en = 1'b1;
            @(posedge clk); #1;
            sample_en = 1'b0;
            check_val("out_valid high", 1, out_valid);
            check_val(name, exp_sample(exp_bit), $signed(bpsk_out));
            @(posedge clk); #1;
            check_val("out_valid low next cycle", 0, out_valid);
            check_val("bpsk_out holds", exp_sample(exp_bit), $signed(bpsk_out));
        end
    endtask

    integer i;
    reg [7:0] pattern;

    initial begin
        errors = 0; tests = 0; exp_bit = 1'b0;
        rst = 1'b1; data_in = 1'b0; data_valid = 1'b0; sample_en = 1'b0;
        repeat (3) @(posedge clk);
        #1;

        // 1. Reset operation
        $display("--- 1: reset ---");
        check_val("bpsk_out after reset", 0, $signed(bpsk_out));
        check_val("out_valid after reset", 0, out_valid);
        rst = 1'b0;
        @(posedge clk); #1;
        check_val("idle: out_valid stays 0", 0, out_valid);
        check_val("idle: bpsk_out stays 0", 0, $signed(bpsk_out));

        // 2. Input bit 0
        $display("--- 2: bit 0 ---");
        send_bit(1'b0);
        take_sample("bit 0 -> +AMPLITUDE");

        // 3. Input bit 1
        $display("--- 3: bit 1 ---");
        send_bit(1'b1);
        take_sample("bit 1 -> -AMPLITUDE");

        // 4. Multiple consecutive bits, one sample each
        $display("--- 4: consecutive bits ---");
        pattern = 8'b1011_0010;
        for (i = 7; i >= 0; i = i - 1) begin
            send_bit(pattern[i]);
            take_sample("pattern bit");
        end

        // 5. Several samples per bit (carrier-like), same bit held
        $display("--- 5: 4 samples per bit ---");
        send_bit(1'b1);
        for (i = 0; i < 4; i = i + 1) take_sample("repeat sample, bit 1");
        send_bit(1'b0);
        for (i = 0; i < 4; i = i + 1) take_sample("repeat sample, bit 0");

        // 6. No sample_en -> no output; data_valid low -> data_in ignored
        $display("--- 6: no sample / ignore data_in ---");
        send_bit(1'b1);
        data_in = 1'b0;                    // data_valid is 0, must be ignored
        repeat (3) begin
            @(posedge clk); #1;
            check_val("no sample_en: out_valid 0", 0, out_valid);
        end
        take_sample("latched bit 1 survives data_in change");

        // 7. Reset in the middle of operation
        $display("--- 7: reset during operation ---");
        rst = 1'b1;
        @(posedge clk); #1;
        rst = 1'b0;
        exp_bit = 1'b0;                    // reset clears latched bit
        check_val("bpsk_out cleared by reset", 0, $signed(bpsk_out));
        check_val("out_valid cleared by reset", 0, out_valid);
        take_sample("first sample after reset is +AMPLITUDE");

        $display("-----------------------------");
        $display("Tests run: %0d, errors: %0d", tests, errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $finish;
    end
    // VCD dump for GTKWave
       initial begin
        $dumpfile("tb_bpsk_modulator.vcd");
        $dumpvars(0, tb_bpsk_modulator);
    
    end
endmodule