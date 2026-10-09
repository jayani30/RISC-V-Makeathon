`timescale 1ns / 1ps
module tb_dsin_lut;

    localparam integer PHASE_WIDTH  = 8;
    localparam integer OUTPUT_WIDTH = 16;
    localparam integer LUT_SIZE     = 256;
    localparam integer AMPLITUDE    = 32767;
    localparam integer TOL          = 2;      // allowed |expected - actual| (LSBs)

    reg                            clk;
    reg                            rst;
    reg  [PHASE_WIDTH-1:0]         phase;
    reg                            enable;
    wire signed [OUTPUT_WIDTH-1:0] sin_out;
    wire                           out_valid;

    integer error_count;
    integer check_count;

    dsin_lut #(
        .PHASE_WIDTH(PHASE_WIDTH), .OUTPUT_WIDTH(OUTPUT_WIDTH),
        .LUT_SIZE(LUT_SIZE), .AMPLITUDE(AMPLITUDE)
    ) dut (
        .clk(clk), .rst(rst), .phase(phase), .enable(enable),
        .sin_out(sin_out), .out_valid(out_valid)
    );

    initial begin
        $dumpfile("tb_dsin_lut.vcd");
        $dumpvars(0, tb_dsin_lut);
    end

    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        #5000000;
        $display("DSIN LUT TEST FAILED (timeout)");
        $finish;
    end

    // Reference model: real-number sine, testbench only
    function integer exp_sin;
        input integer p;
        real a;
        begin
            a = AMPLITUDE * $sin(6.283185307179586 * p / LUT_SIZE);
            if (a >= 0.0) exp_sin = $rtoi(a + 0.5);
            else          exp_sin = $rtoi(a - 0.5);
        end
    endfunction

    task report_err;
        input integer p;
        input integer expv;
        input integer actv;
        begin
            error_count = error_count + 1;
            $display("ERROR: time=%0t phase=%0d expected=%0d actual=%0d difference=%0d",
                     $time, p, expv, actv, actv - expv);
        end
    endtask

    // Apply phase + enable, wait one clock, check sin_out and out_valid
    task check_phase;
        input [PHASE_WIDTH-1:0] pv;
        integer expv, actv, diff;
        begin
            phase  = pv;
            enable = 1'b1;
            @(posedge clk); #1;
            expv = exp_sin(pv);
            actv = sin_out;                 // signed reg -> signed integer
            diff = actv - expv;
            if (diff < 0) diff = -diff;
            check_count = check_count + 1;
            if (diff > TOL) report_err(pv, expv, actv);
            if (out_valid !== 1'b1) begin
                error_count = error_count + 1;
                $display("ERROR: time=%0t phase=%0d out_valid expected=1 actual=%b", $time, pv, out_valid);
            end
        end
    endtask

    task check_zero_invalid;
        input [255:0] name;
        begin
            check_count = check_count + 1;
            if (sin_out !== 16'sd0 || out_valid !== 1'b0) begin
                error_count = error_count + 1;
                $display("ERROR: time=%0t %0s sin_out=%0d out_valid=%b (expected 0,0)",
                         $time, name, sin_out, out_valid);
            end
        end
    endtask

    integer i;
    integer first_cycle [0:LUT_SIZE-1];
    reg [PHASE_WIDTH-1:0] ph;
    integer seed;

    initial begin
        error_count = 0; check_count = 0; seed = 12345;
        rst = 1'b1; enable = 1'b0; phase = 0;
        repeat (3) @(posedge clk);
        #1;

        // Reset behaviour
        check_zero_invalid("after reset");
        rst = 1'b0;
        @(posedge clk); #1;
        check_zero_invalid("idle after reset (enable=0)");

        // Key phase points
        check_phase(8'd0);      // 0 deg
        check_phase(8'd64);     // 90 deg
        check_phase(8'd128);    // 180 deg
        check_phase(8'd192);    // 270 deg
        check_phase(8'd255);    // just before 360 deg

        // Wrap-around: 255 + 1 rolls over to 0, periodic behaviour
        ph = 8'd255;
        ph = ph + 8'd1;
        check_phase(ph);
        if (ph !== 8'd0) begin error_count = error_count + 1; $display("ERROR: wrap failed"); end
        for (i = 0; i < 8; i = i + 1)
            if (exp_sin(i) !== exp_sin(i + LUT_SIZE)) begin
                error_count = error_count + 1; $display("ERROR: reference not periodic at %0d", i);
            end

        // Intermediate values
        check_phase(8'd1);  check_phase(8'd10);  check_phase(8'd32);
        check_phase(8'd45); check_phase(8'd100); check_phase(8'd150);
        check_phase(8'd200); check_phase(8'd230); check_phase(8'd250);

        // Consecutive: full cycle, all 256 phases back to back
        for (i = 0; i < LUT_SIZE; i = i + 1) check_phase(i[PHASE_WIDTH-1:0]);

        // Random phases
        for (i = 0; i < 200; i = i + 1) check_phase($random(seed));

        // Symmetry check (independent of the reference): sin(p+128) = -sin(p)
        for (i = 0; i < 128; i = i + 1) begin
            check_phase(i[PHASE_WIDTH-1:0]);
            first_cycle[i] = sin_out;
            check_phase(i[PHASE_WIDTH-1:0] + 8'd128);
            check_count = check_count + 1;
            if ((first_cycle[i] + sin_out > 2) || (first_cycle[i] + sin_out < -2)) begin
                error_count = error_count + 1;
                $display("ERROR: time=%0t half-wave symmetry phase=%0d a=%0d b=%0d",
                         $time, i, first_cycle[i], sin_out);
            end
        end

        // enable = 0: output forced to 0 and out_valid = 0
        check_phase(8'd64);
        enable = 1'b0; phase = 8'd64;
        @(posedge clk); #1;
        check_zero_invalid("enable=0");
        phase = 8'd192;
        @(posedge clk); #1;
        check_zero_invalid("enable=0, phase changed");
        check_phase(8'd64);          // resumes normally

        // Reset in the middle of operation
        rst = 1'b1;
        @(posedge clk); #1;
        check_zero_invalid("mid-operation reset");
        rst = 1'b0;
        check_phase(8'd64);          // normal operation resumes

        $display("Checks run: %0d", check_count);
        if (error_count == 0) $display("DSIN LUT TEST PASSED");
        else begin
            $display("DSIN LUT TEST FAILED");
            $display("Total errors: %0d", error_count);
        end
        $finish;
    end
    initial begin
        $dumpfile("tb_dsin_lut.vcd");
        $dumpvars(0, tb_dsin_lut);
    end
endmodule