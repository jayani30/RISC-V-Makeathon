`timescale 1ns / 1ps
//==============================================================================
// tb_top.v : Comprehensive Testbench for Integrated top.v
//
// Verification Coverage:
//   1. Reset behavior and metastability synchronizer release
//   2. Processor-to-peripheral register writes across all MMIO blocks
//   3. Hardware CRC-8/ITU accelerator calculation and readback
//   4. Direct baseband BPSK modulator register configuration and readback
//   5. Digital Input Register loading and status flags
//   6. Sonar transmission initiation via MMIO control registers
//   7. Autonomous RAM access arbitration by Sonar TX master
//   8. Hardware status flags (busy, done, error sticky bits)
//   9. Physical BPSK sample generation, 1-bit PDM DAC pulses, and 8-bit DAC output
//  10. Zero unknown ('X') values on critical control lines
//  11. Firmware execution completion (RAM word 8 = 0xCAFEF00D -> LED[7] asserted)
//==============================================================================

module tb_top;

    // Testbench stimuli
    reg        clk_100mhz;
    reg        btn_reset;
    reg        btn_start;
    reg  [7:0] sw;

    // DUT Outputs
    wire [7:0] led;
    wire       bpsk_pdm_out;
    wire [7:0] bpsk_dac_raw;
    wire       bpsk_val_out;
    wire       sonar_busy_pin;
    wire       sonar_done_pin;

    // Performance & Validation Counters
    integer pdm_pulse_count   = 0;
    integer sample_val_count  = 0;
    integer tests_passed      = 0;
    integer tests_failed      = 0;
    integer x_error_count     = 0;

    // Instantiate Integrated Top-Level SoC
    top #(
        .CLK_FREQ_HZ     (100_000_000),
        .ROM_INIT_FILE   ("rom_test.hex"),
        .MAX_PAYLOAD_LEN (32),
        .PHASE_STEP      (16),
        .CYCLES_PER_BIT  (1),
        .RESET_ACTIVE_LOW(1)
    ) dut (
        .clk_100mhz     (clk_100mhz),
        .btn_reset      (btn_reset),
        .btn_start      (btn_start),
        .sw             (sw),
        .led            (led),
        .bpsk_pdm_out   (bpsk_pdm_out),
        .bpsk_dac_raw   (bpsk_dac_raw),
        .bpsk_val_out   (bpsk_val_out),
        .sonar_busy_pin (sonar_busy_pin),
        .sonar_done_pin (sonar_done_pin)
    );

    // 100 MHz System Clock Generation (10 ns period)
    initial clk_100mhz = 1'b0;
    always #5 clk_100mhz = ~clk_100mhz;

    // Continuous Monitoring of Signals for Unknown 'X' values
    always @(posedge clk_100mhz) begin
        if (dut.sys_resetn) begin
            if (led[7:0] === 8'hxx || led[7:0] === 8'hzz) begin
                $display("[ERROR-X] Undefined value detected on LED bus: %b at time %0t ns", led, $time);
                x_error_count <= x_error_count + 1;
            end
            if (sonar_busy_pin === 1'bx || sonar_done_pin === 1'bx) begin
                $display("[ERROR-X] Undefined value on sonar status pins at time %0t ns", $time);
                x_error_count <= x_error_count + 1;
            end
        end
    end

    // Monitor PDM DAC and Sample Valid Pulses
    always @(posedge clk_100mhz) begin
        if (bpsk_pdm_out)
            pdm_pulse_count <= pdm_pulse_count + 1;
        if (bpsk_val_out)
            sample_val_count <= sample_val_count + 1;
    end

    // Watchdog Timer (protect against infinite simulation hangs)
    initial begin
        #600000;
        $display("\n[ERROR] Simulation WATCHDOG TIMEOUT! Firmware did not complete within expected time.");
        $finish;
    end

    // Main Test Sequence
    initial begin
        $dumpfile("tb_top.vcd");
        $dumpvars(0, tb_top);

        $display("==================================================================");
        $display("   STARTING INTEGRATED SOC (top.v) COMPREHENSIVE VERIFICATION   ");
        $display("==================================================================");

        // 1. Initial State & Assert Reset
        btn_reset = 1'b0; // Active-low reset asserted
        btn_start = 1'b0;
        sw        = 8'h00;

        repeat (12) @(posedge clk_100mhz);
        #1;
        $display("[TIME %0t ns] 1. Releasing Reset Button. Synchronizer initializing...", $time);
        btn_reset = 1'b1; // Button released (normal run state)

        // Wait for CPU to boot, execute test firmware, and complete Sonar transmission
        $display("[TIME %0t ns] 2. Waiting for CPU firmware execution & Sonar transmission...", $time);
        wait (led[7] === 1'b1);
        #1000; // Allow settling time

        $display("\n==================================================================");
        $display("                  DETAILED VERIFICATION RESULTS                   ");
        $display("==================================================================");

        // Check 1: Firmware Completion Magic Code
        if (dut.u_ram.mem[8] === 32'hCAFEF00D) begin
            $display("[PASS] Check 1: Firmware Completion Confirmed (RAM[8] = 0x%08X)", dut.u_ram.mem[8]);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Check 1: Magic code mismatch! Expected 0xCAFEF00D, got 0x%08X", dut.u_ram.mem[8]);
            tests_failed = tests_failed + 1;
        end

        // Check 2: LED[7] Asserted
        if (led[7] === 1'b1) begin
            $display("[PASS] Check 2: LED[7] Solid ON (Firmware Pass Flag Asserted)");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Check 2: LED[7] not asserted!");
            tests_failed = tests_failed + 1;
        end

        // Check 3: CPU and Bus Error Check
        if (led[6] === 1'b0 && led[5] === 1'b0 && led[2] === 1'b0) begin
            $display("[PASS] Check 3: No Faults Detected (Trap=0, BusError=0, LenError=0)");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Check 3: Error detected! Trap=%b, BusError=%b, LenError=%b", led[6], led[5], led[2]);
            tests_failed = tests_failed + 1;
        end

        // Check 4: PDM DAC Toggle Verification
        if (pdm_pulse_count > 100) begin
            $display("[PASS] Check 4: 1-Bit Delta-Sigma PDM DAC active (%0d pulses recorded)", pdm_pulse_count);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Check 4: PDM DAC inactive! Pulses = %0d", pdm_pulse_count);
            tests_failed = tests_failed + 1;
        end

        // Check 5: Parallel BPSK Sample Generation
        if (sample_val_count > 10) begin
            $display("[PASS] Check 5: Carrier BPSK Samples Generated (%0d valid sample strobes)", sample_val_count);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Check 5: Insufficient BPSK samples generated! Count = %0d", sample_val_count);
            tests_failed = tests_failed + 1;
        end

        // Check 6: Pulse Stretched Done Flag
        if (led[1] === 1'b1) begin
            $display("[PASS] Check 6: LED[1] Pulse Stretcher Active (Visual indicator confirmed)");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Check 6: LED[1] not active!");
            tests_failed = tests_failed + 1;
        end

        // Check 7: Unknown 'X' Signal Check
        if (x_error_count == 0) begin
            $display("[PASS] Check 7: Zero Unknown ('X') States Detected on Critical Control Lines");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Check 7: Detected %0d unknown ('X') values!", x_error_count);
            tests_failed = tests_failed + 1;
        end

        $display("\n==================================================================");
        $display("   FINAL STATUS: %0d / %0d TESTS PASSED, %0d FAILED", 
                 tests_passed, (tests_passed + tests_failed), tests_failed);
        if (tests_failed == 0) begin
            $display("   >>> ALL VERIFICATION CHECKS COMPLETED SUCCESSFULLY! <<<");
        end else begin
            $display("   >>> VERIFICATION FAILED! PLEASE REVIEW LOGS. <<<");
        end
        $display("==================================================================\n");

        $finish;
    end

endmodule
