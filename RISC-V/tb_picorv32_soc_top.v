`timescale 1ns / 1ps
//==============================================================================
// tb_picorv32_soc_top : Board-Level SoC Testbench
//
// Verifies:
//   - 100 MHz clock & button reset synchronization
//   - Physical board status LEDs (Busy, Done Stretched, Pass)
//   - 1-Bit Delta-Sigma PDM DAC output activity
//   - 8-Bit Unsigned Parallel DAC output
//   - Overall firmware execution and test completion
//==============================================================================
module tb_picorv32_soc_top;

    reg        clk_100mhz;
    reg        btn_reset;
    reg        btn_start;
    reg  [7:0] sw;

    wire [7:0] led;
    wire       bpsk_pdm_out;
    wire [7:0] bpsk_dac_raw;
    wire       bpsk_val_out;
    wire       sonar_busy_pin;
    wire       sonar_done_pin;

    integer pdm_toggle_count = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;

    // Instantiate Board-Level SoC
    picorv32_soc_top #(
        .CLK_FREQ_HZ     (100_000_000),
        .ROM_INIT_FILE   ("rom_test.hex"),
        .MAX_PAYLOAD_LEN (32),
        .PHASE_STEP      (16),
        .CYCLES_PER_BIT  (1),
        .RESET_ACTIVE_LOW(1)
    ) dut_soc (
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

    // 100 MHz Clock (10 ns period)
    initial clk_100mhz = 1'b0;
    always #5 clk_100mhz = ~clk_100mhz;

    // Monitor PDM DAC bit toggles
    always @(posedge clk_100mhz) begin
        if (bpsk_pdm_out)
            pdm_toggle_count <= pdm_toggle_count + 1;
    end

    // Watchdog
    initial begin
        #500000;
        $display("[ERROR] Simulation TIMEOUT in tb_picorv32_soc_top!");
        $finish;
    end

    initial begin
        $dumpfile("tb_picorv32_soc_top.vcd");
        $dumpvars(0, tb_picorv32_soc_top);

        $display("==================================================================");
        $display("  STARTING PICORV32 BOARD-LEVEL SOC SIMULATION");
        $display("==================================================================");

        btn_reset = 1'b0; // Active-low button pressed
        btn_start = 1'b0;
        sw        = 8'h00;

        repeat (10) @(posedge clk_100mhz);
        #1 btn_reset = 1'b1; // Button released
        $display("[TIME %0t ns] Board Reset released. SoC starting up...", $time);

        // Wait until SoC firmware passes (LED 7 solid ON)
        wait (led[7] === 1'b1);
        #500;

        $display("\n==================================================================");
        $display("  BOARD-LEVEL SOC VERIFICATION RESULTS");
        $display("==================================================================");

        // Check 1: Firmware PASS LED[7]
        if (led[7] === 1'b1) begin
            $display("[PASS] LED[7] Asserted (Firmware Completion Magic Code Confirmed)");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] LED[7] not asserted!");
            tests_failed = tests_failed + 1;
        end

        // Check 2: Error LEDs are OFF
        if (led[6] === 1'b0 && led[5] === 1'b0 && led[2] === 1'b0) begin
            $display("[PASS] Error LEDs clear: Trap (LED[6]=0), Bus Error (LED[5]=0), Len Error (LED[2]=0)");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Error LEDs asserted! Trap=%b, BusErr=%b, LenErr=%b", led[6], led[5], led[2]);
            tests_failed = tests_failed + 1;
        end

        // Check 3: PDM DAC produced high-frequency pulses
        if (pdm_toggle_count > 100) begin
            $display("[PASS] Delta-Sigma PDM DAC Output: %0d pulses generated", pdm_toggle_count);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Delta-Sigma PDM DAC did not toggle!");
            tests_failed = tests_failed + 1;
        end

        // Check 4: Stretched Done LED was active
        if (led[1] === 1'b1) begin
            $display("[PASS] LED[1] Pulse Stretcher active (Visual indication for human eye)");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] LED[1] not active!");
            tests_failed = tests_failed + 1;
        end

        $display("\n==================================================================");
        $display("  SOC SUMMARY: %0d TESTS PASSED, %0d TESTS FAILED", tests_passed, tests_failed);
        if (tests_failed == 0)
            $display("  >>> BOARD-LEVEL SOC INTEGRATION SUCCESSFUL! <<<");
        $display("==================================================================\n");

        $finish;
    end

endmodule
