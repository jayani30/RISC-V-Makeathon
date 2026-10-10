`timescale 1ns / 1ps
//==============================================================================
// tb_picorv32_top : Testbench for PicoRV32 SoC with CRC-8 and BPSK Subsystems
//
// Verifies:
//   1. PicoRV32 instruction fetch from Boot ROM (0x0000_0000)
//   2. CPU Read / Write to System RAM (0x0100_0000)
//   3. CPU Access to CRC-8 Hardware Accelerator (0x0200_1000)
//   4. CPU Access to Standalone BPSK Direct Modulator (0x0200_2010 - 0x0200_2014)
//   5. CPU Access to Digital Input Register (0x0200_0000)
//   6. CPU Trigger and Monitor of Sonar BPSK TX Packet Transmission (0x0200_2000)
//   7. Verification of Sonar BPSK sample generation
//   8. Memory integrity check (Magic Pass Code 0xCAFEF00D written to RAM[0x20])
//==============================================================================
module tb_picorv32_top;

    reg clk;
    reg resetn;

    // DUT output signals
    wire                      trap;
    wire                      bus_error;
    wire signed [7:0]         bpsk_sample;
    wire                      sample_valid;
    wire                      sonar_busy;
    wire                      sonar_done;
    wire                      sonar_len_error;
    wire signed [15:0]        bpsk_mod_out;
    wire                      bpsk_mod_valid;
    wire [7:0]                inreg_data_out;
    wire                      inreg_data_load;
    wire [3:0]                sonar_state_dbg;

    integer tests_passed = 0;
    integer tests_failed = 0;
    integer sonar_sample_count = 0;
    reg     sim_completed = 0;

    // Instantiate Top Module
    picorv32_top #(
        .PROGADDR_RESET  (32'h0000_0000),
        .STACKADDR       (32'h0101_0000),
        .ROM_INIT_FILE   ("rom_test.hex"),
        .BARREL_SHIFTER  (1),
        .MAX_PAYLOAD_LEN (32),
        .PHASE_STEP      (16),
        .CYCLES_PER_BIT  (1)
    ) dut (
        .clk             (clk),
        .resetn          (resetn),
        .trap            (trap),
        .bus_error       (bus_error),
        .bpsk_sample     (bpsk_sample),
        .sample_valid    (sample_valid),
        .sonar_busy      (sonar_busy),
        .sonar_done      (sonar_done),
        .sonar_len_error (sonar_len_error),
        .bpsk_mod_out    (bpsk_mod_out),
        .bpsk_mod_valid  (bpsk_mod_valid),
        .inreg_data_out  (inreg_data_out),
        .inreg_data_load (inreg_data_load),
        .sonar_state_dbg (sonar_state_dbg)
    );

    // 100 MHz clock generation (10 ns period)
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // VCD waveform dump
    initial begin
        $dumpfile("tb_picorv32_top.vcd");
        $dumpvars(0, tb_picorv32_top);
    end

    // Watchdog timer (timeout after 500 us = 50,000 cycles)
    initial begin
        #500000;
        if (!sim_completed) begin
            $display("\n=======================================================");
            $display("[ERROR] Simulation TIMEOUT reached at time %0t ns!", $time);
            $display("=======================================================");
            $finish;
        end
    end

    // Track Sonar TX sample stream
    always @(posedge clk) begin
        if (sample_valid) begin
            sonar_sample_count <= sonar_sample_count + 1;
        end
    end

    // Bus cycle monitoring for debugging
    always @(posedge clk) begin
        if (resetn && dut.mem_valid && dut.mem_ready) begin
            if (|dut.mem_wstrb) begin
                // Write access
                if (dut.mem_addr >= 32'h0100_0000 && dut.mem_addr < 32'h0101_0000)
                    $display("[TIME %06t ns] [CPU -> RAM WRITE] Addr=0x%08h, WData=0x%08h, Strb=%b", 
                             $time, dut.mem_addr, dut.mem_wdata, dut.mem_wstrb);
                else if (dut.mem_addr >= 32'h0200_0000 && dut.mem_addr < 32'h0200_1000)
                    $display("[TIME %06t ns] [CPU -> INREG WRITE] Addr=0x%08h, WData=0x%08h", 
                             $time, dut.mem_addr, dut.mem_wdata);
                else if (dut.mem_addr >= 32'h0200_1000 && dut.mem_addr < 32'h0200_2000)
                    $display("[TIME %06t ns] [CPU -> CRC WRITE]   Addr=0x%08h, WData=0x%08h", 
                             $time, dut.mem_addr, dut.mem_wdata);
                else if (dut.mem_addr >= 32'h0200_2000 && dut.mem_addr < 32'h0200_3000)
                    $display("[TIME %06t ns] [CPU -> BPSK WRITE]  Addr=0x%08h, WData=0x%08h", 
                             $time, dut.mem_addr, dut.mem_wdata);
            end else begin
                // Read access
                if (dut.mem_addr >= 32'h0200_1000 && dut.mem_addr < 32'h0200_2000)
                    $display("[TIME %06t ns] [CPU <- CRC READ]    Addr=0x%08h, RData=0x%08h", 
                             $time, dut.mem_addr, dut.mem_rdata);
                else if (dut.mem_addr >= 32'h0200_2000 && dut.mem_addr < 32'h0200_3000)
                    $display("[TIME %06t ns] [CPU <- BPSK READ]   Addr=0x%08h, RData=0x%08h", 
                             $time, dut.mem_addr, dut.mem_rdata);
                else if (dut.mem_addr >= 32'h0200_0000 && dut.mem_addr < 32'h0200_1000)
                    $display("[TIME %06t ns] [CPU <- INREG READ]  Addr=0x%08h, RData=0x%08h", 
                             $time, dut.mem_addr, dut.mem_rdata);
            end
        end
    end

    // Monitor Trap and Bus Error
    always @(posedge clk) begin
        if (resetn && trap) begin
            $display("[FATAL ERROR] PicoRV32 TRAP asserted at time %0t ns!", $time);
            tests_failed = tests_failed + 1;
            $finish;
        end
        if (resetn && bus_error) begin
            $display("[FATAL ERROR] Bus Error asserted at time %0t ns! Addr=0x%08h", $time, dut.mem_addr);
            tests_failed = tests_failed + 1;
            $finish;
        end
    end

    // Test sequence
    initial begin
        $display("==================================================================");
        $display("  STARTING PICORV32 SOC SIMULATION (CRC-8 & BPSK INTEGRATION)");
        $display("==================================================================");

        // Reset system
        resetn = 1'b0;
        repeat (10) @(posedge clk);
        #1;
        resetn = 1'b1;
        $display("[TIME %06t ns] Reset released. CPU starting execution at 0x0000_0000...", $time);

        // Wait until Magic PASS Word (0xCAFEF00D) is written to RAM[32] (word_addr = 8)
        // Word address: 0x0100_0020 >> 2 = 8
        wait (dut.u_ram.mem[8] === 32'hCAFEF00D);
        #100; // allow pipeline to settle

        $display("\n==================================================================");
        $display("  PicoRV32 FIRMWARE EXECUTION COMPLETED: VERIFYING SOC REGISTERS");
        $display("==================================================================");

        // 1. Check RAM R/W test at RAM[0] (0x0100_0000)
        if (dut.u_ram.mem[0] === 32'h12345678) begin
            $display("[PASS] RAM Basic R/W Check: expected=0x12345678, actual=0x%08h", dut.u_ram.mem[0]);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] RAM Basic R/W Check: expected=0x12345678, actual=0x%08h", dut.u_ram.mem[0]);
            tests_failed = tests_failed + 1;
        end

        // 2. Check CRC Reset Result at RAM[1] (0x0100_0004) -> expect 0x00000055
        if (dut.u_ram.mem[1] === 32'h00000055) begin
            $display("[PASS] CRC-8 Reset Check: expected=0x00000055, actual=0x%08h", dut.u_ram.mem[1]);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] CRC-8 Reset Check: expected=0x00000055, actual=0x%08h", dut.u_ram.mem[1]);
            tests_failed = tests_failed + 1;
        end

        // 3. Check CRC-8 of '1' (0x31) at RAM[2] (0x0100_0008) -> expect 0x000000C2
        if (dut.u_ram.mem[2] === 32'h000000C2) begin
            $display("[PASS] CRC-8 Byte '1' Check: expected=0x000000C2, actual=0x%08h", dut.u_ram.mem[2]);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] CRC-8 Byte '1' Check: expected=0x000000C2, actual=0x%08h", dut.u_ram.mem[2]);
            tests_failed = tests_failed + 1;
        end

        // 4. Check CRC-8 of '12' at RAM[3] (0x0100_000C) -> non-zero valid CRC
        if (dut.u_ram.mem[3][31:8] === 24'h0 && dut.u_ram.mem[3][7:0] !== 8'h0) begin
            $display("[PASS] CRC-8 Two Bytes '12' Check: actual=0x%08h", dut.u_ram.mem[3]);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] CRC-8 Two Bytes '12' Check: actual=0x%08h", dut.u_ram.mem[3]);
            tests_failed = tests_failed + 1;
        end

        // 5. Check Direct BPSK Modulator output for Bit 1 at RAM[4] (0x0100_0010)
        // Bit 1 -> -1000 (0xFFFF_FC18)
        if ($signed(dut.u_ram.mem[4]) === -32'sd1000) begin
            $display("[PASS] BPSK Direct Modulator Bit 1: expected=-1000, actual=%0d (0x%08h)", 
                     $signed(dut.u_ram.mem[4]), dut.u_ram.mem[4]);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] BPSK Direct Modulator Bit 1: expected=-1000, actual=%0d (0x%08h)", 
                     $signed(dut.u_ram.mem[4]), dut.u_ram.mem[4]);
            tests_failed = tests_failed + 1;
        end

        // 6. Check Direct BPSK Modulator output for Bit 0 at RAM[5] (0x0100_0014)
        // Bit 0 -> +1000 (0x0000_03E8)
        if ($signed(dut.u_ram.mem[5]) === 32'sd1000) begin
            $display("[PASS] BPSK Direct Modulator Bit 0: expected=+1000, actual=%0d (0x%08h)", 
                     $signed(dut.u_ram.mem[5]), dut.u_ram.mem[5]);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] BPSK Direct Modulator Bit 0: expected=+1000, actual=%0d (0x%08h)", 
                     $signed(dut.u_ram.mem[5]), dut.u_ram.mem[5]);
            tests_failed = tests_failed + 1;
        end

        // 7. Check Digital Input Register readback at RAM[6] (0x0100_0018)
        if (dut.u_ram.mem[6] === 32'h000000A5) begin
            $display("[PASS] Digital Input Register Check: expected=0x000000A5, actual=0x%08h", dut.u_ram.mem[6]);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Digital Input Register Check: expected=0x000000A5, actual=0x%08h", dut.u_ram.mem[6]);
            tests_failed = tests_failed + 1;
        end

        // 8. Check Sonar TX packet generation & samples
        // 1 byte payload + 1 byte CRC = 16 bits * 16 samples/bit = 256 samples
        if (sonar_sample_count > 0) begin
            $display("[PASS] Sonar TX Sample Generator Stream: received %0d valid samples", sonar_sample_count);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Sonar TX Sample Generator Stream: no samples generated!");
            tests_failed = tests_failed + 1;
        end

        // 9. Check Magic PASS code at RAM[8]
        if (dut.u_ram.mem[8] === 32'hCAFEF00D) begin
            $display("[PASS] SoC Magic Signature Check: expected=0xCAFEF00D, actual=0x%08h", dut.u_ram.mem[8]);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] SoC Magic Signature Check: expected=0xCAFEF00D, actual=0x%08h", dut.u_ram.mem[8]);
            tests_failed = tests_failed + 1;
        end

        sim_completed = 1'b1;

        $display("\n==================================================================");
        $display("  SOC SIMULATION SUMMARY: %0d TESTS PASSED, %0d TESTS FAILED", tests_passed, tests_failed);
        if (tests_failed == 0)
            $display("  >>> ALL CHECKS PASSED SUCCESSFULLY! <<<");
        else
            $display("  >>> SOME CHECKS FAILED! <<<");
        $display("==================================================================\n");

        $finish;
    end

endmodule
