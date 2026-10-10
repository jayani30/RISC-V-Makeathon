`timescale 1ns / 1ps
//==============================================================================
// picorv32_soc_top : FPGA Board-Level SoC Top Wrapper
//
// Features:
//   - Integrates PicoRV32 SoC (PicoRV32 RISC-V CPU, Boot ROM, Dual-Port RAM,
//     CRC-8 Hardware Accelerator, BPSK Modulator, Digital Input Register, Sonar TX)
//   - 100 MHz Board Clock & Metastability-Hardened Reset Synchronizer
//   - 1-Bit First-Order Delta-Sigma (PDM) DAC for direct PMOD / Scope probing
//   - 8-Bit Unsigned Parallel DAC Bus (for R-2R PMOD / External DAC)
//   - Pulse-Stretched Visual LED Status Indicators (Busy, Done, Errors, Heartbeat)
//   - Board User Buttons (Manual Start Trigger) & Switches
//==============================================================================
module picorv32_soc_top #(
    parameter CLK_FREQ_HZ     = 100_000_000,
    parameter ROM_INIT_FILE   = "rom_test.hex",
    parameter MAX_PAYLOAD_LEN = 32,
    parameter PHASE_STEP      = 16,
    parameter CYCLES_PER_BIT  = 1,
    parameter RESET_ACTIVE_LOW= 1
)(
    input  wire        clk_100mhz,      // 100 MHz board oscillator
    input  wire        btn_reset,       // Board reset button (default active-low)

    // User interaction ports
    input  wire        btn_start,       // Optional manual transmit trigger
    input  wire [7:0]  sw,              // Optional slide switches

    // Board diagnostic LEDs
    output wire [7:0]  led,

    // Analog / Physical BPSK outputs
    output wire        bpsk_pdm_out,    // 1-bit Delta-Sigma PDM DAC (for direct scope/audio pin)
    output wire [7:0]  bpsk_dac_raw,    // 8-bit unsigned parallel DAC (for R-2R ladder)
    output wire        bpsk_val_out,    // Sample valid pulse flag
    output wire        sonar_busy_pin,  // Dedicated busy output pin
    output wire        sonar_done_pin   // Dedicated done output pin
);

    //==========================================================================
    // 1. Reset Synchronizer (Metastability protection)
    //==========================================================================
    wire raw_resetn = RESET_ACTIVE_LOW ? btn_reset : ~btn_reset;
    reg [2:0] rst_sync = 3'b000;

    always @(posedge clk_100mhz) begin
        rst_sync <= {rst_sync[1:0], raw_resetn};
    end

    wire soc_resetn = rst_sync[2];

    //==========================================================================
    // 2. Button Edge Detector
    //==========================================================================
    reg [1:0] btn_sync = 2'b00;
    always @(posedge clk_100mhz) begin
        btn_sync <= {btn_sync[0], btn_start};
    end
    wire btn_start_pulse = (btn_sync == 2'b01);

    //==========================================================================
    // 3. Instantiate Core PicoRV32 SoC Top
    //==========================================================================
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

    picorv32_top #(
        .PROGADDR_RESET  (32'h0000_0000),
        .STACKADDR       (32'h0101_0000),
        .ROM_INIT_FILE   (ROM_INIT_FILE),
        .BARREL_SHIFTER  (1),
        .MAX_PAYLOAD_LEN (MAX_PAYLOAD_LEN),
        .PHASE_STEP      (PHASE_STEP),
        .CYCLES_PER_BIT  (CYCLES_PER_BIT)
    ) u_soc_core (
        .clk             (clk_100mhz),
        .resetn          (soc_resetn),

        // CPU status
        .trap            (trap),
        .bus_error       (bus_error),

        // Sonar TX BPSK output stream
        .bpsk_sample     (bpsk_sample),
        .sample_valid    (sample_valid),
        .sonar_busy      (sonar_busy),
        .sonar_done      (sonar_done),
        .sonar_len_error (sonar_len_error),

        // Standalone direct BPSK modulator outputs
        .bpsk_mod_out    (bpsk_mod_out),
        .bpsk_mod_valid  (bpsk_mod_valid),

        // Diagnostic / debug ports
        .inreg_data_out  (inreg_data_out),
        .inreg_data_load (inreg_data_load),
        .sonar_state_dbg (sonar_state_dbg)
    );

    //==========================================================================
    // 4. First-Order Delta-Sigma (PDM) DAC for 1-Pin Analog BPSK Output
    //==========================================================================
    // Convert signed 8-bit (-128 .. +127) to unsigned 8-bit (0 .. 255)
    wire [7:0] sample_u8 = bpsk_sample + 8'sd128;
    assign bpsk_dac_raw  = sample_u8;
    assign bpsk_val_out  = sample_valid;

    reg [8:0] sigma_acc = 9'd0;
    reg       pdm_out_r = 1'b0;

    always @(posedge clk_100mhz) begin
        if (!soc_resetn) begin
            sigma_acc <= 9'd0;
            pdm_out_r <= 1'b0;
        end else begin
            sigma_acc <= sigma_acc[7:0] + sample_u8;
            pdm_out_r <= sigma_acc[8];
        end
    end

    assign bpsk_pdm_out   = pdm_out_r;
    assign sonar_busy_pin = sonar_busy;
    assign sonar_done_pin = sonar_done;

    //==========================================================================
    // 5. LED Pulse Stretcher (Allows human eye to see fast pulses)
    //==========================================================================
    // ~20ms stretch counter (2,000,000 cycles at 100 MHz)
    localparam STRETCH_CYCLES = CLK_FREQ_HZ / 50; 
    reg [21:0] stretch_done = 22'd0;
    reg [21:0] stretch_load = 22'd0;

    always @(posedge clk_100mhz) begin
        if (!soc_resetn) begin
            stretch_done <= 22'd0;
            stretch_load <= 22'd0;
        end else begin
            if (sonar_done)
                stretch_done <= STRETCH_CYCLES;
            else if (stretch_done > 0)
                stretch_done <= stretch_done - 22'd1;

            if (inreg_data_load)
                stretch_load <= STRETCH_CYCLES;
            else if (stretch_load > 0)
                stretch_load <= stretch_load - 22'd1;
        end
    end

    // Heartbeat blinker (1 Hz blink when SoC is alive)
    reg [25:0] heartbeat_cnt = 26'd0;
    always @(posedge clk_100mhz) begin
        if (!soc_resetn)
            heartbeat_cnt <= 26'd0;
        else
            heartbeat_cnt <= heartbeat_cnt + 26'd1;
    end

    // Monitor firmware completion: Check if Magic Pass Code is in RAM word 8
    wire fw_passed = (u_soc_core.u_ram.mem[8] == 32'hCAFEF00D);

    // LED mapping
    assign led[0] = sonar_busy;                     // LED 0: Sonar TX Busy
    assign led[1] = (stretch_done > 0) || sonar_done;// LED 1: Sonar TX Done (stretched)
    assign led[2] = sonar_len_error;                // LED 2: Packet length error
    assign led[3] = sample_valid;                   // LED 3: BPSK sample valid stream
    assign led[4] = (stretch_load > 0);             // LED 4: Input Register loaded
    assign led[5] = bus_error;                      // LED 5: Memory bus error
    assign led[6] = trap;                           // LED 6: PicoRV32 CPU trap
    assign led[7] = fw_passed ? 1'b1 : heartbeat_cnt[24]; // LED 7: Solid ON when firmware passed, else heartbeat

endmodule
