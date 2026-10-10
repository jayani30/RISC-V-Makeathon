`timescale 1ns / 1ps
//==============================================================================
// Module: top.v
// Project: Integration of PicoRV32 RISC-V SoC with Sonar Transmitter RTL
// Target: Xilinx Vivado (Artix-7 / Basys 3 / Any FPGA board)
//
// Description:
//   Clean, unified FPGA top-level design integrating:
//     1. PicoRV32 RISC-V 32-bit CPU (Claire Wolf / YosysHQ)
//     2. 64 KB Instruction Boot ROM (rom.v, initialized from ROM_INIT_FILE)
//     3. 64 KB Shared Data RAM (ram.v, byte-addressable)
//     4. 2-to-1 Memory Bus Arbiter (sonar_ram_arbiter: CPU vs Sonar TX Master)
//     5. MMIO Address Decoder (mmio_address_decoder: 0x0000_0000 - 0x0200_2FFF)
//     6. Hardware CRC-8/ITU Accelerator (crcaccel.v at 0x0200_1000)
//     7. Sonar MMIO Control & Status Registers (bpsk_ctrl.v at 0x0200_2000)
//     8. Sonar Transmitter Top Module (sonar_tx_top.v)
//     9. Standalone Baseband BPSK Modulator (bpsk.v)
//    10. First-Order Delta-Sigma (PDM) 1-Bit DAC for direct oscilloscope / PMOD probing
//    11. 8-Bit Unsigned Parallel DAC Bus for external R-2R ladder DAC
//    12. Metastability-hardened 3-stage reset & button synchronizers
//    13. Pulse-stretched board diagnostic LEDs
//
// Memory Map:
//   0x0000_0000 - 0x0000_FFFF : 64 KB Instruction Boot ROM (Read-Only)
//   0x0100_0000 - 0x0100_FFFF : 64 KB System RAM (CPU RW, Sonar TX Read-Master)
//   0x0200_0000 - 0x0200_0FFF : Digital Input Register (1-byte immediate payload)
//   0x0200_1000 - 0x0200_1FFF : CRC-8/ITU Accelerator (DATA, CTRL, RESULT, STATUS)
//   0x0200_2000 - 0x0200_2FFF : Sonar & BPSK Control Registers:
//                                 0x00: CONTROL   [bit 0: start, bit 1: src_sel]
//                                 0x04: CONFIG    [bits 7:0: payload_len, 31:16: ram_base]
//                                 0x08: STATUS    [bit 0: busy, bit 1: done, bit 2: len_err]
//                                 0x0C: SAMPLE    [bits 7:0: last_sample, 8: val, 31:16: cnt]
//                                 0x10: DIRECT_MOD[direct modulator test controls]
//                                 0x14: DIRECT_OUT[direct modulator 16-bit sample]
//==============================================================================

module top #(
    parameter integer CLK_FREQ_HZ      = 100_000_000,
    parameter         ROM_INIT_FILE    = "rom_test.hex",
    parameter integer MAX_PAYLOAD_LEN  = 32,
    parameter integer PHASE_STEP       = 16,
    parameter integer CYCLES_PER_BIT   = 1,
    parameter integer RESET_ACTIVE_LOW = 1,
    parameter [31:0]  PROGADDR_RESET   = 32'h0000_0000,
    parameter [31:0]  STACKADDR        = 32'h0101_0000,
    parameter integer BARREL_SHIFTER   = 1
)(
    // Clock and Reset
    input  wire        clk_100mhz,      // 100 MHz board oscillator
    input  wire        btn_reset,       // Reset button (active-low by default)

    // User interaction ports
    input  wire        btn_start,       // Optional manual transmit trigger button
    input  wire [7:0]  sw,              // 8-bit board slide switches

    // Diagnostic LEDs
    output wire [7:0]  led,             // Diagnostic LEDs (Busy, Done, Error, Pass)

    // Physical / Analog Sonar Outputs
    output wire        bpsk_pdm_out,    // 1-Bit Delta-Sigma PDM DAC (PMOD JA pin 1)
    output wire [7:0]  bpsk_dac_raw,    // 8-Bit Unsigned Parallel DAC (PMOD JB)
    output wire        bpsk_val_out,    // Sample valid strobe pulse (PMOD JA pin 2)
    output wire        sonar_busy_pin,  // Dedicated busy output pin (PMOD JA pin 3)
    output wire        sonar_done_pin   // Dedicated done output pin (PMOD JA pin 4)
);

    //==========================================================================
    // 1. Reset Synchronizer & Button Edge Detection
    //==========================================================================
    wire raw_resetn = RESET_ACTIVE_LOW ? btn_reset : ~btn_reset;
    reg [2:0] rst_sync = 3'b000;

    always @(posedge clk_100mhz) begin
        rst_sync <= {rst_sync[1:0], raw_resetn};
    end

    wire sys_resetn = rst_sync[2];

    reg [1:0] btn_sync = 2'b00;
    always @(posedge clk_100mhz) begin
        btn_sync <= {btn_sync[0], btn_start};
    end
    wire btn_start_pulse = (btn_sync == 2'b01);

    //==========================================================================
    // 2. Internal Interconnect Signals
    //==========================================================================
    // PicoRV32 Native Memory Interface
    wire        mem_valid;
    wire        mem_instr;
    wire        mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wstrb;
    wire [31:0] mem_rdata;
    wire        trap;
    wire        bus_error;

    // ROM Channel
    wire        rom_valid;
    wire [31:0] rom_addr;
    wire [31:0] rom_wdata;
    wire [3:0]  rom_wstrb;
    wire [31:0] rom_rdata;
    wire        rom_ready;

    // RAM Channel (CPU side)
    wire        cpu_ram_valid;
    wire [31:0] cpu_ram_addr;
    wire [31:0] cpu_ram_wdata;
    wire [3:0]  cpu_ram_wstrb;
    wire [31:0] cpu_ram_rdata;
    wire        cpu_ram_ready;

    // Digital Input Register Channel (MMIO 0x0200_0000)
    wire        in_valid;
    wire        in_write_en;
    wire [31:0] in_addr;
    wire [31:0] in_wdata;
    wire [3:0]  in_wstrb;
    wire [31:0] in_rdata;
    wire        in_ready;

    // CRC-8 Accelerator Channel (MMIO 0x0200_1000)
    wire        crc_valid;
    wire        crc_write_en;
    wire [31:0] crc_addr;
    wire [31:0] crc_wdata;
    wire [3:0]  crc_wstrb;
    wire [31:0] crc_rdata;
    wire        crc_ready;

    // BPSK Control Channel (MMIO 0x0200_2000)
    wire        bpsk_valid;
    wire        bpsk_write_en;
    wire [31:0] bpsk_addr;
    wire [31:0] bpsk_wdata;
    wire [3:0]  bpsk_wstrb;
    wire [31:0] bpsk_rdata;
    wire        bpsk_ready;

    // Shared RAM Physical Bus
    wire        ram_valid;
    wire [31:0] ram_addr;
    wire [31:0] ram_wdata;
    wire [3:0]  ram_wstrb;
    wire [31:0] ram_rdata;
    wire        ram_ready;

    // Sonar TX RAM Read Master Bus
    wire        son_rm_valid;
    wire [31:0] son_rm_addr;
    wire [31:0] son_rm_rdata;
    wire        son_rm_ready;
    wire        son_rm_busy;

    // Control and Status Wires between bpsk_ctrl and sonar_tx_top
    wire        c_sonar_start;
    wire [7:0]  c_sonar_payload_len;
    wire        c_sonar_src_sel;
    wire [15:0] c_sonar_ram_base_addr;
    wire        sonar_busy;
    wire        sonar_done;
    wire        sonar_len_error;
    wire signed [7:0] bpsk_sample;
    wire        sample_valid;
    wire        inreg_data_load;
    wire [3:0]  sonar_state_dbg;

    // Direct Modulator Wires
    wire        c_direct_data_in;
    wire        c_direct_data_valid;
    wire        c_direct_sample_en;
    wire signed [15:0] bpsk_mod_out;
    wire        bpsk_mod_valid;

    // Combined Start Trigger (MMIO write OR physical button pulse)
    wire effective_start = c_sonar_start | btn_start_pulse;

    //==========================================================================
    // 3. PicoRV32 RISC-V CPU Core
    //==========================================================================
    picorv32 #(
        .PROGADDR_RESET      (PROGADDR_RESET),
        .STACKADDR           (STACKADDR),
        .ENABLE_COUNTERS     (1),
        .ENABLE_COUNTERS64   (1),
        .ENABLE_REGS_16_31   (1),
        .ENABLE_REGS_DUALPORT(1),
        .LATCHED_MEM_RDATA   (0),
        .TWO_STAGE_SHIFT     (1),
        .BARREL_SHIFTER      (BARREL_SHIFTER),
        .TWO_CYCLE_COMPARE   (0),
        .TWO_CYCLE_ALU       (0),
        .COMPRESSED_ISA      (0),
        .CATCH_MISALIGN      (1),
        .CATCH_ILLINSN       (1),
        .ENABLE_PCPI         (0),
        .ENABLE_MUL          (0),
        .ENABLE_FAST_MUL     (0),
        .ENABLE_DIV          (0),
        .ENABLE_IRQ          (0),
        .ENABLE_TRACE        (0),
        .REGS_INIT_ZERO      (1)
    ) u_cpu (
        .clk          (clk_100mhz),
        .resetn       (sys_resetn),
        .trap         (trap),

        .mem_valid    (mem_valid),
        .mem_instr    (mem_instr),
        .mem_ready    (mem_ready),
        .mem_addr     (mem_addr),
        .mem_wdata    (mem_wdata),
        .mem_wstrb    (mem_wstrb),
        .mem_rdata    (mem_rdata),

        // Unused optional interfaces
        .mem_la_read  (),
        .mem_la_write (),
        .mem_la_addr  (),
        .mem_la_wdata (),
        .mem_la_wstrb (),
        .pcpi_valid   (),
        .pcpi_insn    (),
        .pcpi_rs1     (),
        .pcpi_rs2     (),
        .pcpi_wr      (1'b0),
        .pcpi_rd      (32'h0),
        .pcpi_wait    (1'b0),
        .pcpi_ready   (1'b0),
        .irq          (32'h0),
        .eoi          (),
        .trace_valid  (),
        .trace_data   ()
    );

    //==========================================================================
    // 4. Memory-Mapped Address Decoder
    //==========================================================================
    mmio_address_decoder u_decoder (
        .mem_valid    (mem_valid),
        .mem_addr     (mem_addr),
        .mem_wdata    (mem_wdata),
        .mem_wstrb    (mem_wstrb),
        .mem_ready    (mem_ready),
        .mem_rdata    (mem_rdata),

        // ROM (0x0000_0000 - 0x0000_FFFF)
        .rom_valid    (rom_valid),
        .rom_addr     (rom_addr),
        .rom_wdata    (rom_wdata),
        .rom_wstrb    (rom_wstrb),
        .rom_rdata    (rom_rdata),
        .rom_ready    (rom_ready),

        // RAM (0x0100_0000 - 0x0100_FFFF)
        .ram_valid    (cpu_ram_valid),
        .ram_addr     (cpu_ram_addr),
        .ram_wdata    (cpu_ram_wdata),
        .ram_wstrb    (cpu_ram_wstrb),
        .ram_rdata    (cpu_ram_rdata),
        .ram_ready    (cpu_ram_ready),

        // Digital Input Register (0x0200_0000 - 0x0200_0FFF)
        .in_valid     (in_valid),
        .in_write_en  (in_write_en),
        .in_addr      (in_addr),
        .in_wdata     (in_wdata),
        .in_wstrb     (in_wstrb),
        .in_rdata     (in_rdata),
        .in_ready     (in_ready),

        // CRC-8 Accelerator (0x0200_1000 - 0x0200_1FFF)
        .crc_valid    (crc_valid),
        .crc_write_en (crc_write_en),
        .crc_addr     (crc_addr),
        .crc_wdata    (crc_wdata),
        .crc_wstrb    (crc_wstrb),
        .crc_rdata    (crc_rdata),
        .crc_ready    (crc_ready),

        // BPSK & Sonar Control (0x0200_2000 - 0x0200_2FFF)
        .bpsk_valid   (bpsk_valid),
        .bpsk_write_en(bpsk_write_en),
        .bpsk_addr    (bpsk_addr),
        .bpsk_wdata   (bpsk_wdata),
        .bpsk_wstrb   (bpsk_wstrb),
        .bpsk_rdata   (bpsk_rdata),
        .bpsk_ready   (bpsk_ready),

        .bus_error    (bus_error)
    );

    //==========================================================================
    // 5. Instruction Boot ROM (64 KB)
    //==========================================================================
    rom #(
        .WORDS        (16384),
        .INIT_FILE    (ROM_INIT_FILE)
    ) u_rom (
        .clk          (clk_100mhz),
        .resetn       (sys_resetn),
        .valid        (rom_valid),
        .addr         (rom_addr),
        .rdata        (rom_rdata),
        .ready        (rom_ready)
    );

    //==========================================================================
    // 6. 2-to-1 Shared RAM Arbiter (CPU vs Sonar TX Read Master)
    //==========================================================================
    sonar_ram_arbiter u_ram_arbiter (
        .sel_sonar    (son_rm_busy),

        // CPU Slave Port
        .cpu_valid    (cpu_ram_valid),
        .cpu_addr     (cpu_ram_addr),
        .cpu_wdata    (cpu_ram_wdata),
        .cpu_wstrb    (cpu_ram_wstrb),
        .cpu_rdata    (cpu_ram_rdata),
        .cpu_ready    (cpu_ram_ready),

        // Sonar TX Master Port
        .son_valid    (son_rm_valid),
        .son_addr     (son_rm_addr),
        .son_rdata    (son_rm_rdata),
        .son_ready    (son_rm_ready),

        // Physical RAM Port
        .ram_valid    (ram_valid),
        .ram_addr     (ram_addr),
        .ram_wdata    (ram_wdata),
        .ram_wstrb    (ram_wstrb),
        .ram_rdata    (ram_rdata),
        .ram_ready    (ram_ready)
    );

    //==========================================================================
    // 7. System Data RAM (64 KB)
    //==========================================================================
    ram #(
        .WORDS        (16384)
    ) u_ram (
        .clk          (clk_100mhz),
        .resetn       (sys_resetn),
        .valid        (ram_valid),
        .addr         (ram_addr),
        .wdata        (ram_wdata),
        .wstrb        (ram_wstrb),
        .rdata        (ram_rdata),
        .ready        (ram_ready)
    );

    //==========================================================================
    // 8. Hardware CRC-8 Accelerator (MMIO peripheral at 0x0200_1000)
    //==========================================================================
    crc8_accelerator u_crc_accel (
        .clk          (clk_100mhz),
        .resetn       (sys_resetn),
        .valid        (crc_valid),
        .write_en     (crc_write_en),
        .addr         (crc_addr),
        .wdata        (crc_wdata),
        .wstrb        (crc_wstrb),
        .rdata        (crc_rdata),
        .ready        (crc_ready)
    );

    //==========================================================================
    // 9. Sonar & BPSK Control Registers (MMIO peripheral at 0x0200_2000)
    //==========================================================================
    bpsk_ctrl u_bpsk_ctrl (
        .clk                 (clk_100mhz),
        .resetn              (sys_resetn),

        // MMIO bus
        .valid               (bpsk_valid),
        .write_en            (bpsk_write_en),
        .addr                (bpsk_addr),
        .wdata               (bpsk_wdata),
        .wstrb               (bpsk_wstrb),
        .rdata               (bpsk_rdata),
        .ready               (bpsk_ready),

        // Sonar TX control & status
        .sonar_start         (c_sonar_start),
        .sonar_payload_len   (c_sonar_payload_len),
        .sonar_src_sel       (c_sonar_src_sel),
        .sonar_ram_base_addr (c_sonar_ram_base_addr),
        .sonar_busy          (sonar_busy),
        .sonar_done          (sonar_done),
        .sonar_len_error     (sonar_len_error),
        .sonar_bpsk_sample   (bpsk_sample),
        .sonar_sample_valid  (sample_valid),

        // Standalone modulator interface
        .direct_data_in      (c_direct_data_in),
        .direct_data_valid   (c_direct_data_valid),
        .direct_sample_en    (c_direct_sample_en),
        .direct_bpsk_out     (bpsk_mod_out),
        .direct_out_valid    (bpsk_mod_valid)
    );

    //==========================================================================
    // 10. Sonar Transmitter Top RTL Module (sonar_tx_top.v)
    //==========================================================================
    sonar_tx_top #(
        .MAX_LEN             (MAX_PAYLOAD_LEN),
        .PHASE_STEP          (PHASE_STEP),
        .CYCLES_PER_BIT      (CYCLES_PER_BIT),
        .BIT1_NEGATIVE       (0),
        .RAM_BASE            (32'h0100_0000)
    ) u_sonar_tx (
        .clk                 (clk_100mhz),
        .resetn              (sys_resetn),

        // PicoRV32 native bus ports (tied off cleanly to prevent warnings & conflicts)
        .mem_valid           (1'b0),
        .mem_addr            (32'h0000_0000),
        .mem_wdata           (32'h0000_0000),
        .mem_wstrb           (4'h0),
        .mem_ready           (),
        .mem_rdata           (),
        .bus_error           (),

        // Control / Command Interface
        .start               (effective_start),
        .payload_len         (c_sonar_payload_len),
        .src_sel             (c_sonar_src_sel),
        .ram_base_addr       (c_sonar_ram_base_addr),
        .busy                (sonar_busy),
        .done                (sonar_done),
        .len_error           (sonar_len_error),

        // Physical BPSK Output Stream
        .bpsk_sample         (bpsk_sample),
        .sample_valid        (sample_valid),
        .bpsk_mod_out        (),
        .bpsk_mod_valid      (),

        // External RAM Interface (Unused internally, driven by arbiter)
        .ram_valid           (),
        .ram_addr            (),
        .ram_wdata           (),
        .ram_wstrb           (),
        .ram_rdata           (32'h0000_0000),
        .ram_ready           (1'b0),

        // Dedicated Sonar RAM Master Interface
        .rm_valid            (son_rm_valid),
        .rm_addr             (son_rm_addr),
        .rm_rdata            (son_rm_rdata),
        .rm_ready            (son_rm_ready),
        .rm_busy             (son_rm_busy),

        // Diagnostic / Input Register Ports
        .inreg_data_out      (),
        .inreg_data_load     (inreg_data_load),

        // Digital Input Register Direct Slave Interface (MMIO 0x0200_0000)
        .in_valid            (in_valid),
        .in_write_en         (in_write_en),
        .in_addr             (in_addr),
        .in_wdata            (in_wdata),
        .in_wstrb            (in_wstrb),
        .in_rdata            (in_rdata),
        .in_ready            (in_ready),
        .in_load             (),

        // Internal Debug Ports
        .dbg_bit_valid       (),
        .dbg_bit_data        (),
        .dbg_bit_ready       (),
        .dbg_bit_last        (),
        .dbg_state           (sonar_state_dbg)
    );

    //==========================================================================
    // 11. Standalone Direct Baseband BPSK Modulator (bpsk.v)
    //==========================================================================
    bpsk_modulator #(
        .OUTPUT_WIDTH        (16),
        .AMPLITUDE           (1000)
    ) u_bpsk_mod (
        .clk                 (clk_100mhz),
        .rst                 (!sys_resetn),
        .data_in             (c_direct_data_in),
        .data_valid          (c_direct_data_valid),
        .sample_en           (c_direct_sample_en),
        .bpsk_out            (bpsk_mod_out),
        .out_valid           (bpsk_mod_valid)
    );

    //==========================================================================
    // 12. First-Order Delta-Sigma (PDM) 1-Bit DAC
    //==========================================================================
    // Converts signed 8-bit carrier BPSK sample (-128..+127) to unsigned (0..255)
    wire [7:0] sample_u8 = bpsk_sample + 8'sd128;
    assign bpsk_dac_raw  = sample_u8;
    assign bpsk_val_out  = sample_valid;

    reg [8:0] sigma_acc = 9'd0;
    reg       pdm_out_r = 1'b0;

    always @(posedge clk_100mhz) begin
        if (!sys_resetn) begin
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
    // 13. Pulse Stretchers & Diagnostic LED Indicators
    //==========================================================================
    // ~20ms pulse stretcher for human visible feedback
    localparam STRETCH_CYCLES = CLK_FREQ_HZ / 50; 
    reg [21:0] stretch_done = 22'd0;
    reg [21:0] stretch_load = 22'd0;

    always @(posedge clk_100mhz) begin
        if (!sys_resetn) begin
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

    // Heartbeat counter (1 Hz toggle)
    reg [25:0] heartbeat_cnt = 26'd0;
    always @(posedge clk_100mhz) begin
        if (!sys_resetn)
            heartbeat_cnt <= 26'd0;
        else
            heartbeat_cnt <= heartbeat_cnt + 26'd1;
    end

    // Detect firmware completion: check if magic pass code 0xCAFEF00D is in RAM word 8
    wire fw_passed = (u_ram.mem[8] == 32'hCAFEF00D);

    // Board Diagnostic LED Assignment
    assign led[0] = sonar_busy;                           // LED 0: Sonar Transmitter Busy
    assign led[1] = (stretch_done > 0) || sonar_done;     // LED 1: Sonar TX Done (stretched)
    assign led[2] = sonar_len_error;                      // LED 2: Packet length error
    assign led[3] = sample_valid;                         // LED 3: Carrier BPSK sample valid stream
    assign led[4] = (stretch_load > 0);                   // LED 4: Input Register loaded
    assign led[5] = bus_error;                            // LED 5: Memory bus error
    assign led[6] = trap;                                 // LED 6: CPU Trap indicator
    assign led[7] = fw_passed ? 1'b1 : heartbeat_cnt[24]; // LED 7: Solid ON when firmware passed, else heartbeat

endmodule
