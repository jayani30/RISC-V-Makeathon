`timescale 1ns / 1ps
//==============================================================================
// picorv32_top : Top-level SoC integrating PicoRV32 RISC-V CPU with:
//   - 64 KB Boot ROM            (0x0000_0000 - 0x0000_FFFF)
//   - 64 KB System RAM          (0x0100_0000 - 0x0100_FFFF)
//   - Digital Input Register    (0x0200_0000 - 0x0200_0FFF)
//   - CRC-8 Hardware Accelerator(0x0200_1000 - 0x0200_1FFF)
//   - BPSK Sonar Controller     (0x0200_2000 - 0x0200_2FFF)
//   - Standalone BPSK Modulator
//==============================================================================
module picorv32_top #(
    parameter [31:0] PROGADDR_RESET = 32'h0000_0000,
    parameter [31:0] STACKADDR      = 32'h0101_0000,
    parameter        ROM_INIT_FILE  = "rom_test.hex",
    parameter        BARREL_SHIFTER = 1,
    parameter        MAX_PAYLOAD_LEN= 32,
    parameter        PHASE_STEP     = 16,
    parameter        CYCLES_PER_BIT = 1
)(
    input  wire                      clk,
    input  wire                      resetn,

    // CPU status
    output wire                      trap,
    output wire                      bus_error,

    // Sonar TX BPSK output stream
    output wire signed [7:0]         bpsk_sample,
    output wire                      sample_valid,
    output wire                      sonar_busy,
    output wire                      sonar_done,
    output wire                      sonar_len_error,

    // Standalone direct BPSK modulator outputs
    output wire signed [15:0]        bpsk_mod_out,
    output wire                      bpsk_mod_valid,

    // Diagnostic / debug ports
    output wire [7:0]                inreg_data_out,
    output wire                      inreg_data_load,
    output wire [3:0]                sonar_state_dbg
);

    //==========================================================================
    // PicoRV32 Native Memory Bus Wires
    //==========================================================================
    wire        mem_valid;
    wire        mem_instr;
    wire        mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wstrb;
    wire [31:0] mem_rdata;

    //==========================================================================
    // MMIO Address Decoder Wires
    //==========================================================================
    // ROM
    wire        rom_valid;
    wire [31:0] rom_addr;
    wire [31:0] rom_wdata;
    wire [3:0]  rom_wstrb;
    wire [31:0] rom_rdata;
    wire        rom_ready;

    // RAM (CPU side of the arbiter)
    wire        cpu_ram_valid;
    wire [31:0] cpu_ram_addr;
    wire [31:0] cpu_ram_wdata;
    wire [3:0]  cpu_ram_wstrb;
    wire [31:0] cpu_ram_rdata;
    wire        cpu_ram_ready;

    // Digital Input Register (connected to sonar_tx_top slave)
    wire        in_valid;
    wire        in_write_en;
    wire [31:0] in_addr;
    wire [31:0] in_wdata;
    wire [3:0]  in_wstrb;
    wire [31:0] in_rdata;
    wire        in_ready;

    // CRC-8 Accelerator
    wire        crc_valid;
    wire        crc_write_en;
    wire [31:0] crc_addr;
    wire [31:0] crc_wdata;
    wire [3:0]  crc_wstrb;
    wire [31:0] crc_rdata;
    wire        crc_ready;

    // BPSK Control
    wire        bpsk_valid;
    wire        bpsk_write_en;
    wire [31:0] bpsk_addr;
    wire [31:0] bpsk_wdata;
    wire [3:0]  bpsk_wstrb;
    wire [31:0] bpsk_rdata;
    wire        bpsk_ready;

    //==========================================================================
    // Shared RAM & Sonar Master Wires
    //==========================================================================
    wire        son_rm_valid;
    wire [31:0] son_rm_addr;
    wire [31:0] son_rm_rdata;
    wire        son_rm_ready;
    wire        son_rm_busy;

    wire        ram_valid;
    wire [31:0] ram_addr;
    wire [31:0] ram_wdata;
    wire [3:0]  ram_wstrb;
    wire [31:0] ram_rdata;
    wire        ram_ready;

    //==========================================================================
    // BPSK Control <-> Sonar TX & Modulator Wires
    //==========================================================================
    wire        c_sonar_start;
    wire [7:0]  c_sonar_payload_len;
    wire        c_sonar_src_sel;
    wire [15:0] c_sonar_ram_base_addr;

    wire        c_direct_data_in;
    wire        c_direct_data_valid;
    wire        c_direct_sample_en;

    //==========================================================================
    // 1. PicoRV32 RISC-V CPU Core
    //==========================================================================
    picorv32 #(
        .PROGADDR_RESET     (PROGADDR_RESET),
        .STACKADDR          (STACKADDR),
        .ENABLE_COUNTERS    (1),
        .ENABLE_COUNTERS64  (1),
        .ENABLE_REGS_16_31  (1),
        .ENABLE_REGS_DUALPORT(1),
        .LATCHED_MEM_RDATA  (0),
        .TWO_STAGE_SHIFT    (1),
        .BARREL_SHIFTER     (BARREL_SHIFTER),
        .TWO_CYCLE_COMPARE  (0),
        .TWO_CYCLE_ALU      (0),
        .COMPRESSED_ISA     (0),
        .CATCH_MISALIGN     (1),
        .CATCH_ILLINSN      (1),
        .ENABLE_PCPI        (0),
        .ENABLE_MUL         (0),
        .ENABLE_FAST_MUL    (0),
        .ENABLE_DIV         (0),
        .ENABLE_IRQ         (0),
        .ENABLE_TRACE       (0),
        .REGS_INIT_ZERO     (1)
    ) u_cpu (
        .clk         (clk),
        .resetn      (resetn),
        .trap        (trap),

        .mem_valid   (mem_valid),
        .mem_instr   (mem_instr),
        .mem_ready   (mem_ready),
        .mem_addr    (mem_addr),
        .mem_wdata   (mem_wdata),
        .mem_wstrb   (mem_wstrb),
        .mem_rdata   (mem_rdata),

        // Unused optional interfaces
        .mem_la_read (),
        .mem_la_write(),
        .mem_la_addr (),
        .mem_la_wdata(),
        .mem_la_wstrb(),
        .pcpi_valid  (),
        .pcpi_insn   (),
        .pcpi_rs1    (),
        .pcpi_rs2    (),
        .pcpi_wr     (1'b0),
        .pcpi_rd     (32'h0),
        .pcpi_wait   (1'b0),
        .pcpi_ready  (1'b0),
        .irq         (32'h0),
        .eoi         (),
        .trace_valid (),
        .trace_data  ()
    );

    //==========================================================================
    // 2. Memory-Mapped Address Decoder
    //==========================================================================
    mmio_address_decoder u_decoder (
        .mem_valid    (mem_valid),
        .mem_addr     (mem_addr),
        .mem_wdata    (mem_wdata),
        .mem_wstrb    (mem_wstrb),
        .mem_ready    (mem_ready),
        .mem_rdata    (mem_rdata),

        // ROM
        .rom_valid    (rom_valid),
        .rom_addr     (rom_addr),
        .rom_wdata    (rom_wdata),
        .rom_wstrb    (rom_wstrb),
        .rom_rdata    (rom_rdata),
        .rom_ready    (rom_ready),

        // RAM
        .ram_valid    (cpu_ram_valid),
        .ram_addr     (cpu_ram_addr),
        .ram_wdata    (cpu_ram_wdata),
        .ram_wstrb    (cpu_ram_wstrb),
        .ram_rdata    (cpu_ram_rdata),
        .ram_ready    (cpu_ram_ready),

        // Digital Input Register
        .in_valid     (in_valid),
        .in_write_en  (in_write_en),
        .in_addr      (in_addr),
        .in_wdata     (in_wdata),
        .in_wstrb     (in_wstrb),
        .in_rdata     (in_rdata),
        .in_ready     (in_ready),

        // CRC-8 Accelerator
        .crc_valid    (crc_valid),
        .crc_write_en (crc_write_en),
        .crc_addr     (crc_addr),
        .crc_wdata    (crc_wdata),
        .crc_wstrb    (crc_wstrb),
        .crc_rdata    (crc_rdata),
        .crc_ready    (crc_ready),

        // BPSK Control
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
    // 3. Boot ROM (64 KB)
    //==========================================================================
    rom #(
        .WORDS        (16384),
        .INIT_FILE    (ROM_INIT_FILE)
    ) u_rom (
        .clk          (clk),
        .resetn       (resetn),
        .valid        (rom_valid),
        .addr         (rom_addr),
        .rdata        (rom_rdata),
        .ready        (rom_ready)
    );

    //==========================================================================
    // 4. Sonar RAM Arbiter (CPU vs Sonar TX)
    //==========================================================================
    sonar_ram_arbiter u_ram_arbiter (
        .sel_sonar    (son_rm_busy),

        // CPU port
        .cpu_valid    (cpu_ram_valid),
        .cpu_addr     (cpu_ram_addr),
        .cpu_wdata    (cpu_ram_wdata),
        .cpu_wstrb    (cpu_ram_wstrb),
        .cpu_rdata    (cpu_ram_rdata),
        .cpu_ready    (cpu_ram_ready),

        // Sonar TX port
        .son_valid    (son_rm_valid),
        .son_addr     (son_rm_addr),
        .son_rdata    (son_rm_rdata),
        .son_ready    (son_rm_ready),

        // Physical RAM port
        .ram_valid    (ram_valid),
        .ram_addr     (ram_addr),
        .ram_wdata    (ram_wdata),
        .ram_wstrb    (ram_wstrb),
        .ram_rdata    (ram_rdata),
        .ram_ready    (ram_ready)
    );

    //==========================================================================
    // 5. System RAM (64 KB)
    //==========================================================================
    ram #(
        .WORDS        (16384)
    ) u_ram (
        .clk          (clk),
        .resetn       (resetn),
        .valid        (ram_valid),
        .addr         (ram_addr),
        .wdata        (ram_wdata),
        .wstrb        (ram_wstrb),
        .rdata        (ram_rdata),
        .ready        (ram_ready)
    );

    //==========================================================================
    // 6. CRC-8 Accelerator (MMIO peripheral at 0x0200_1000)
    //==========================================================================
    crc8_accelerator u_crc_accel (
        .clk          (clk),
        .resetn       (resetn),
        .valid        (crc_valid),
        .write_en     (crc_write_en),
        .addr         (crc_addr),
        .wdata        (crc_wdata),
        .wstrb        (crc_wstrb),
        .rdata        (crc_rdata),
        .ready        (crc_ready)
    );

    //==========================================================================
    // 7. BPSK Control Register Block (MMIO at 0x0200_2000)
    //==========================================================================
    bpsk_ctrl u_bpsk_ctrl (
        .clk                 (clk),
        .resetn              (resetn),

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
    // 8. Sonar Transmitter Top Module
    //    (Contains: Digital Input Register, Controller, CRC-8, Sine LUT, BPSK Gen)
    //==========================================================================
    sonar_tx_top #(
        .MAX_LEN             (MAX_PAYLOAD_LEN),
        .PHASE_STEP          (PHASE_STEP),
        .CYCLES_PER_BIT      (CYCLES_PER_BIT),
        .BIT1_NEGATIVE       (0),
        .RAM_BASE            (32'h0100_0000)
    ) u_sonar_tx (
        .clk                 (clk),
        .resetn              (resetn),

        .start               (c_sonar_start),
        .payload_len         (c_sonar_payload_len),
        .src_sel             (c_sonar_src_sel),
        .ram_base_addr       (c_sonar_ram_base_addr),
        .busy                (sonar_busy),
        .done                (sonar_done),
        .len_error           (sonar_len_error),

        .bpsk_sample         (bpsk_sample),
        .sample_valid        (sample_valid),

        // Digital input register slave (MMIO 0x0200_0000)
        .in_valid            (in_valid),
        .in_write_en         (in_write_en),
        .in_addr             (in_addr),
        .in_wdata            (in_wdata),
        .in_wstrb            (in_wstrb),
        .in_rdata            (in_rdata),
        .in_ready            (in_ready),
        .in_load             (inreg_data_load),

        // RAM read master
        .rm_valid            (son_rm_valid),
        .rm_addr             (son_rm_addr),
        .rm_rdata            (son_rm_rdata),
        .rm_ready            (son_rm_ready),
        .rm_busy             (son_rm_busy),

        // Debug outputs
        .dbg_bit_valid       (),
        .dbg_bit_data        (),
        .dbg_bit_ready       (),
        .dbg_bit_last        (),
        .dbg_state           (sonar_state_dbg)
    );

    // Output input register stored byte for debug
    assign inreg_data_out = in_rdata[7:0];

    //==========================================================================
    // 9. Standalone Direct BPSK Modulator (bpsk.v)
    //==========================================================================
    bpsk_modulator #(
        .OUTPUT_WIDTH        (16),
        .AMPLITUDE           (1000)
    ) u_bpsk_mod (
        .clk                 (clk),
        .rst                 (!resetn),
        .data_in             (c_direct_data_in),
        .data_valid          (c_direct_data_valid),
        .sample_en           (c_direct_sample_en),
        .bpsk_out            (bpsk_mod_out),
        .out_valid           (bpsk_mod_valid)
    );

endmodule
