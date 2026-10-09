`timescale 1ns / 1ps
//==============================================================================
// mmio_address_decoder : purely combinational address decoder for PicoRV32
//
//   0x0000_0000 - 0x0000_FFFF : ROM   (64 KB)
//   0x0100_0000 - 0x0100_FFFF : RAM   (64 KB)
//   0x0200_0000 - 0x0200_0FFF : Digital Input Register (4 KB)
//   0x0200_1000 - 0x0200_1FFF : CRC-8 Accelerator      (4 KB)
//   0x0200_2000 - 0x0200_2FFF : BPSK Control           (4 KB)
//
// Behaviour:
//   * A peripheral/memory request is generated only when mem_valid = 1 and
//     the address is inside its range.
//   * write_en = |mem_wstrb for the selected peripheral. wstrb is forced to 0
//     for every peripheral that is not selected.
//   * mem_rdata is the selected block's rdata (AND-OR mux, 0 when nothing
//     is selected -> no latches).
//   * mem_ready comes from the selected block's ready.
//   * ROM is read-only: a write to ROM creates no rom_valid, is ignored, and is
//     terminated immediately (mem_ready = 1, mem_rdata = 0, bus_error = 1).
//   * Unmapped address: no block is selected, mem_rdata = 0, mem_ready = 1
//     (immediate, so the CPU does not hang), bus_error = 1.
//   * No clock or reset: the decoder is purely combinational.
//==============================================================================
module mmio_address_decoder (
    // ---------------- PicoRV32 native memory interface ----------------
    input  wire        mem_valid,
    input  wire [31:0] mem_addr,
    input  wire [31:0] mem_wdata,
    input  wire [3:0]  mem_wstrb,
    output wire        mem_ready,
    output wire [31:0] mem_rdata,

    // ---------------- ROM ----------------
    output wire        rom_valid,
    output wire [31:0] rom_addr,
    output wire [31:0] rom_wdata,
    output wire [3:0]  rom_wstrb,
    input  wire [31:0] rom_rdata,
    input  wire        rom_ready,

    // ---------------- RAM ----------------
    output wire        ram_valid,
    output wire [31:0] ram_addr,
    output wire [31:0] ram_wdata,
    output wire [3:0]  ram_wstrb,
    input  wire [31:0] ram_rdata,
    input  wire        ram_ready,

    // ---------------- Digital Input Register ----------------
    output wire        in_valid,
    output wire        in_write_en,
    output wire [31:0] in_addr,
    output wire [31:0] in_wdata,
    output wire [3:0]  in_wstrb,
    input  wire [31:0] in_rdata,
    input  wire        in_ready,

    // ---------------- CRC-8 Accelerator ----------------
    output wire        crc_valid,
    output wire        crc_write_en,
    output wire [31:0] crc_addr,
    output wire [31:0] crc_wdata,
    output wire [3:0]  crc_wstrb,
    input  wire [31:0] crc_rdata,
    input  wire        crc_ready,

    // ---------------- BPSK Control ----------------
    output wire        bpsk_valid,
    output wire        bpsk_write_en,
    output wire [31:0] bpsk_addr,
    output wire [31:0] bpsk_wdata,
    output wire [3:0]  bpsk_wstrb,
    input  wire [31:0] bpsk_rdata,
    input  wire        bpsk_ready,

    // ---------------- status ----------------
    output wire        bus_error      // unmapped access or write to ROM
);

    // ---- address range hits (aligned ranges, so upper-bit compares are exact)
    wire hit_rom  = (mem_addr[31:16] == 16'h0000);   // 0x0000_0000-0x0000_FFFF
    wire hit_ram  = (mem_addr[31:16] == 16'h0100);   // 0x0100_0000-0x0100_FFFF
    wire hit_in   = (mem_addr[31:12] == 20'h02000);  // 0x0200_0000-0x0200_0FFF
    wire hit_crc  = (mem_addr[31:12] == 20'h02001);  // 0x0200_1000-0x0200_1FFF
    wire hit_bpsk = (mem_addr[31:12] == 20'h02002);  // 0x0200_2000-0x0200_2FFF

    wire is_write = |mem_wstrb;

    // ---- selects (all qualified by mem_valid)
    wire sel_rom_rd = mem_valid & hit_rom & ~is_write;   // ROM read only
    wire sel_ram    = mem_valid & hit_ram;
    wire sel_in     = mem_valid & hit_in;
    wire sel_crc    = mem_valid & hit_crc;
    wire sel_bpsk   = mem_valid & hit_bpsk;

    wire rom_wr_blocked = mem_valid & hit_rom & is_write;
    wire unmapped       = mem_valid & ~(hit_rom | hit_ram | hit_in | hit_crc | hit_bpsk);

    // ---- ROM (read-only)
    assign rom_valid = sel_rom_rd;
    assign rom_addr  = mem_addr;
    assign rom_wdata = 32'h0000_0000;
    assign rom_wstrb = 4'b0000;

    // ---- RAM
    assign ram_valid = sel_ram;
    assign ram_addr  = mem_addr;
    assign ram_wdata = mem_wdata;
    assign ram_wstrb = {4{sel_ram}} & mem_wstrb;

    // ---- Digital Input Register
    assign in_valid    = sel_in;
    assign in_write_en = sel_in & is_write;
    assign in_addr     = mem_addr;
    assign in_wdata    = mem_wdata;
    assign in_wstrb    = {4{sel_in}} & mem_wstrb;

    // ---- CRC-8
    assign crc_valid    = sel_crc;
    assign crc_write_en = sel_crc & is_write;
    assign crc_addr     = mem_addr;
    assign crc_wdata    = mem_wdata;
    assign crc_wstrb    = {4{sel_crc}} & mem_wstrb;

    // ---- BPSK Control
    assign bpsk_valid    = sel_bpsk;
    assign bpsk_write_en = sel_bpsk & is_write;
    assign bpsk_addr     = mem_addr;
    assign bpsk_wdata    = mem_wdata;
    assign bpsk_wstrb    = {4{sel_bpsk}} & mem_wstrb;

    // ---- read data mux (AND-OR; zero when nothing selected)
    assign mem_rdata = ({32{sel_rom_rd}} & rom_rdata)
                     | ({32{sel_ram}}    & ram_rdata)
                     | ({32{sel_in}}     & in_rdata)
                     | ({32{sel_crc}}    & crc_rdata)
                     | ({32{sel_bpsk}}   & bpsk_rdata);

    // ---- ready mux; blocked/unmapped accesses finish immediately
    assign mem_ready = (sel_rom_rd & rom_ready)
                     | (sel_ram    & ram_ready)
                     | (sel_in     & in_ready)
                     | (sel_crc    & crc_ready)
                     | (sel_bpsk   & bpsk_ready)
                     | rom_wr_blocked
                     | unmapped;

    assign bus_error = rom_wr_blocked | unmapped;

endmodule