`timescale 1ns / 1ps
module tb_mmio_address_decoder;

    // PicoRV32 side
    reg         mem_valid;
    reg  [31:0] mem_addr;
    reg  [31:0] mem_wdata;
    reg  [3:0]  mem_wstrb;
    wire        mem_ready;
    wire [31:0] mem_rdata;

    // peripheral models (driven by the testbench)
    reg  [31:0] rom_rdata, ram_rdata, in_rdata, crc_rdata, bpsk_rdata;
    reg         rom_ready, ram_ready, in_ready, crc_ready, bpsk_ready;

    // decoder outputs
    wire        rom_valid, ram_valid, in_valid, crc_valid, bpsk_valid;
    wire [31:0] rom_addr,  ram_addr,  in_addr,  crc_addr,  bpsk_addr;
    wire [31:0] rom_wdata, ram_wdata, in_wdata, crc_wdata, bpsk_wdata;
    wire [3:0]  rom_wstrb, ram_wstrb, in_wstrb, crc_wstrb, bpsk_wstrb;
    wire        in_write_en, crc_write_en, bpsk_write_en;
    wire        bus_error;

    localparam [31:0] ROM_D  = 32'hA0A0_0001;
    localparam [31:0] RAM_D  = 32'hB0B0_0002;
    localparam [31:0] IN_D   = 32'hC0C0_0003;
    localparam [31:0] CRC_D  = 32'hD0D0_0004;
    localparam [31:0] BPSK_D = 32'hE0E0_0005;

    integer errors, tests, pat_cur, seed, i, sel;
    reg     verbose;
    reg [31:0] ra;
    reg [3:0]  rws;
    reg        rv;

    mmio_address_decoder dut (
        .mem_valid(mem_valid), .mem_addr(mem_addr), .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb), .mem_ready(mem_ready), .mem_rdata(mem_rdata),

        .rom_valid(rom_valid), .rom_addr(rom_addr), .rom_wdata(rom_wdata),
        .rom_wstrb(rom_wstrb), .rom_rdata(rom_rdata), .rom_ready(rom_ready),

        .ram_valid(ram_valid), .ram_addr(ram_addr), .ram_wdata(ram_wdata),
        .ram_wstrb(ram_wstrb), .ram_rdata(ram_rdata), .ram_ready(ram_ready),

        .in_valid(in_valid), .in_write_en(in_write_en), .in_addr(in_addr),
        .in_wdata(in_wdata), .in_wstrb(in_wstrb), .in_rdata(in_rdata), .in_ready(in_ready),

        .crc_valid(crc_valid), .crc_write_en(crc_write_en), .crc_addr(crc_addr),
        .crc_wdata(crc_wdata), .crc_wstrb(crc_wstrb), .crc_rdata(crc_rdata), .crc_ready(crc_ready),

        .bpsk_valid(bpsk_valid), .bpsk_write_en(bpsk_write_en), .bpsk_addr(bpsk_addr),
        .bpsk_wdata(bpsk_wdata), .bpsk_wstrb(bpsk_wstrb), .bpsk_rdata(bpsk_rdata),
        .bpsk_ready(bpsk_ready),

        .bus_error(bus_error)
    );

    initial begin
        $dumpfile("tb_mmio_address_decoder.vcd");
        $dumpvars(0, tb_mmio_address_decoder);
    end

    //--------------------------------------------------------------
    // Independent range model: 0 unmapped, 1 ROM, 2 RAM, 3 IN, 4 CRC, 5 BPSK
    //--------------------------------------------------------------
    function integer kind_of;
        input [31:0] a;
        begin
            if      (a <= 32'h0000FFFF)                          kind_of = 1;
            else if (a >= 32'h01000000 && a <= 32'h0100FFFF)     kind_of = 2;
            else if (a >= 32'h02000000 && a <= 32'h02000FFF)     kind_of = 3;
            else if (a >= 32'h02001000 && a <= 32'h02001FFF)     kind_of = 4;
            else if (a >= 32'h02002000 && a <= 32'h02002FFF)     kind_of = 5;
            else                                                 kind_of = 0;
        end
    endfunction

    task chk;
        input [255:0] name;
        input [31:0]  expected;
        input [31:0]  actual;
        begin
            tests = tests + 1;
            if (expected !== actual) begin
                errors = errors + 1;
                $display("FAIL: %0s addr=%08h wstrb=%h valid=%b ready_pattern=%0d expected=%08h actual=%08h",
                         name, mem_addr, mem_wstrb, mem_valid, pat_cur, expected, actual);
            end
        end
    endtask

    // One access, repeated with 4 ready patterns:
    //   0: all ready = 0        1: all ready = 1
    //   2: only selected = 1    3: all except selected = 1
    task access;
        input        v;
        input [31:0] a;
        input [3:0]  ws;
        integer pat, k, err0;
        reg ev_rom, ev_ram, ev_in, ev_crc, ev_bpsk;
        reg e_ready, e_err;
        reg [31:0] e_rdata;
        begin
            err0 = errors;
            k = kind_of(a);
            for (pat = 0; pat < 4; pat = pat + 1) begin
                pat_cur   = pat;
                mem_valid = v;
                mem_addr  = a;
                mem_wdata = a ^ 32'h5A5A_C3C3;
                mem_wstrb = ws;
                rom_ready  = (pat==1) || (pat==2 && k==1) || (pat==3 && k!=1);
                ram_ready  = (pat==1) || (pat==2 && k==2) || (pat==3 && k!=2);
                in_ready   = (pat==1) || (pat==2 && k==3) || (pat==3 && k!=3);
                crc_ready  = (pat==1) || (pat==2 && k==4) || (pat==3 && k!=4);
                bpsk_ready = (pat==1) || (pat==2 && k==5) || (pat==3 && k!=5);
                #1;

                // expected values
                ev_rom  = v && (k==1) && (ws==4'b0000);
                ev_ram  = v && (k==2);
                ev_in   = v && (k==3);
                ev_crc  = v && (k==4);
                ev_bpsk = v && (k==5);
                e_err   = v && ((k==0) || (k==1 && ws!=4'b0000));

                e_rdata = 32'h0;
                if (v) begin
                    case (k)
                        1: if (ws == 4'b0000) e_rdata = ROM_D;
                        2: e_rdata = RAM_D;
                        3: e_rdata = IN_D;
                        4: e_rdata = CRC_D;
                        5: e_rdata = BPSK_D;
                        default: e_rdata = 32'h0;
                    endcase
                end

                if (!v)            e_ready = 1'b0;
                else if (e_err)    e_ready = 1'b1;
                else case (k)
                    1: e_ready = rom_ready;
                    2: e_ready = ram_ready;
                    3: e_ready = in_ready;
                    4: e_ready = crc_ready;
                    5: e_ready = bpsk_ready;
                    default: e_ready = 1'b1;
                endcase

                // requests
                chk("rom_valid",  ev_rom,  rom_valid);
                chk("ram_valid",  ev_ram,  ram_valid);
                chk("in_valid",   ev_in,   in_valid);
                chk("crc_valid",  ev_crc,  crc_valid);
                chk("bpsk_valid", ev_bpsk, bpsk_valid);

                // write enables (0 unless selected AND a write)
                chk("in_write_en",   ev_in   && (ws != 0), in_write_en);
                chk("crc_write_en",  ev_crc  && (ws != 0), crc_write_en);
                chk("bpsk_write_en", ev_bpsk && (ws != 0), bpsk_write_en);

                // byte enables (0 unless selected)
                chk("rom_wstrb",  4'b0000,            rom_wstrb);
                chk("ram_wstrb",  ev_ram  ? ws : 4'b0, ram_wstrb);
                chk("in_wstrb",   ev_in   ? ws : 4'b0, in_wstrb);
                chk("crc_wstrb",  ev_crc  ? ws : 4'b0, crc_wstrb);
                chk("bpsk_wstrb", ev_bpsk ? ws : 4'b0, bpsk_wstrb);

                // address / write data pass-through
                chk("rom_addr",   a, rom_addr);
                chk("ram_addr",   a, ram_addr);
                chk("in_addr",    a, in_addr);
                chk("crc_addr",   a, crc_addr);
                chk("bpsk_addr",  a, bpsk_addr);
                chk("rom_wdata",  32'h0, rom_wdata);
                chk("ram_wdata",  mem_wdata, ram_wdata);
                chk("in_wdata",   mem_wdata, in_wdata);
                chk("crc_wdata",  mem_wdata, crc_wdata);
                chk("bpsk_wdata", mem_wdata, bpsk_wdata);

                // routing back to the CPU
                chk("mem_rdata",  e_rdata, mem_rdata);
                chk("mem_ready",  e_ready, mem_ready);
                chk("bus_error",  e_err,   bus_error);
            end
            if (verbose && errors == err0)
                $display("PASS: valid=%b addr=%08h wstrb=%h kind=%0d", v, a, ws, k);
        end
    endtask

    // read, full write, partial write, and idle at the same address
    task access_all;
        input [31:0] a;
        begin
            access(1'b1, a, 4'b0000);
            access(1'b1, a, 4'b1111);
            access(1'b1, a, 4'b0101);
            access(1'b0, a, 4'b0000);
            access(1'b0, a, 4'b1111);
        end
    endtask

    initial begin
        errors = 0; tests = 0; pat_cur = 0; seed = 7; verbose = 1'b1;
        mem_valid = 0; mem_addr = 0; mem_wdata = 0; mem_wstrb = 0;
        rom_rdata = ROM_D; ram_rdata = RAM_D; in_rdata = IN_D;
        crc_rdata = CRC_D; bpsk_rdata = BPSK_D;
        rom_ready = 0; ram_ready = 0; in_ready = 0; crc_ready = 0; bpsk_ready = 0;
        #5;

        $display("--- ROM 0x00000000-0x0000FFFF ---");
        access_all(32'h00000000); access_all(32'h00000004);
        access_all(32'h00008000); access_all(32'h0000FFFC); access_all(32'h0000FFFF);

        $display("--- gap after ROM ---");
        access_all(32'h00010000); access_all(32'h00FFFFFC); access_all(32'h00FFFFFF);

        $display("--- RAM 0x01000000-0x0100FFFF ---");
        access_all(32'h01000000); access_all(32'h01000004);
        access_all(32'h01008000); access_all(32'h0100FFFC); access_all(32'h0100FFFF);

        $display("--- gap around RAM ---");
        access_all(32'h01010000); access_all(32'h01FFFFFC);

        $display("--- Digital Input Register 0x02000000-0x02000FFF ---");
        access_all(32'h02000000); access_all(32'h02000004); access_all(32'h02000008);
        access_all(32'h02000800); access_all(32'h02000FFC); access_all(32'h02000FFF);

        $display("--- CRC-8 0x02001000-0x02001FFF ---");
        access_all(32'h02001000); access_all(32'h02001004); access_all(32'h02001008);
        access_all(32'h0200100C); access_all(32'h02001800); access_all(32'h02001FFC);
        access_all(32'h02001FFF);

        $display("--- BPSK Control 0x02002000-0x02002FFF ---");
        access_all(32'h02002000); access_all(32'h02002004); access_all(32'h02002800);
        access_all(32'h02002FFC); access_all(32'h02002FFF);

        $display("--- unmapped ---");
        access_all(32'h02003000); access_all(32'h02FFFFFC); access_all(32'h03000000);
        access_all(32'h10000000); access_all(32'h40000000); access_all(32'h80000000);
        access_all(32'hFFFFFFFC); access_all(32'hFFFFFFFF);

        $display("--- 4000 random accesses (only failures printed) ---");
        verbose = 1'b0;
        for (i = 0; i < 4000; i = i + 1) begin
            ra  = $random(seed);
            rws = $random(seed);
            rv  = $random(seed);
            if (i % 5 == 0) rv = 1'b1;
            if (i % 3 == 0) rws = 4'b0000;
            sel = i % 8;
            case (sel)
                0: ra = ra & 32'h0000FFFF;
                1: ra = 32'h01000000 | (ra & 32'h0000FFFF);
                2: ra = 32'h02000000 | (ra & 32'h00000FFF);
                3: ra = 32'h02001000 | (ra & 32'h00000FFF);
                4: ra = 32'h02002000 | (ra & 32'h00000FFF);
                5: ra = ra;                                    // fully random
                6: ra = 32'h02003000 | (ra & 32'h00000FFF);    // just past BPSK
                7: ra = 32'h00010000 | (ra & 32'h00FFFFFF);    // just past ROM
            endcase
            access(rv, ra, rws);
        end
        verbose = 1'b1;

        $display("-----------------------------");
        $display("Checks run: %0d, errors: %0d", tests, errors);
        if (errors == 0) begin
            $display("====================================");
            $display("MMIO ADDRESS DECODER TEST PASSED");
            $display("====================================");
        end else begin
            $display("====================================");
            $display("MMIO ADDRESS DECODER TEST FAILED");
            $display("====================================");
        end
        $finish;
    end

endmodule