`timescale 1ns / 1ps
module tb_crc8_accelerator;

    reg         clk;
    reg         resetn;
    reg         valid;
    reg         write_en;
    reg  [31:0] addr;
    reg  [31:0] wdata;
    reg  [3:0]  wstrb;
    wire [31:0] rdata;
    wire        ready;

    integer errors;
    integer tests;

    // test message buffer
    reg [7:0] msg [0:31];
    integer   msg_len;

    crc8_accelerator dut (
        .clk(clk), .resetn(resetn),
        .valid(valid), .write_en(write_en),
        .addr(addr), .wdata(wdata), .wstrb(wstrb),
        .rdata(rdata), .ready(ready)
    );

    // 100 MHz clock
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // watchdog
    initial begin
        #5000000;
        $display("FAIL: simulation timeout");
        $display("TEST FAILED");
        $finish;
    end

    //--------------------------------------------------------------
    // Independent reference model: serial LFSR, one BIT at a time.
    // (Different structure from the DUT's byte-wise function.)
    //--------------------------------------------------------------
    function [7:0] ref_update;       // raw CRC, no xorout
        input [7:0] crc_in;
        input [7:0] data;
        integer b;
        reg [7:0] c;
        reg       fb;
        begin
            c = crc_in;
            for (b = 7; b >= 0; b = b - 1) begin
                fb = c[7] ^ data[b];
                c  = {c[6:0], 1'b0};
                if (fb) c = c ^ 8'h07;
            end
            ref_update = c;
        end
    endfunction

    function [7:0] ref_crc8;         // final value for msg[0..len-1]
        input integer len;
        integer k;
        reg [7:0] c;
        begin
            c = 8'h00;
            for (k = 0; k < len; k = k + 1)
                c = ref_update(c, msg[k]);
            ref_crc8 = c ^ 8'h55;
        end
    endfunction

    //--------------------------------------------------------------
    // Bus tasks (drive 1 ns after the clock edge, wait for ready)
    //--------------------------------------------------------------
    task bus_write;
        input [31:0] a;
        input [31:0] d;
        begin
            valid = 1'b1; write_en = 1'b1;
            addr = a; wdata = d; wstrb = 4'hF;
            @(posedge clk); #1;
            while (!ready) begin @(posedge clk); #1; end
            valid = 1'b0; write_en = 1'b0;
            addr = 32'b0; wdata = 32'b0; wstrb = 4'h0;
        end
    endtask

    task bus_read;
        input  [31:0] a;
        output [31:0] d;
        begin
            valid = 1'b1; write_en = 1'b0;
            addr = a; wdata = 32'b0; wstrb = 4'h0;
            @(posedge clk); #1;
            while (!ready) begin @(posedge clk); #1; end
            d = rdata;
            valid = 1'b0;
            addr = 32'b0;
        end
    endtask

    task check;
        input [255:0] name;
        input [31:0]  expected;
        input [31:0]  actual;
        begin
            tests = tests + 1;
            if (expected === actual)
                $display("PASS: %0s  expected=0x%08h actual=0x%08h", name, expected, actual);
            else begin
                $display("FAIL: %0s  expected=0x%08h actual=0x%08h", name, expected, actual);
                errors = errors + 1;
            end
        end
    endtask

    // Reset CRC, feed msg[0..msg_len-1] byte by byte, compare RESULT
    task run_msg;
        input [255:0] name;
        integer i;
        reg [31:0] r;
        reg [31:0] s;
        reg [7:0]  exp;
        begin
            bus_write(32'h04, 32'h1);                 // CONTROL: reset CRC
            bus_read (32'h0C, s);
            check("STATUS DONE cleared after reset", 32'h0, s);
            for (i = 0; i < msg_len; i = i + 1)
                bus_write(32'h00, {24'b0, msg[i]});   // DATA
            exp = ref_crc8(msg_len);
            bus_read(32'h08, r);
            check(name, {24'b0, exp}, r);
                       bus_read(32'h0C, s);
            if (msg_len > 0)
                check("STATUS DONE set after data", 32'h1, s);
            else
                check("STATUS DONE stays 0 for empty msg", 32'h0, s);        
end
    endtask

    task single;
        input [7:0] b;
        begin
            msg[0] = b; msg_len = 1;
            run_msg("single byte");
        end
    endtask

    integer j;
    reg [31:0] rd;

    initial begin
        errors = 0; tests = 0;
        valid = 0; write_en = 0; addr = 0; wdata = 0; wstrb = 0;
        resetn = 1'b0;
        repeat (5) @(posedge clk);
        #1 resetn = 1'b1;
        repeat (2) @(posedge clk);

        // A. Empty / reset state: raw CRC 0x00 ^ 0x55 = 0x55
        $display("--- A: reset state ---");
        bus_read(32'h08, rd);  check("RESULT after reset", 32'h55, rd);
        bus_read(32'h0C, rd);  check("STATUS after reset", 32'h0, rd);
        msg_len = 0;
        run_msg("empty message");

        // B. Single bytes
        $display("--- B: single bytes ---");
        single(8'h00);
        single(8'h01);
        single(8'h55);
        single(8'hAA);
        single(8'hFF);

        // C. "123456789"
        $display("--- C: 123456789 ---");
        msg[0]="1"; msg[1]="2"; msg[2]="3"; msg[3]="4"; msg[4]="5";
        msg[5]="6"; msg[6]="7"; msg[7]="8"; msg[8]="9"; msg_len = 9;
        run_msg("123456789 vs reference model");
        // Published check value for CRC-8/ITU
        bus_read(32'h08, rd);
        check("123456789 vs known check 0xA1", 32'hA1, rd);

        // D. Arbitrary sequences
        $display("--- D: arbitrary sequences ---");
        msg[0]=8'hDE; msg[1]=8'hAD; msg[2]=8'hBE; msg[3]=8'hEF;
        msg[4]=8'h12; msg[5]=8'h34; msg[6]=8'h56; msg_len = 7;
        run_msg("DEADBEEF123456");
        for (j = 0; j < 16; j = j + 1) msg[j] = (j * 37 + 11) & 8'hFF;
        msg_len = 16;
        run_msg("16-byte pattern");

        // E. Read-only registers ignore writes; upper bits zero
        $display("--- E: misc ---");
        bus_write(32'h08, 32'hFFFFFFFF);
        bus_read (32'h08, rd);
        check("RESULT upper bits zero", 32'h0, rd & 32'hFFFFFF00);

        $display("-----------------------------");
        $display("Tests run: %0d, errors: %0d", tests, errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $finish;
    end

    // VCD dump for GTKWave
    initial begin
        $dumpfile("tb_crc8_accelerator.vcd");
        $dumpvars(0, tb_crc8_accelerator);
    end

endmodule