`timescale 1ns / 1ps
module tb_input_register;

    reg         clk;
    reg         resetn;
    reg         valid;
    reg         write_en;
    reg  [31:0] addr;
    reg  [31:0] wdata;
    reg  [3:0]  wstrb;
    wire [31:0] rdata;
    wire        ready;
    wire [7:0]  data_out;
    wire        data_valid;

    integer errors;
    integer tests;
    integer pulse_cnt;         // counts clock cycles with data_valid = 1
    integer expected_pulses;   // number of LOAD writes the testbench issued
    integer pulse_before;

    // Testbench's own model of the register
    reg [7:0] exp_data;
    reg       exp_status;

    reg [7:0] vals [0:5];
    reg [31:0] r;
    integer k;

    digital_input_register dut (
        .clk(clk), .resetn(resetn),
        .valid(valid), .write_en(write_en),
        .addr(addr), .wdata(wdata), .wstrb(wstrb),
        .rdata(rdata), .ready(ready),
        .data_out(data_out), .data_valid(data_valid)
    );

    initial begin
        $dumpfile("tb_input_register.vcd");
        $dumpvars(0, tb_input_register);
    end

    // 100 MHz clock
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // watchdog
    initial begin
        #2000000;
        $display("FAIL: simulation timeout");
        $display("====================================");
        $display("INPUT REGISTER TEST FAILED");
        $display("====================================");
        $finish;
    end

    // Pulse counter: samples data_valid at every clock edge
    initial pulse_cnt = 0;
    always @(posedge clk) begin
        if (data_valid === 1'b1) pulse_cnt = pulse_cnt + 1;
    end

    //--------------------------------------------------------------
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

    // Bus write with explicit wstrb. Drives 1 ns after a clock edge.
    task bus_write_s;
        input [31:0] a;
        input [31:0] d;
        input [3:0]  s;
        begin
            valid = 1'b1; write_en = 1'b1;
            addr = a; wdata = d; wstrb = s;
            @(posedge clk); #1;
            while (!ready) begin @(posedge clk); #1; end
            valid = 1'b0; write_en = 1'b0;
            addr = 32'b0; wdata = 32'b0; wstrb = 4'h0;
            @(posedge clk); #1;
            check("ready drops after write", 32'h0, {31'b0, ready});
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
            valid = 1'b0; addr = 32'b0;
            @(posedge clk); #1;
            check("ready drops after read", 32'h0, {31'b0, ready});
        end
    endtask

    // Write DATA (full strobe) and update the model
    task write_data;
        input [7:0] v;
        begin
            bus_write_s(32'h00, {24'b0, v}, 4'hF);
            exp_data   = v;
            exp_status = 1'b0;
            check("data_out after DATA write", {24'b0, exp_data}, {24'b0, data_out});
        end
    endtask

    task read_data_check;
        input [255:0] name;
        begin
            bus_read(32'h00, r);
            check(name, {24'b0, exp_data}, r);
        end
    endtask

    task read_status_check;
        input [255:0] name;
        begin
            bus_read(32'h08, r);
            check(name, {31'b0, exp_status}, r);
        end
    endtask

    // LOAD: write CONTROL=1 and verify data_valid is high for exactly one cycle
    task do_load;
        begin
            pulse_before = pulse_cnt;
            valid = 1'b1; write_en = 1'b1;
            addr = 32'h04; wdata = 32'h1; wstrb = 4'hF;
            @(posedge clk); #1;                       // access edge
            check("LOAD: ready high",      32'h1, {31'b0, ready});
            check("LOAD: data_valid high", 32'h1, {31'b0, data_valid});
            valid = 1'b0; write_en = 1'b0;
            addr = 32'b0; wdata = 32'b0; wstrb = 4'h0;
            @(posedge clk); #1;                       // one cycle later
            check("LOAD: data_valid low after 1 cycle", 32'h0, {31'b0, data_valid});
            check("LOAD: ready low",                    32'h0, {31'b0, ready});
            repeat (3) @(posedge clk);
            #1;
            check("LOAD: exactly one pulse", 32'h1, pulse_cnt - pulse_before);
            check("LOAD: data_out unchanged", {24'b0, exp_data}, {24'b0, data_out});
            exp_status      = 1'b1;
            expected_pulses = expected_pulses + 1;
        end
    endtask

    task apply_reset;
        begin
            valid = 1'b0; write_en = 1'b0;
            resetn = 1'b0;
            repeat (3) @(posedge clk);
            #1;
            resetn = 1'b1;
            @(posedge clk); #1;
            exp_data = 8'h00; exp_status = 1'b0;
        end
    endtask

    //--------------------------------------------------------------
    initial begin
        errors = 0; tests = 0; expected_pulses = 0;
        exp_data = 8'h00; exp_status = 1'b0;
        valid = 0; write_en = 0; addr = 0; wdata = 0; wstrb = 0;
        vals[0] = 8'h00; vals[1] = 8'h01; vals[2] = 8'h55;
        vals[3] = 8'hAA; vals[4] = 8'hFF; vals[5] = 8'h3C;
        resetn = 1'b0;
        repeat (5) @(posedge clk);
        #1 resetn = 1'b1;
        repeat (2) @(posedge clk);
        #1;

        // 1. Reset state
        $display("--- 1: reset state ---");
        check("data_out after reset",   32'h0, {24'b0, data_out});
        check("data_valid after reset", 32'h0, {31'b0, data_valid});
        check("ready after reset",      32'h0, {31'b0, ready});
        read_data_check("DATA read after reset");
        read_status_check("STATUS after reset");

        // 2. Basic: write 0xB2, read back, LOAD
        $display("--- 2: 0xB2 ---");
        write_data(8'hB2);
        check("data_valid not set by DATA write", 32'h0, {31'b0, data_valid});
        read_data_check("DATA read back 0xB2");
        read_status_check("STATUS before LOAD");
        do_load;
        check("data_out == 0xB2", 32'hB2, {24'b0, data_out});
        read_status_check("STATUS after LOAD");
        read_data_check("DATA still 0xB2 after LOAD");

        // 3. Additional values
        $display("--- 3: more values ---");
        for (k = 0; k < 6; k = k + 1) begin
            write_data(vals[k]);
            read_data_check("DATA read back");
            read_status_check("STATUS cleared by new DATA");
            do_load;
            read_status_check("STATUS set by LOAD");
        end

        // 4. Byte-lane and data-width handling
        $display("--- 4: wstrb / wdata width ---");
        bus_write_s(32'h00, 32'hFFFFFF5A, 4'hF);          // upper bits ignored
        exp_data = 8'h5A; exp_status = 1'b0;
        read_data_check("upper wdata bits ignored");
        bus_write_s(32'h00, 32'h000000EE, 4'h0);          // no strobe: ignored
        read_data_check("wstrb=0 write ignored");
        bus_write_s(32'h00, 32'h0000EE00, 4'b0010);       // only byte 1: ignored
        read_data_check("wstrb=0010 write ignored");
        bus_write_s(32'h00, 32'h00000077, 4'b0001);       // byte 0: accepted
        exp_data = 8'h77;
        read_data_check("wstrb=0001 write accepted");
        check("data_out == 0x77", 32'h77, {24'b0, data_out});

        // 5. CONTROL writes that must NOT load
        $display("--- 5: CONTROL without LOAD ---");
        do_load;                                           // status = 1
        bus_write_s(32'h04, 32'hFFFFFFFE, 4'hF);          // bit0 = 0
        bus_write_s(32'h04, 32'h00000001, 4'h0);          // no strobe
        read_status_check("STATUS unchanged, no extra LOAD");
        bus_read(32'h04, r);
        check("CONTROL reads 0", 32'h0, r);

        // 6. Invalid addresses
        $display("--- 6: invalid addresses ---");
        write_data(8'h3C);
        do_load;
        bus_write_s(32'h0C, 32'hFFFFFFFF, 4'hF);
        bus_write_s(32'h10, 32'hFFFFFFFF, 4'hF);
        bus_write_s(32'h40, 32'hFFFFFFFF, 4'hF);
        bus_write_s(32'hFC, 32'hFFFFFFFF, 4'hF);
        bus_write_s(32'h05, 32'hFFFFFFFF, 4'hF);          // misaligned
        bus_write_s(32'h08, 32'hFFFFFFFF, 4'hF);          // STATUS is read-only
        check("data_out unchanged by invalid writes", {24'b0, exp_data}, {24'b0, data_out});
        read_data_check("DATA unchanged by invalid writes");
        read_status_check("STATUS unchanged by invalid writes");
        bus_read(32'h0C, r); check("read 0x0C returns 0", 32'h0, r);
        bus_read(32'h10, r); check("read 0x10 returns 0", 32'h0, r);
        bus_read(32'h40, r); check("read 0x40 returns 0", 32'h0, r);
        read_data_check("DATA unchanged by invalid reads");

        // 7. Reset behaviour with stored data
        $display("--- 7: reset with data stored ---");
        write_data(8'hFF);
        do_load;
        apply_reset;
        check("data_out cleared by reset",   32'h0, {24'b0, data_out});
        check("data_valid low after reset",  32'h0, {31'b0, data_valid});
        check("ready low after reset",       32'h0, {31'b0, ready});
        read_data_check("DATA cleared by reset");
        read_status_check("STATUS cleared by reset");

        // 8. Normal operation resumes after reset
        $display("--- 8: after reset ---");
        write_data(8'hB2);
        read_data_check("DATA after reset+write");
        do_load;

        // Global pulse check: no stray data_valid pulses anywhere
        repeat (5) @(posedge clk);
        #1;
        check("total data_valid pulses == LOAD writes", expected_pulses, pulse_cnt);

        $display("-----------------------------");
        $display("Tests run: %0d, errors: %0d", tests, errors);
        if (errors == 0) begin
            $display("====================================");
            $display("INPUT REGISTER TEST PASSED");
            $display("====================================");
        end else begin
            $display("====================================");
            $display("INPUT REGISTER TEST FAILED");
            $display("====================================");
        end
        $finish;
    end
initial begin
    $dumpfile("tb_input_register.vcd");
    $dumpvars(0, tb_input_register);
end

endmodule