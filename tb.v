`timescale 1ns / 1ps

module tb_async_fifo;

    reg        clk_wr = 1'b0, clk_rd = 1'b0;
    reg        rst;
    reg        wr_en, rd_en;
    reg  [7:0] d_in;
    wire [7:0] d_out;
    wire       full, almost_full, empty, almost_empty;

    async_fifo dut (
        .clk_wr(clk_wr), .clk_rd(clk_rd), .rst(rst),
        .wr_en(wr_en), .d_in(d_in), .rd_en(rd_en), .d_out(d_out),
        .full(full), .almost_full(almost_full),
        .empty(empty), .almost_empty(almost_empty)
    );

    // Two unrelated clocks: 100 MHz write, ~74.6 MHz read.
    // The offsets are chosen so edges of the two clocks never coincide
    // exactly, which avoids simulation races in the scoreboard.
    always #5 clk_wr = ~clk_wr;
    initial begin
        #1.37;
        forever #6.7 clk_rd = ~clk_rd;
    end

    // ------------------------------------------------------------
    // Scoreboard: total accepted writes / reads (one per clock domain)
    // ------------------------------------------------------------
    integer   wr_total = 0, rd_total = 0;
    reg [7:0] sb [0:255];
    reg [7:0] exp_data = 8'd0;
    integer   errors = 0;
    reg       checking = 1'b0;

    // Write monitor: a write is accepted when wr_en && !full at the clock edge
    always @(posedge clk_wr or negedge rst) begin
        if (!rst) begin
            wr_total = 0;
        end else if (wr_en && !full) begin
            if (wr_total - rd_total >= 16) begin
                errors = errors + 1;
                $display("[%0t] ERROR: OVERFLOW - write accepted while FIFO truly full", $time);
            end
            sb[wr_total % 256] = d_in;
            wr_total = wr_total + 1;
        end
    end

    // Read monitor: a read is accepted when rd_en && !empty at the clock edge
    always @(posedge clk_rd or negedge rst) begin
        if (!rst) begin
            rd_total = 0;
            exp_data = 8'd0;
        end else if (rd_en && !empty) begin
            if (rd_total >= wr_total) begin
                errors = errors + 1;
                $display("[%0t] ERROR: UNDERFLOW - read accepted while FIFO truly empty", $time);
            end else begin
                exp_data = sb[rd_total % 256];
                rd_total = rd_total + 1;
            end
        end
    end

    // ------------------------------------------------------------
    // Continuous checks (flags may lag across domains, but must never
    // be optimistic: never say "not full" / "not empty" when wrong)
    // ------------------------------------------------------------
    integer cw, cr;
    integer seen_full = 0, seen_empty = 0, seen_afull = 0, seen_aempty = 0;

    always @(negedge clk_wr) if (checking) begin
        cw = wr_total - rd_total;
        if (cw == 16 && !full) begin
            errors = errors + 1;
            $display("[%0t] ERROR: full not asserted at count=%0d", $time, cw);
        end
        if (cw >= 14 && !almost_full) begin
            errors = errors + 1;
            $display("[%0t] ERROR: almost_full not asserted at count=%0d", $time, cw);
        end
        if (full)        seen_full  = seen_full  + 1;
        if (almost_full) seen_afull = seen_afull + 1;
    end

    always @(negedge clk_rd) if (checking) begin
        cr = wr_total - rd_total;
        if (d_out !== exp_data) begin
            errors = errors + 1;
            $display("[%0t] ERROR: d_out=%0h expected=%0h", $time, d_out, exp_data);
        end
        if (cr == 0 && !empty) begin
            errors = errors + 1;
            $display("[%0t] ERROR: empty not asserted at count=%0d", $time, cr);
        end
        if (cr <= 2 && !almost_empty) begin
            errors = errors + 1;
            $display("[%0t] ERROR: almost_empty not asserted at count=%0d", $time, cr);
        end
        if (empty)        seen_empty  = seen_empty  + 1;
        if (almost_empty) seen_aempty = seen_aempty + 1;
    end

    // ------------------------------------------------------------
    // Settle check: with no activity, flags must be EXACT
    // ------------------------------------------------------------
    task settle_check(input [8*24:1] tag);
        integer c;
        begin
            wr_en = 0; rd_en = 0;
            repeat (8) @(negedge clk_wr);
            repeat (8) @(negedge clk_rd);
            c = wr_total - rd_total;
            if (full         !== (c == 16)) begin errors = errors + 1; $display("[%0t] ERROR (%0s): full=%b count=%0d",         $time, tag, full, c);         end
            if (empty        !== (c == 0))  begin errors = errors + 1; $display("[%0t] ERROR (%0s): empty=%b count=%0d",        $time, tag, empty, c);        end
            if (almost_full  !== (c >= 14)) begin errors = errors + 1; $display("[%0t] ERROR (%0s): almost_full=%b count=%0d",  $time, tag, almost_full, c);  end
            if (almost_empty !== (c <= 2))  begin errors = errors + 1; $display("[%0t] ERROR (%0s): almost_empty=%b count=%0d", $time, tag, almost_empty, c); end
        end
    endtask

    // ------------------------------------------------------------
    // Write-side tasks (call while aligned to clk_wr negedge)
    // ------------------------------------------------------------
    task w_push;                       // waits while full, then writes one word
        begin
            while (full) @(negedge clk_wr);
            wr_en = 1; d_in = $random;
            @(negedge clk_wr);
            wr_en = 0;
        end
    endtask

    task w_one;      begin @(negedge clk_wr); w_push; end endtask

    task w_one_raw;                    // writes even if full (must be ignored)
        begin
            @(negedge clk_wr);
            wr_en = 1; d_in = $random;
            @(negedge clk_wr);
            wr_en = 0;
        end
    endtask

    task w_stream(input integer n, input integer gap_pct);
        integer k;
        begin
            @(negedge clk_wr);
            k = 0;
            while (k < n) begin
                if (({$random} % 100) < gap_pct) @(negedge clk_wr);
                else begin w_push; k = k + 1; end
            end
        end
    endtask

    task rand_w(input integer n, input integer pct);   // ignores flags
        integer k;
        begin
            @(negedge clk_wr);
            for (k = 0; k < n; k = k + 1) begin
                wr_en   = (({$random} % 100) < pct);
                d_in = $random;
                @(negedge clk_wr);
            end
            wr_en = 0;
        end
    endtask

    // ------------------------------------------------------------
    // Read-side tasks (call while aligned to clk_rd negedge)
    // ------------------------------------------------------------
    task r_pop;                        // waits while empty, then reads one word
        begin
            while (empty) @(negedge clk_rd);
            rd_en = 1;
            @(negedge clk_rd);
            rd_en = 0;
        end
    endtask

    task r_one;      begin @(negedge clk_rd); r_pop; end endtask

    task r_one_raw;                    // reads even if empty (must be ignored)
        begin
            @(negedge clk_rd);
            rd_en = 1;
            @(negedge clk_rd);
            rd_en = 0;
        end
    endtask

    task r_stream(input integer n, input integer gap_pct);
        integer k;
        begin
            @(negedge clk_rd);
            k = 0;
            while (k < n) begin
                if (({$random} % 100) < gap_pct) @(negedge clk_rd);
                else begin r_pop; k = k + 1; end
            end
        end
    endtask

    task rand_r(input integer n, input integer pct);   // ignores flags
        integer k;
        begin
            @(negedge clk_rd);
            for (k = 0; k < n; k = k + 1) begin
                rd_en = (({$random} % 100) < pct);
                @(negedge clk_rd);
            end
            rd_en = 0;
        end
    endtask

    task do_reset;
        begin
            wr_en = 0; rd_en = 0;
            rst = 0;
            repeat (3) @(negedge clk_wr);
            rst = 1;
            repeat (8) @(negedge clk_wr);
        end
    endtask

    // ------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------
    integer i;

    initial begin
        $dumpfile("tb_async_fifo.vcd");
        $dumpvars(0, tb_async_fifo);

        wr_en = 0; rd_en = 0; d_in = 0;
        rst = 0;
        repeat (4) @(negedge clk_wr);
        rst = 1;
        repeat (8) @(negedge clk_wr);   // internal reset synchronizers release
        checking = 1;

        $display("--- Test 1: flags after reset");
        settle_check("after reset");

        $display("--- Test 2: fill one by one, exact flags at every level");
        for (i = 1; i <= 16; i = i + 1) begin
            w_one;
            settle_check("fill walk");
        end

        $display("--- Test 3: write while full (must be ignored)");
        repeat (4) w_one_raw;
        settle_check("overflow attempt");

        $display("--- Test 4: drain one by one, exact flags at every level");
        for (i = 1; i <= 16; i = i + 1) begin
            r_one;
            settle_check("drain walk");
        end

        $display("--- Test 5: read while empty (must be ignored)");
        repeat (4) r_one_raw;
        settle_check("underflow attempt");

        $display("--- Test 6: fast writer, slow reader (FIFO fills up)");
        fork
            w_stream(500, 0);
            r_stream(500, 40);
        join
        settle_check("stream fast wr");

        $display("--- Test 7: slow writer, fast reader (FIFO runs empty)");
        fork
            w_stream(500, 60);
            r_stream(500, 0);
        join
        settle_check("stream fast rd");

        $display("--- Test 8: both sides with random gaps");
        fork
            w_stream(500, 20);
            r_stream(500, 20);
        join
        settle_check("stream balanced");

        $display("--- Test 9: reset in the middle of operation");
        repeat (9) w_one;
        do_reset;
        settle_check("after mid reset");
        w_one; r_one;
        settle_check("post-reset use");

        $display("--- Test 10: random traffic ignoring flags (safety check)");
        fork rand_w(1000, 80); rand_r(1000, 20); join
        fork rand_w(1000, 20); rand_r(1000, 80); join
        fork rand_w(1000, 50); rand_r(1000, 50); join
        while (wr_total > rd_total) r_one;       // drain whatever is left
        settle_check("final drain");

        $display("");
        $display("Coverage (cycles): full=%0d empty=%0d almost_full=%0d almost_empty=%0d",
                 seen_full, seen_empty, seen_afull, seen_aempty);
        $display("Words transferred in last reset window: %0d", rd_total);
        if (errors == 0) $display("*** TEST PASSED ***");
        else             $display("*** TEST FAILED: %0d errors ***", errors);
        $finish;
    end

    // Watchdog
    initial begin
        #20000000;
        $display("*** TIMEOUT ***");
        $finish;
    end

endmodule
