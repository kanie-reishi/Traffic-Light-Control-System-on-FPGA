`timescale 1ns / 1ps

// ============================================================================
// tb_seg7_display.v
// Testbench for the 7-segment multiplexed display subsystem
//
// Tests:
//   1. seg7_hex_decoder — all digits 0-9 produce correct cathode patterns
//   2. seg7_mux_driver  — verifies BCD conversion, digit scanning, anode cycling
//   3. Integration      — drives countdown values and checks display output
// ============================================================================
module tb_seg7_display;

    // ========================================================================
    // Clock and Reset
    // ========================================================================
    reg clk;
    reg rst_n;

    initial clk = 0;
    always #5 clk = ~clk; // 100 MHz

    // ========================================================================
    // Part 1: Standalone decoder verification
    // ========================================================================
    reg  [3:0] dec_bcd;
    wire [6:0] dec_seg;

    seg7_hex_decoder u_dec_test (
        .bcd_i(dec_bcd),
        .seg_o(dec_seg)
    );

    // Expected cathode patterns {A,B,C,D,E,F,G} active-low for 0–9
    reg [6:0] expected_seg [0:9];
    initial begin
        expected_seg[0] = 7'b100_0000;
        expected_seg[1] = 7'b111_1001;
        expected_seg[2] = 7'b010_0100;
        expected_seg[3] = 7'b011_0000;
        expected_seg[4] = 7'b001_1001;
        expected_seg[5] = 7'b001_0010;
        expected_seg[6] = 7'b000_0010;
        expected_seg[7] = 7'b111_1000;
        expected_seg[8] = 7'b000_0000;
        expected_seg[9] = 7'b001_0000;
    end

    // ========================================================================
    // Part 2: MUX driver instantiation (use small refresh counter for sim)
    // ========================================================================
    reg  [5:0] ns_countdown;
    reg  [5:0] ew_countdown;
    wire [6:0] mux_seg;
    wire [3:0] mux_an;

    seg7_mux_driver #(
        .REFR_BITS(4)  // 2^4 = 16 clocks per refresh cycle (fast for sim)
    ) u_mux_test (
        .clk           (clk),
        .rst_n         (rst_n),
        .ns_countdown_i(ns_countdown),
        .ew_countdown_i(ew_countdown),
        .seg_o         (mux_seg),
        .an_o          (mux_an)
    );

    // ========================================================================
    // Test Sequence
    // ========================================================================
    integer i;
    integer pass_count;
    integer fail_count;

    initial begin
        $display("==================================================");
        $display("  7-SEGMENT DISPLAY TESTBENCH");
        $display("==================================================");

        pass_count = 0;
        fail_count = 0;

        // -----------------------------------------------------------------
        // Test 1: Decoder — sweep all BCD digits
        // -----------------------------------------------------------------
        $display("\n--- Test 1: seg7_hex_decoder sweep ---");
        for (i = 0; i < 10; i = i + 1) begin
            dec_bcd = i[3:0];
            #10; // allow combinational settle
            if (dec_seg === expected_seg[i]) begin
                $display("  [PASS] digit %0d: seg = %b", i, dec_seg);
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] digit %0d: seg = %b, expected = %b",
                         i, dec_seg, expected_seg[i]);
                fail_count = fail_count + 1;
            end
        end

        // Test invalid BCD (should show dash)
        dec_bcd = 4'd15;
        #10;
        if (dec_seg === 7'b011_1111) begin
            $display("  [PASS] invalid BCD (15): seg = %b (dash)", dec_seg);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] invalid BCD (15): seg = %b, expected = 0111111", dec_seg);
            fail_count = fail_count + 1;
        end

        // -----------------------------------------------------------------
        // Test 2: MUX driver — verify digit scanning and BCD conversion
        // -----------------------------------------------------------------
        $display("\n--- Test 2: seg7_mux_driver scan ---");

        rst_n = 0;
        ns_countdown = 6'd25;  // N-S shows "25"
        ew_countdown = 6'd10;  // E-W shows "10"
        #50;
        rst_n = 1;

        // Wait for several full refresh cycles (4 digits × 4 clocks each = 16 clocks)
        // Run multiple cycles to observe scanning
        $display("  Driving ns_countdown = 25, ew_countdown = 10");
        $display("  Expected: AN3=2, AN2=5, AN1=1, AN0=0");
        $display("  Monitoring anode/cathode outputs for 10 refresh cycles...\n");

        repeat (10) begin   // 10 full scan cycles
            repeat (16) begin
                @(posedge clk);
                #1; // small delay for output settle after registered cathode
            end
        end

        // Sample each digit position by waiting for specific anode
        // Force a known starting point
        $display("  Sampling individual digit positions:");

        // Wait for AN0 active (ew_ones = 0)
        wait(mux_an == 4'b1110);
        @(posedge clk); #1;
        $display("  AN0 (EW ones): an=%b seg=%b (expect digit 0)", mux_an, mux_seg);

        // Wait for AN1 active (ew_tens = 1)
        wait(mux_an == 4'b1101);
        @(posedge clk); #1;
        $display("  AN1 (EW tens): an=%b seg=%b (expect digit 1)", mux_an, mux_seg);

        // Wait for AN2 active (ns_ones = 5)
        wait(mux_an == 4'b1011);
        @(posedge clk); #1;
        $display("  AN2 (NS ones): an=%b seg=%b (expect digit 5)", mux_an, mux_seg);

        // Wait for AN3 active (ns_tens = 2)
        wait(mux_an == 4'b0111);
        @(posedge clk); #1;
        $display("  AN3 (NS tens): an=%b seg=%b (expect digit 2)", mux_an, mux_seg);

        // -----------------------------------------------------------------
        // Test 3: Change countdown values mid-run
        // -----------------------------------------------------------------
        $display("\n--- Test 3: Dynamic countdown change ---");

        ns_countdown = 6'd45;
        ew_countdown = 6'd3;
        $display("  Changed to ns=45, ew=03");

        // Let a few cycles pass
        repeat (5) begin
            repeat (16) @(posedge clk);
        end

        // Sample AN3 (ns_tens = 4)
        wait(mux_an == 4'b0111);
        @(posedge clk); #1;
        $display("  AN3 (NS tens): an=%b seg=%b (expect digit 4)", mux_an, mux_seg);

        // Sample AN0 (ew_ones = 3)
        wait(mux_an == 4'b1110);
        @(posedge clk); #1;
        $display("  AN0 (EW ones): an=%b seg=%b (expect digit 3)", mux_an, mux_seg);

        // -----------------------------------------------------------------
        // Test 4: Zero countdown (both directions idle)
        // -----------------------------------------------------------------
        $display("\n--- Test 4: All zeros ---");

        ns_countdown = 6'd0;
        ew_countdown = 6'd0;
        repeat (5) begin
            repeat (16) @(posedge clk);
        end

        wait(mux_an == 4'b0111);
        @(posedge clk); #1;
        $display("  AN3 (NS tens): seg=%b (expect digit 0 = 1000000)", mux_seg);

        wait(mux_an == 4'b1110);
        @(posedge clk); #1;
        $display("  AN0 (EW ones): seg=%b (expect digit 0 = 1000000)", mux_seg);

        // -----------------------------------------------------------------
        // Test 5: Maximum value (63)
        // -----------------------------------------------------------------
        $display("\n--- Test 5: Maximum value (63) ---");

        ns_countdown = 6'd63;
        ew_countdown = 6'd63;
        repeat (5) begin
            repeat (16) @(posedge clk);
        end

        // 63 → tens=6, ones=3
        wait(mux_an == 4'b0111);
        @(posedge clk); #1;
        $display("  AN3 (NS tens): seg=%b (expect digit 6 = 0000010)", mux_seg);

        wait(mux_an == 4'b1011);
        @(posedge clk); #1;
        $display("  AN2 (NS ones): seg=%b (expect digit 3 = 0110000)", mux_seg);

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        $display("\n==================================================");
        $display("  RESULTS: %0d passed, %0d failed (decoder sweep)", pass_count, fail_count);
        $display("  MUX driver tests: visual inspection above");
        $display("==================================================");

        if (fail_count == 0)
            $display("  ALL DECODER TESTS PASSED");
        else
            $display("  SOME TESTS FAILED — review output above");

        $finish;
    end

    // ========================================================================
    // VCD dump for waveform viewing
    // ========================================================================
    initial begin
        $dumpfile("tb_seg7_display.vcd");
        $dumpvars(0, tb_seg7_display);
    end

endmodule
