// =============================================================================
// tb_uart_camera_rx.v
// Testbench for uart_camera_rx — Camera AI UART Receiver
//
// How to compile and run (Icarus Verilog):
//   iverilog -o tb_uart.vvp tb_uart_camera_rx.v uart_camera_rx.v
//   vvp tb_uart.vvp
//
// Test Suite
// ----------
//   TC-01  Valid packet  — basic density extraction (0xAA, 0xC3, 0x55)
//   TC-02  Maximum density values (NS=15, EW=15)
//   TC-03  Zero density values    (NS=0,  EW=0)
//   TC-04  camera_valid lifecycle — LOW on reset, HIGH after first packet
//   TC-05  Wrong start byte       — packet ignored, re-sync on next valid
//   TC-06  Wrong end byte         — data NOT committed, previous held
//   TC-07  Framing error          — bad stop bit byte silently discarded
//   TC-08  Glitch rejection       — pulse shorter than 7 baud ticks ignored
//   TC-09  Back-to-back packets   — each overwrites previous correctly
//   TC-10  Timeout watchdog       — camera_valid goes LOW after TIMEOUT_MS
//   TC-11  Recovery after timeout — camera_valid goes HIGH on new packet
// =============================================================================

`timescale 1ns/1ps

module tb_uart_camera_rx;

    // =========================================================================
    // DUT Parameters
    // Using 50 MHz clock and 115200 baud (real hardware values).
    // TIMEOUT_MS is set to 2 (2 ms) so the timeout test completes quickly.
    // =========================================================================
    localparam CLK_FREQ_P   = 50_000_000;
    localparam BAUD_RATE_P  = 115_200;
    localparam TIMEOUT_MS_P = 2;

    // =========================================================================
    // Derived Timing Constants
    // These are computed to match the DUT's internal calculations so the
    // testbench sends bits at the correct rate and uses correct thresholds.
    // =========================================================================
    localparam CLK_PERIOD      = 20;   // ns → 50 MHz simulation clock

    // BIT_CYCLES: how many clock cycles the testbench holds each UART bit.
    // = CLK_FREQ / BAUD_RATE = 50_000_000 / 115_200 = 434 cycles per bit.
    localparam BIT_CYCLES      = CLK_FREQ_P / BAUD_RATE_P;   // 434

    // CLKS_PER_TICK_P: matches the DUT's internal baud_cnt limit.
    // = CLK_FREQ / (BAUD_RATE * 16) = 50_000_000 / 1_843_200 = 27.
    // The DUT uses 16x oversampling, so 1 baud tick = CLKS_PER_TICK_P clocks.
    // Used in TC-08 to send a glitch that is shorter than 7 baud ticks.
    localparam CLKS_PER_TICK_P = CLK_FREQ_P / (BAUD_RATE_P * 16);  // 27

    // TIMEOUT_CYCLES: how many clocks the DUT counts before asserting timeout.
    // = (CLK_FREQ / 1000) * TIMEOUT_MS = 50_000 * 2 = 100_000 cycles = 2 ms.
    localparam TIMEOUT_CYCLES  = (CLK_FREQ_P / 1000) * TIMEOUT_MS_P;  // 100_000

    // =========================================================================
    // DUT Signals
    // =========================================================================
    reg        clk;
    reg        rst_n;
    reg        uart_rx;

    wire [3:0] ns_density;
    wire [3:0] ew_density;
    wire       camera_valid;

    // =========================================================================
    // Scoreboard
    // =========================================================================
    integer pass_count;
    integer fail_count;

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    uart_camera_rx #(
        .CLK_FREQ  (CLK_FREQ_P),
        .BAUD_RATE (BAUD_RATE_P),
        .TIMEOUT_MS(TIMEOUT_MS_P)
    ) dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .uart_rx       (uart_rx),
        .ns_density_o  (ns_density),
        .ew_density_o  (ew_density),
        .camera_valid_o(camera_valid)
    );

    // =========================================================================
    // Internal Probes (reach into DUT for deeper assertions)
    // =========================================================================
    wire [1:0] rx_state  = dut.rx_state;    // UART byte receiver FSM
    wire [1:0] pkt_state = dut.pkt_state;   // Packet assembler FSM
    wire       byte_rdy  = dut.byte_ready;  // Pulses when a full byte arrives

    // =========================================================================
    // Clock Generation
    // =========================================================================
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // Helper Tasks
    // =========================================================================

    // --- assert_check ---
    // Standard pass/fail assertion. Prints state dump on failure.
    task assert_check;
        input condition;
        input [255:0] label;
        begin
            if (condition) begin
                $display("  [PASS] %s", label);
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] %s", label);
                $display("         ns_density=%0d  ew_density=%0d  camera_valid=%b  pkt_state=%0d",
                         ns_density, ew_density, camera_valid, pkt_state);
                fail_count = fail_count + 1;
            end
        end
    endtask

    task print_header;
        input [511:0] title;
        begin
            $display("\n============================================================");
            $display("  %s", title);
            $display("============================================================");
        end
    endtask

    // --- wait_cycles ---
    // Advance simulation by n clock cycles.
    task wait_cycles;
        input integer n;
        begin
            repeat (n) @(posedge clk);
        end
    endtask

    // --- send_byte ---
    // Transmit one UART byte using 8N1 framing:
    //   1 start bit (LOW) + 8 data bits (LSB first) + 1 stop bit (HIGH)
    //
    // Bits are driven at negedge clk so they are stable before the next
    // posedge — matching setup time requirements.
    //
    // The testbench sends at BIT_CYCLES per bit. The DUT's 16x oversampler
    // accepts a ±6/16 bit-period tolerance, so minor cycle-count rounding
    // is fine (this mirrors real hardware where oscillator frequencies differ
    // slightly between the Camera AI board and the FPGA).
    task send_byte;
        input [7:0] data;
        integer i;
        begin
            // Start bit: pull line LOW
            @(negedge clk); uart_rx = 1'b0;
            repeat (BIT_CYCLES) @(posedge clk);

            // 8 data bits, LSB first (UART convention)
            for (i = 0; i < 8; i = i + 1) begin
                @(negedge clk); uart_rx = data[i];
                repeat (BIT_CYCLES) @(posedge clk);
            end

            // Stop bit: release line HIGH (idle)
            @(negedge clk); uart_rx = 1'b1;
            repeat (BIT_CYCLES) @(posedge clk);
        end
    endtask

    // --- send_packet ---
    // Transmit a complete 3-byte packet: 0xAA | {ns[3:0], ew[3:0]} | 0x55
    // Followed by a short idle gap between packets.
    task send_packet;
        input [3:0] ns;
        input [3:0] ew;
        reg   [7:0] data_byte;
        begin
            data_byte = {ns, ew};  // upper nibble = NS, lower nibble = EW
            send_byte(8'hAA);      // start marker
            send_byte(data_byte);  // density payload
            send_byte(8'h55);      // end marker
            wait_cycles(20);       // inter-packet gap (line stays idle HIGH)
        end
    endtask

    // --- send_byte_bad_stop ---
    // Same as send_byte but drives the stop bit LOW instead of HIGH.
    // This creates a framing error. The DUT's RX state machine checks the
    // stop bit and discards the byte silently when it is not HIGH.
    // After the bad stop, the line is forced back to idle.
    task send_byte_bad_stop;
        input [7:0] data;
        integer i;
        begin
            @(negedge clk); uart_rx = 1'b0;          // start bit
            repeat (BIT_CYCLES) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                @(negedge clk); uart_rx = data[i];
                repeat (BIT_CYCLES) @(posedge clk);
            end
            @(negedge clk); uart_rx = 1'b0;          // BAD stop bit (LOW)
            repeat (BIT_CYCLES) @(posedge clk);
            @(negedge clk); uart_rx = 1'b1;          // return line to idle
            wait_cycles(20);
        end
    endtask

    // --- do_reset ---
    // Apply active-low reset and return to a clean known state.
    task do_reset;
        begin
            rst_n   = 1'b0;
            uart_rx = 1'b1;  // UART idle = HIGH
            wait_cycles(5);
            @(negedge clk); rst_n = 1'b1;
            wait_cycles(5);
        end
    endtask

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        pass_count = 0;
        fail_count = 0;
        $dumpfile("tb_uart_camera_rx.vcd");
        $dumpvars(0, tb_uart_camera_rx);

        // ==================================================================
        // TC-01: Valid Packet — Basic Density Extraction
        //
        // Send packet: 0xAA | 0xC3 | 0x55
        //   0xC3 = 8'b1100_0011 → NS nibble = 0xC = 12, EW nibble = 0x3 = 3
        //
        // Verify: ns_density=12, ew_density=3, camera_valid=1
        // ==================================================================
        print_header("TC-01: Valid Packet - Basic Density Extraction");
        do_reset;

        send_packet(4'd12, 4'd3);
        @(posedge clk); #1;

        assert_check(ns_density   === 4'd12, "TC01: ns_density == 12");
        assert_check(ew_density   === 4'd3,  "TC01: ew_density == 3");
        assert_check(camera_valid === 1'b1,  "TC01: camera_valid HIGH after valid packet");

        // ==================================================================
        // TC-02: Maximum Density Values (NS=15, EW=15)
        //
        // 0xFF = 8'b1111_1111 → both nibbles = 15
        // Ensures the 4-bit max value is handled correctly with no overflow.
        // ==================================================================
        print_header("TC-02: Maximum Density Values (NS=15, EW=15)");
        do_reset;

        send_packet(4'd15, 4'd15);
        @(posedge clk); #1;

        assert_check(ns_density === 4'd15, "TC02: ns_density == 15 (max)");
        assert_check(ew_density === 4'd15, "TC02: ew_density == 15 (max)");

        // ==================================================================
        // TC-03: Zero Density Values (NS=0, EW=0)
        //
        // 0x00 → both nibbles = 0. Empty intersection.
        // Ensures zero is not treated as an error or reset condition.
        // ==================================================================
        print_header("TC-03: Zero Density Values (NS=0, EW=0)");
        do_reset;

        send_packet(4'd0, 4'd0);
        @(posedge clk); #1;

        assert_check(ns_density === 4'd0, "TC03: ns_density == 0 (empty road)");
        assert_check(ew_density === 4'd0, "TC03: ew_density == 0 (empty road)");

        // ==================================================================
        // TC-04: camera_valid Lifecycle
        //
        // After reset:        camera_valid = 0  (no data received yet)
        // After first packet: camera_valid = 1  (fresh data available)
        //
        // This is critical — adaptive_timing_logic uses camera_valid=0 to
        // fall back to fixed timing. A false HIGH on reset would mean the
        // system runs on uninitialised density values.
        // ==================================================================
        print_header("TC-04: camera_valid Lifecycle");
        do_reset;
        @(posedge clk); #1;

        assert_check(camera_valid === 1'b0, "TC04: camera_valid LOW immediately after reset");

        send_packet(4'd7, 4'd7);
        @(posedge clk); #1;

        assert_check(camera_valid === 1'b1, "TC04: camera_valid HIGH after first valid packet");

        // ==================================================================
        // TC-05: Wrong Start Byte — Ignored, Packet Assembler Re-Syncs
        //
        // Send: 0xBB (wrong) | 0xFF (would be NS=15,EW=15) | 0x55
        //
        // The packet assembler sits in PKT_WAIT_START and checks every
        // incoming byte for 0xAA. 0xBB fails, so state never advances.
        // The whole malformed packet is silently discarded.
        //
        // Then send a valid packet to prove the assembler cleanly re-syncs
        // and resumes correct operation on the very next 0xAA it sees.
        // ==================================================================
        print_header("TC-05: Wrong Start Byte - Ignored and Re-Sync");
        do_reset;
        send_packet(4'd5, 4'd5);  // establish known baseline
        @(posedge clk); #1;

        send_byte(8'hBB);  // wrong start marker
        send_byte(8'hFF);  // would update to NS=15, EW=15 if accepted
        send_byte(8'h55);
        wait_cycles(20);
        @(posedge clk); #1;

        assert_check(ns_density === 4'd5, "TC05: ns_density unchanged after wrong start byte");
        assert_check(ew_density === 4'd5, "TC05: ew_density unchanged after wrong start byte");

        send_packet(4'd9, 4'd2);  // valid packet — must be accepted correctly
        @(posedge clk); #1;

        assert_check(ns_density === 4'd9, "TC05: re-sync OK, ns_density updated to 9");
        assert_check(ew_density === 4'd2, "TC05: re-sync OK, ew_density updated to 2");

        // ==================================================================
        // TC-06: Wrong End Byte — Data NOT Committed
        //
        // Send: 0xAA | 0xF0 (NS=15, EW=0) | 0x44 (wrong end)
        //
        // The packet assembler advances through PKT_WAIT_START →
        // PKT_WAIT_DATA correctly, latches the data byte, then in
        // PKT_WAIT_END checks the end marker. 0x44 ≠ 0x55 so it
        // discards the latched data and resets to PKT_WAIT_START.
        // ns/ew_density must NOT change.
        // ==================================================================
        print_header("TC-06: Wrong End Byte - Data Discarded");
        do_reset;
        send_packet(4'd6, 4'd6);  // baseline
        @(posedge clk); #1;

        send_byte(8'hAA);  // correct start
        send_byte(8'hF0);  // NS=15, EW=0 — would be catastrophic if accepted
        send_byte(8'h44);  // wrong end marker
        wait_cycles(20);
        @(posedge clk); #1;

        assert_check(ns_density === 4'd6, "TC06: ns_density unchanged after bad end byte");
        assert_check(ew_density === 4'd6, "TC06: ew_density unchanged after bad end byte");

        // ==================================================================
        // TC-07: Framing Error — Bad Stop Bit Discarded
        //
        // The UART RX state machine checks that the stop bit is HIGH.
        // A LOW stop bit = framing error → byte is NOT published to the
        // packet assembler (byte_ready stays LOW).
        //
        // We corrupt the start marker (0xAA) with a bad stop bit.
        // Without the start marker, the packet assembler never leaves
        // PKT_WAIT_START, so the following data and end bytes are also
        // ignored.
        // ==================================================================
        print_header("TC-07: Framing Error - Bad Stop Bit Discarded");
        do_reset;
        send_packet(4'd4, 4'd4);  // baseline
        @(posedge clk); #1;

        send_byte_bad_stop(8'hAA);  // 0xAA with bad stop → discarded
        send_byte(8'hFF);           // lost context: assembler never saw 0xAA
        send_byte(8'h55);
        wait_cycles(20);
        @(posedge clk); #1;

        assert_check(ns_density === 4'd4, "TC07: ns_density unchanged after framing error");
        assert_check(ew_density === 4'd4, "TC07: ew_density unchanged after framing error");

        // Also verify normal packets still work after framing error
        send_packet(4'd10, 4'd5);
        @(posedge clk); #1;
        assert_check(ns_density === 4'd10, "TC07: normal packet works after framing error");
        assert_check(ew_density === 4'd5,  "TC07: ew_density correct after framing error");

        // ==================================================================
        // TC-08: Glitch Rejection on Start Bit
        //
        // The RX_START state waits for baud_tick #7 (the centre of the
        // start bit) before confirming it is a real start. A pulse that
        // returns to HIGH before tick 7 is treated as a line glitch.
        //
        // Valid start   = hold LOW for >= 7 * CLKS_PER_TICK cycles
        // Glitch        = hold LOW for  < 7 * CLKS_PER_TICK cycles
        //
        // We hold LOW for only 4 * CLKS_PER_TICK (= 4 * 27 = 108 cycles),
        // well below the 7 * 27 = 189 cycle threshold.
        //
        // No byte_ready pulse should fire, so densities stay unchanged.
        // ==================================================================
        print_header("TC-08: Glitch Rejection on Start Bit");
        do_reset;
        send_packet(4'd3, 4'd3);  // baseline
        @(posedge clk); #1;

        // Inject glitch: LOW pulse of 4 baud ticks (< 7 required)
        @(negedge clk); uart_rx = 1'b0;
        repeat (4 * CLKS_PER_TICK_P) @(posedge clk);
        @(negedge clk); uart_rx = 1'b1;              // release before tick 7
        wait_cycles(BIT_CYCLES * 5);                 // confirm no byte received
        @(posedge clk); #1;

        assert_check(ns_density === 4'd3, "TC08: ns_density unchanged after glitch");
        assert_check(ew_density === 4'd3, "TC08: ew_density unchanged after glitch");
        assert_check(byte_rdy   === 1'b0, "TC08: byte_ready did not fire on glitch");

        // Normal operation must be unaffected after glitch
        send_packet(4'd11, 4'd7);
        @(posedge clk); #1;

        assert_check(ns_density === 4'd11, "TC08: normal packet accepted after glitch rejection");
        assert_check(ew_density === 4'd7,  "TC08: ew_density correct after glitch rejection");

        // ==================================================================
        // TC-09: Back-to-Back Packets
        //
        // Send 5 packets with distinct density values in rapid succession.
        // Each packet must overwrite the previous correctly.
        // This validates that the packet assembler properly resets to
        // PKT_WAIT_START after each complete packet, ready for the next one.
        // ==================================================================
        print_header("TC-09: Back-to-Back Packets (5 consecutive)");
        do_reset;

        begin : multi_packet_test
            reg all_ok;
            all_ok = 1;

            // Packet 1
            send_packet(4'd1, 4'd14);
            @(posedge clk); #1;
            if (ns_density !== 4'd1 || ew_density !== 4'd14) begin
                all_ok = 0;
                $display("    MISMATCH pkt1: expected NS=1 EW=14, got NS=%0d EW=%0d",
                         ns_density, ew_density);
            end

            // Packet 2
            send_packet(4'd5, 4'd10);
            @(posedge clk); #1;
            if (ns_density !== 4'd5 || ew_density !== 4'd10) begin
                all_ok = 0;
                $display("    MISMATCH pkt2: expected NS=5 EW=10, got NS=%0d EW=%0d",
                         ns_density, ew_density);
            end

            // Packet 3 — symmetric
            send_packet(4'd8, 4'd8);
            @(posedge clk); #1;
            if (ns_density !== 4'd8 || ew_density !== 4'd8) begin
                all_ok = 0;
                $display("    MISMATCH pkt3: expected NS=8 EW=8, got NS=%0d EW=%0d",
                         ns_density, ew_density);
            end

            // Packet 4
            send_packet(4'd13, 4'd2);
            @(posedge clk); #1;
            if (ns_density !== 4'd13 || ew_density !== 4'd2) begin
                all_ok = 0;
                $display("    MISMATCH pkt4: expected NS=13 EW=2, got NS=%0d EW=%0d",
                         ns_density, ew_density);
            end

            // Packet 5 — extreme
            send_packet(4'd15, 4'd0);
            @(posedge clk); #1;
            if (ns_density !== 4'd15 || ew_density !== 4'd0) begin
                all_ok = 0;
                $display("    MISMATCH pkt5: expected NS=15 EW=0, got NS=%0d EW=%0d",
                         ns_density, ew_density);
            end

            assert_check(all_ok, "TC09: all 5 back-to-back packets decoded correctly");
        end

        // ==================================================================
        // TC-10: Timeout Watchdog — camera_valid Goes LOW
        //
        // After TIMEOUT_MS (2 ms = 100_000 clock cycles) with no packet,
        // the timeout counter saturates and pulls camera_valid LOW.
        //
        // This signals adaptive_timing_logic to fall back to fixed timing.
        //
        // Importantly, the density values themselves are HELD — the module
        // does not zero them out. The fallback is communicated only through
        // camera_valid.
        // ==================================================================
        print_header("TC-10: Timeout Watchdog - camera_valid Goes LOW");
        do_reset;

        send_packet(4'd8, 4'd8);
        @(posedge clk); #1;
        assert_check(camera_valid === 1'b1, "TC10: camera_valid HIGH before timeout");

        // Let the timeout counter run out (add small margin)
        wait_cycles(TIMEOUT_CYCLES + 200);
        @(posedge clk); #1;

        assert_check(camera_valid === 1'b0, "TC10: camera_valid LOW after timeout");
        assert_check(ns_density   === 4'd8, "TC10: ns_density held (not zeroed) during timeout");
        assert_check(ew_density   === 4'd8, "TC10: ew_density held (not zeroed) during timeout");

        // ==================================================================
        // TC-11: Recovery After Timeout
        //
        // After camera_valid has gone LOW, the arrival of a new valid
        // packet must:
        //   1. Reset the timeout counter to 0
        //   2. Raise camera_valid back to HIGH
        //   3. Update ns_density and ew_density to the new values
        //
        // This is the "camera reconnected" or "AI process restarted" case.
        // ==================================================================
        print_header("TC-11: Recovery After Timeout");
        // Continue from TC-10 (camera_valid is currently LOW)

        send_packet(4'd14, 4'd1);
        @(posedge clk); #1;

        assert_check(camera_valid === 1'b1, "TC11: camera_valid HIGH after recovery packet");
        assert_check(ns_density   === 4'd14, "TC11: ns_density updated after recovery");
        assert_check(ew_density   === 4'd1,  "TC11: ew_density updated after recovery");

        // Verify continued stable operation
        send_packet(4'd7, 4'd9);
        @(posedge clk); #1;
        assert_check(ns_density === 4'd7, "TC11: continued operation after recovery works");
        assert_check(ew_density === 4'd9, "TC11: ew_density correct in continued operation");

        // ======================================================================
        // Final Summary
        // ======================================================================
        $display("\n============================================================");
        $display("  TEST RESULTS");
        $display("============================================================");
        $display("  PASS : %0d", pass_count);
        $display("  FAIL : %0d", fail_count);
        $display("  TOTAL: %0d", pass_count + fail_count);
        if (fail_count == 0)
            $display("  *** ALL TESTS PASSED ***");
        else
            $display("  *** %0d TEST(S) FAILED — check [FAIL] lines above ***", fail_count);
        $display("============================================================\n");

        $finish;
    end

    // =========================================================================
    // Watchdog — kill simulation if it exceeds expected runtime
    // Expected runtime estimate:
    //   ~15 send_packet calls × (3 bytes × 434 + 20) cycles = ~20,000 cycles
    //   TC-10 timeout wait = 100,200 cycles
    //   Total ≈ 150,000 cycles × 20ns = ~3ms
    // 500ms gives 150× headroom for any unexpected hang.
    // =========================================================================
    initial begin
        #500_000_000;
        $display("[WATCHDOG] Simulation exceeded 500ms. Forcing stop.");
        $finish;
    end

endmodule