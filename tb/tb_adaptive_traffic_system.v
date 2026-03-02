// =============================================================================
// tb_traffic_system_top.v  — FIXED VERSION
// Testbench for traffic_system_top (Adaptive Traffic Light Controller)
//
// FIXES vs previous version
// --------------------------
//   FIX-1  Removed dependency on clock_divider stub.
//          The old stub conflicted with the real clock_divider.v when both
//          were compiled together. If the real 50 MHz divider ran, tick_1hz
//          fired once every 50,000,000 cycles — the entire testbench finished
//          in ~600,000 cycles, so ZERO ticks fired and the timer never
//          decremented. This is why ALL state-transition tests timed out.
//          Solution: generate tick_1hz directly in the testbench and
//          force it onto dut.tick_1hz. The DUT's internal clock_divider
//          is completely bypassed.
//
//   FIX-2  Flash detection now samples on @(negedge clk).
//          tick_1hz is a registered signal — it changes via NBA at posedge.
//          Sampling ns_leds at posedge clk in the same delta-cycle reads
//          the OLD value (0) before the NBA settles, so the test always
//          saw LIGHT_OFF even when the light should be YELLOW.
//          Sampling at negedge means tick_1hz is stable for half a clock
//          period before we read it.
//
//   FIX-3  Timeouts scaled to actual timer durations.
//          T_GREEN_MAX = 45 ticks needs at least 50-tick margin.
//          wait_for_state() now receives cycle counts, not tick counts,
//          to be independent of TICK_CYCLES.
//
// How to compile and run (Icarus Verilog)
// ----------------------------------------
//   iverilog -o tb_traffic.vvp \
//       tb_traffic_system_top.v \
//       adaptive_timing_logic.v \
//       traffic_controller_core.v
//   vvp tb_traffic.vvp
//
//   *** DO NOT add clock_divider.v to this command line. ***
//   The testbench drives tick_1hz directly — the clock_divider module
//   is never called. The stub at the top of this file only satisfies
//   the linker when traffic_system_top instantiates it.
//
// ModelSim / Questasim
// ---------------------
//   vlog tb_traffic_system_top.v adaptive_timing_logic.v traffic_controller_core.v
//   vsim -c work.tb_traffic_system_top -do "run -all; quit"
// =============================================================================

`timescale 1ns/1ps

// =============================================================================
// clock_divider STUB (linker-only — not used at runtime)
// =============================================================================
// This stub exists only so that traffic_system_top compiles without needing
// a separate clock_divider.v file. At runtime, `force dut.tick_1hz = tb_tick`
// in do_reset() overrides whatever this module produces, so its exact
// behaviour has no effect on test results.
// If you see a "module redefinition" warning, add +define+NO_CD_STUB to your
// compile flags and guard this block with `ifndef NO_CD_STUB.
// =============================================================================
module clock_divider #(parameter FREQ = 50_000_000) (
    input  wire clk,
    input  wire rst_n,
    output reg  tick_out
);
    localparam DIV = 10;
    integer cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin cnt <= 0; tick_out <= 0; end
        else begin
            tick_out <= 0;
            if (cnt == DIV - 1) begin cnt <= 0; tick_out <= 1; end
            else cnt <= cnt + 1;
        end
    end
endmodule

// =============================================================================
// Testbench
// =============================================================================
module tb_traffic_system_top;

    // =========================================================================
    // DUT ports
    // =========================================================================
    reg        clk;
    reg        rst_n;
    reg        emergency_sw;
    reg  [3:0] ns_density;
    reg  [3:0] ew_density;
    reg        camera_valid;

    wire [2:0] ns_leds;
    wire [2:0] ew_leds;
    wire       ped_ns_led;
    wire       ped_ew_led;

    // =========================================================================
    // Encodings (mirror DUT)
    // =========================================================================
    `define LIGHT_RED    3'b100
    `define LIGHT_YELLOW 3'b010
    `define LIGHT_GREEN  3'b001
    `define LIGHT_OFF    3'b000

    `define S_NS_GREEN   5'b00001
    `define S_NS_YELLOW  5'b00010
    `define S_EW_GREEN   5'b00100
    `define S_EW_YELLOW  5'b01000
    `define S_ERROR      5'b10000

    localparam T_GREEN_MIN    = 6'd8;
    localparam T_GREEN_BASE   = 6'd15;
    localparam T_GREEN_MAX    = 6'd45;
    localparam T_YELLOW_FIXED = 6'd5;

    // =========================================================================
    // Simulation speed
    // =========================================================================
    localparam CLK_PERIOD  = 10;    // ns per clock (100 MHz sim clock)
    localparam TICK_CYCLES = 20;    // clock cycles per simulated second
    //
    // Why TICK_CYCLES = 20 and not 10?
    // tick_1hz is driven from tb_tick (see below). tb_tick goes HIGH for
    // one full clock cycle, driven at negedge. The core samples it at
    // posedge. With TICK_CYCLES = 20, the HIGH phase lands squarely between
    // two posedges, giving 10 ns of stable setup time before the sample edge.
    // TICK_CYCLES = 10 would make the pulse barely one cycle wide and
    // re-introduce timing risk.

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    traffic_system_top dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .emergency_sw  (emergency_sw),
        .ns_density_i  (ns_density),
        .ew_density_i  (ew_density),
        .camera_valid_i(camera_valid),
        .ns_leds       (ns_leds),
        .ew_leds       (ew_leds),
        .ped_ns_led    (ped_ns_led),
        .ped_ew_led    (ped_ew_led)
    );

    // =========================================================================
    // FIX-1: Testbench-controlled tick generator
    // =========================================================================
    // tb_tick fires for one full clock period every TICK_CYCLES clocks.
    // It is driven at negedge so it is fully settled before the next posedge
    // where the core's timer samples it.
    //
    // do_reset() calls:
    //   force dut.tick_1hz = tb_tick;
    // which overrides whatever the internal clock_divider produces.
    // =========================================================================
    reg tb_tick;

    initial tb_tick = 0;

    always begin
        // Hold low for (TICK_CYCLES - 1) full cycles
        repeat (TICK_CYCLES - 1) @(posedge clk);
        // Drive HIGH at negedge: stable well before the next posedge
        @(negedge clk); tb_tick = 1;
        // Hold HIGH for one full clock period
        @(negedge clk); tb_tick = 0;
    end

    // =========================================================================
    // Internal probes
    // =========================================================================
    wire [4:0] current_state = dut.core_inst.current_state;
    wire [4:0] next_state    = dut.core_inst.next_state;
    wire [5:0] timer_val     = dut.core_inst.timer;
    wire [5:0] duration_out  = dut.timer_duration;

    // =========================================================================
    // Scoreboard
    // =========================================================================
    integer pass_count;
    integer fail_count;

    // =========================================================================
    // Clock
    // =========================================================================
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // Helper tasks
    // =========================================================================

    // Wait for n full tick periods (each = TICK_CYCLES clock cycles)
    task wait_ticks;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1)
                repeat (TICK_CYCLES) @(posedge clk);
        end
    endtask

    // Poll current_state every clock cycle until it matches target or times out.
    // timeout_cycles is in raw clock cycles (not ticks) for fine-grained control.
    // Adds #1 after each posedge to let NBAs settle before reading.
    task wait_for_state;
        input [4:0] target;
        input integer timeout_cycles;
        integer elapsed;
        begin
            elapsed = 0;
            while (current_state !== target && elapsed < timeout_cycles) begin
                @(posedge clk); #1;
                elapsed = elapsed + 1;
            end
            if (current_state !== target)
                $display("  [TIMEOUT] %0d cycles elapsed, state never reached %05b (stuck at %05b, timer=%0d)",
                         elapsed, target, current_state, timer_val);
        end
    endtask

    // Print pass/fail with context dump on failure
    task assert_check;
        input condition;
        input [255:0] label;
        begin
            if (condition) begin
                $display("  [PASS] %s", label);
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] %s", label);
                $display("         state=%05b  ns=%03b  ew=%03b  duration=%0d  timer=%0d",
                         current_state, ns_leds, ew_leds, duration_out, timer_val);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // Reset DUT and install tick bypass
    task do_reset;
        begin
            rst_n        = 0;
            emergency_sw = 0;
            ns_density   = 4'd7;
            ew_density   = 4'd7;
            camera_valid = 1;
            repeat (4) @(posedge clk);
            @(negedge clk); rst_n = 1;   // deassert at negedge for clean setup

            // FIX-1: bypass clock_divider — drive tick_1hz directly
            force dut.tick_1hz = tb_tick;

            wait_ticks(3);  // let FSM initialize (first_cycle clears, timer loads)
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

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin
        pass_count = 0;
        fail_count = 0;
        $dumpfile("tb_traffic.vcd");
        $dumpvars(0, tb_traffic_system_top);

        // ------------------------------------------------------------------
        // TC-01  Reset and Default Startup
        // ------------------------------------------------------------------
        print_header("TC-01: Reset and Default Startup");
        do_reset;
        @(posedge clk); #1;

        assert_check(current_state === `S_NS_GREEN,  "TC01: state == S_NS_GREEN after reset");
        assert_check(ns_leds === `LIGHT_GREEN,        "TC01: NS light is GREEN");
        assert_check(ew_leds === `LIGHT_RED,          "TC01: EW light is RED");
        assert_check(ped_ns_led === 1'b1,             "TC01: NS pedestrian walk ON");
        assert_check(ped_ew_led === 1'b0,             "TC01: EW pedestrian walk OFF");

        // ------------------------------------------------------------------
        // TC-02  Balanced traffic — full 4-state cycle
        // Equal density: no advantage → NS gets T_GREEN_MIN.
        // Verify full cycle: NS_GREEN → NS_YELLOW → EW_GREEN → EW_YELLOW → NS_GREEN
        // ------------------------------------------------------------------
        print_header("TC-02: Balanced Traffic - Full Cycle");
        do_reset;
        ns_density = 4'd8; ew_density = 4'd8; camera_valid = 1;
        @(posedge clk); #1;

        assert_check(current_state === `S_NS_GREEN,    "TC02: starts in S_NS_GREEN");
        assert_check(duration_out  === T_GREEN_MIN,    "TC02: NS green == T_GREEN_MIN (no advantage)");

        // T_GREEN_MIN=8 ticks → 8*TICK_CYCLES cycles, plus generous margin
        wait_for_state(`S_NS_YELLOW, (T_GREEN_MIN + 5) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(current_state === `S_NS_YELLOW,   "TC02: NS_GREEN → NS_YELLOW");
        assert_check(ns_leds === `LIGHT_YELLOW,        "TC02: NS light YELLOW in S_NS_YELLOW");

        wait_for_state(`S_EW_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(current_state === `S_EW_GREEN,    "TC02: NS_YELLOW → EW_GREEN");
        assert_check(ew_leds === `LIGHT_GREEN,         "TC02: EW light GREEN in S_EW_GREEN");

        wait_for_state(`S_EW_YELLOW, (T_GREEN_MIN + 5) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(current_state === `S_EW_YELLOW,   "TC02: EW_GREEN → EW_YELLOW");

        wait_for_state(`S_NS_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(current_state === `S_NS_GREEN,    "TC02: full cycle complete, back to S_NS_GREEN");

        // ------------------------------------------------------------------
        // TC-03  NS heavy (NS=15, EW=0)
        // NS green must be clamped to T_GREEN_MAX.
        // Yellow phase must remain T_YELLOW_FIXED (non-negotiable).
        // ------------------------------------------------------------------
        print_header("TC-03: NS Heavy Traffic (NS=15, EW=0)");
        do_reset;
        ns_density = 4'd15; ew_density = 4'd0; camera_valid = 1;
        @(posedge clk); #1;

        assert_check(current_state === `S_NS_GREEN,    "TC03: starts in S_NS_GREEN");
        assert_check(duration_out  === T_GREEN_MAX,    "TC03: NS green clamped to T_GREEN_MAX");

        wait_for_state(`S_NS_YELLOW, (T_GREEN_MAX + 10) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(current_state === `S_NS_YELLOW,   "TC03: transitions to S_NS_YELLOW");
        assert_check(duration_out  === T_YELLOW_FIXED, "TC03: yellow is FIXED T_YELLOW_FIXED (not adaptive)");

        // ------------------------------------------------------------------
        // TC-04  EW heavy (NS=0, EW=15)
        // NS has no advantage → T_GREEN_MIN.  EW should get T_GREEN_MAX.
        // ------------------------------------------------------------------
        print_header("TC-04: EW Heavy Traffic (NS=0, EW=15)");
        do_reset;
        ns_density = 4'd0; ew_density = 4'd15; camera_valid = 1;
        @(posedge clk); #1;

        assert_check(duration_out === T_GREEN_MIN,     "TC04: NS green == T_GREEN_MIN (no NS advantage)");

        wait_for_state(`S_EW_GREEN, (T_GREEN_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(current_state === `S_EW_GREEN,    "TC04: transitions to S_EW_GREEN");
        assert_check(duration_out  === T_GREEN_MAX,    "TC04: EW green clamped to T_GREEN_MAX");

        // ------------------------------------------------------------------
        // TC-05  Camera invalid → fallback to T_GREEN_BASE
        // With density 15/0 (max advantage), valid camera gives T_GREEN_MAX.
        // Invalid camera must give T_GREEN_BASE regardless.
        // ------------------------------------------------------------------
        print_header("TC-05: Camera Invalid - Fallback Timing");
        do_reset;
        ns_density = 4'd15; ew_density = 4'd0; camera_valid = 0;
        @(posedge clk); #1;

        assert_check(duration_out  === T_GREEN_BASE,   "TC05: NS green falls back to T_GREEN_BASE");
        assert_check(current_state === `S_NS_GREEN,    "TC05: system still operates normally");

        wait_for_state(`S_EW_GREEN, (T_GREEN_BASE + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(duration_out === T_GREEN_BASE,    "TC05: EW green also uses T_GREEN_BASE fallback");

        // ------------------------------------------------------------------
        // TC-06  Emergency mode → S_ERROR + flashing yellow
        //
        // FIX-2: Flash detection samples at @(negedge clk) to avoid the
        // delta-cycle race. tick_1hz changes at posedge via NBA; sampling
        // at negedge means the signal has been stable for half a cycle.
        // ------------------------------------------------------------------
        print_header("TC-06: Emergency Mode - S_ERROR and Flashing Yellow");
        do_reset;
        ns_density = 4'd5; ew_density = 4'd5; camera_valid = 1;
        wait_ticks(3);
        emergency_sw = 1;
        wait_ticks(2);
        @(posedge clk); #1;

        assert_check(current_state === `S_ERROR,       "TC06: FSM enters S_ERROR");
        assert_check(ped_ns_led   === 1'b0,            "TC06: NS pedestrian walk OFF");
        assert_check(ped_ew_led   === 1'b0,            "TC06: EW pedestrian walk OFF");

        begin : flash_check
            reg ns_saw_yellow, ns_saw_off, ew_saw_yellow, ew_saw_off;
            integer f;
            ns_saw_yellow = 0; ns_saw_off = 0;
            ew_saw_yellow = 0; ew_saw_off = 0;
            // Sample over 5 tick periods at negedge to catch both YELLOW and OFF
            for (f = 0; f < TICK_CYCLES * 5; f = f + 1) begin
                @(negedge clk); // FIX-2: stable sampling point
                if (ns_leds === `LIGHT_YELLOW) ns_saw_yellow = 1;
                if (ns_leds === `LIGHT_OFF)    ns_saw_off    = 1;
                if (ew_leds === `LIGHT_YELLOW) ew_saw_yellow = 1;
                if (ew_leds === `LIGHT_OFF)    ew_saw_off    = 1;
            end
            assert_check(ns_saw_yellow && ns_saw_off,
                         "TC06: NS light flashes YELLOW/OFF (saw both phases)");
            assert_check(ew_saw_yellow && ew_saw_off,
                         "TC06: EW light flashes YELLOW/OFF (saw both phases)");
        end

        // ------------------------------------------------------------------
        // TC-07  Emergency release → clean recovery
        // S_ERROR is sticky — needs reset. After reset, FSM must return to
        // S_NS_GREEN with correct outputs and no residual error state.
        // ------------------------------------------------------------------
        print_header("TC-07: Emergency Release and Recovery");
        emergency_sw = 0;
        rst_n = 0;
        repeat (4) @(posedge clk);
        @(negedge clk); rst_n = 1;
        force dut.tick_1hz = tb_tick; // re-apply after reset
        wait_ticks(3);
        @(posedge clk); #1;

        assert_check(current_state === `S_NS_GREEN,    "TC07: returns to S_NS_GREEN after reset");
        assert_check(ns_leds       === `LIGHT_GREEN,   "TC07: NS light is GREEN");
        assert_check(ew_leds       === `LIGHT_RED,     "TC07: EW light is RED");
        assert_check(ped_ns_led    === 1'b1,           "TC07: NS pedestrian walk ON");
        assert_check(ped_ew_led    === 1'b0,           "TC07: EW pedestrian walk OFF");

        // ------------------------------------------------------------------
        // TC-08  MAX green clamp — 4 full cycles
        // Monitor every S_NS_GREEN and S_EW_GREEN entry; fail if any
        // duration exceeds T_GREEN_MAX.
        // ------------------------------------------------------------------
        print_header("TC-08: MAX Green Clamp Verification (4 cycles)");
        do_reset;
        ns_density = 4'd15; ew_density = 4'd0; camera_valid = 1;
        begin : max_clamp
            integer cyc; reg ok; ok = 1;
            for (cyc = 0; cyc < 4; cyc = cyc + 1) begin
                wait_for_state(`S_NS_GREEN, (T_GREEN_MAX + T_YELLOW_FIXED + 10) * TICK_CYCLES);
                @(posedge clk); #1;
                if (duration_out > T_GREEN_MAX) begin
                    ok = 0;
                    $display("    VIOLATION cycle %0d: NS green duration=%0d > MAX=%0d",
                             cyc, duration_out, T_GREEN_MAX);
                end
                wait_for_state(`S_EW_GREEN, (T_GREEN_MAX + T_YELLOW_FIXED + 10) * TICK_CYCLES);
                @(posedge clk); #1;
                if (duration_out > T_GREEN_MAX) begin
                    ok = 0;
                    $display("    VIOLATION cycle %0d: EW green duration=%0d > MAX=%0d",
                             cyc, duration_out, T_GREEN_MAX);
                end
            end
            assert_check(ok, "TC08: duration never exceeds T_GREEN_MAX across 4 cycles");
        end

        // ------------------------------------------------------------------
        // TC-09  MIN green clamp — 4 full cycles
        // With EW at max density, NS has no advantage and gets T_GREEN_MIN.
        // Fail if any green duration drops below T_GREEN_MIN.
        // ------------------------------------------------------------------
        print_header("TC-09: MIN Green Clamp Verification (4 cycles)");
        do_reset;
        ns_density = 4'd0; ew_density = 4'd15; camera_valid = 1;
        begin : min_clamp
            integer cyc2; reg ok2; ok2 = 1;
            for (cyc2 = 0; cyc2 < 4; cyc2 = cyc2 + 1) begin
                wait_for_state(`S_NS_GREEN, (T_GREEN_MAX + T_YELLOW_FIXED + 10) * TICK_CYCLES);
                @(posedge clk); #1;
                if (duration_out < T_GREEN_MIN) begin
                    ok2 = 0;
                    $display("    VIOLATION cycle %0d: NS green duration=%0d < MIN=%0d",
                             cyc2, duration_out, T_GREEN_MIN);
                end
                wait_for_state(`S_EW_GREEN, (T_GREEN_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
                @(posedge clk); #1;
                if (duration_out < T_GREEN_MIN) begin
                    ok2 = 0;
                    $display("    VIOLATION cycle %0d: EW green duration=%0d < MIN=%0d",
                             cyc2, duration_out, T_GREEN_MIN);
                end
            end
            assert_check(ok2, "TC09: duration never falls below T_GREEN_MIN across 4 cycles");
        end

        // ------------------------------------------------------------------
        // TC-10  Starvation watchdog
        // Force ew_wait_timer above STARVATION_LIMIT (50).
        // Even with NS at max density, EW green must be cut to T_GREEN_MIN.
        // ------------------------------------------------------------------
        print_header("TC-10: Starvation Watchdog");
        do_reset;
        ns_density = 4'd15; ew_density = 4'd0; camera_valid = 1;

        wait_for_state(`S_NS_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);

        // Inject precondition: EW has been starved for 51 simulated seconds
        force dut.timing_inst.ew_wait_timer = 7'd51;
        @(posedge clk); #1;
        release dut.timing_inst.ew_wait_timer;

        // Starvation check fires when adaptive_timing_logic computes duration
        // for S_EW_GREEN: ew_wait_timer >= 50 forces final_duration = T_GREEN_MIN
        wait_for_state(`S_EW_GREEN, (T_GREEN_MAX + 10) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(duration_out === T_GREEN_MIN,
                     "TC10: EW green cut to T_GREEN_MIN after NS held EW starved");

        // ------------------------------------------------------------------
        // TC-11  Consecutive-max-bonus limiter
        // Force ns_consec_max to MAX_CONSEC_BONUS (2'd2).
        // With NS density = 15, raw would be T_GREEN_MAX, but Step D must
        // override it back to T_GREEN_BASE.
        // ------------------------------------------------------------------
        print_header("TC-11: Consecutive-Max-Bonus Limiter");
        do_reset;
        ns_density = 4'd15; ew_density = 4'd0; camera_valid = 1;

        wait_for_state(`S_NS_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);

        force dut.timing_inst.ns_consec_max = 2'd2;  // MAX_CONSEC_BONUS
        @(posedge clk); #1;
        release dut.timing_inst.ns_consec_max;

        assert_check(duration_out === T_GREEN_BASE,
                     "TC11: NS green reduced to T_GREEN_BASE after consecutive-max limit");

        // ------------------------------------------------------------------
        // TC-12  Pedestrian signals track light state correctly
        // Walk ON only for the active green direction; OFF in all others.
        // ------------------------------------------------------------------
        print_header("TC-12: Pedestrian Signal Verification");
        do_reset;
        ns_density = 4'd7; ew_density = 4'd7; camera_valid = 1;

        // S_NS_GREEN: NS walk ON, EW walk OFF
        wait_for_state(`S_NS_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);
        wait_ticks(1); @(posedge clk); #1;
        assert_check(ped_ns_led === 1'b1,  "TC12: NS walk ON  in S_NS_GREEN");
        assert_check(ped_ew_led === 1'b0,  "TC12: EW walk OFF in S_NS_GREEN");

        // S_NS_YELLOW: both OFF
        wait_for_state(`S_NS_YELLOW, (T_GREEN_MIN + 5) * TICK_CYCLES);
        wait_ticks(1); @(posedge clk); #1;
        assert_check(ped_ns_led === 1'b0,  "TC12: NS walk OFF in S_NS_YELLOW");
        assert_check(ped_ew_led === 1'b0,  "TC12: EW walk OFF in S_NS_YELLOW");

        // S_EW_GREEN: EW walk ON, NS walk OFF
        wait_for_state(`S_EW_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);
        wait_ticks(1); @(posedge clk); #1;
        assert_check(ped_ns_led === 1'b0,  "TC12: NS walk OFF in S_EW_GREEN");
        assert_check(ped_ew_led === 1'b1,  "TC12: EW walk ON  in S_EW_GREEN");

        // S_EW_YELLOW: both OFF
        wait_for_state(`S_EW_YELLOW, (T_GREEN_MIN + 5) * TICK_CYCLES);
        wait_ticks(1); @(posedge clk); #1;
        assert_check(ped_ns_led === 1'b0,  "TC12: NS walk OFF in S_EW_YELLOW");
        assert_check(ped_ew_led === 1'b0,  "TC12: EW walk OFF in S_EW_YELLOW");

        // ------------------------------------------------------------------
        // TC-13 (Black-box): Chứng minh lỗi Consecutive-Max-Bonus Limiter
        // Kịch bản: Hướng NS luôn đông xe (15), EW luôn vắng (0).
        // Kỳ vọng: Chu kỳ 1, 2, 3 NS nhận MAX (45s). Chu kỳ 4 phải bị ép về BASE (15s).
        // Thực tế RTL: Chu kỳ 4 vẫn nhận MAX (45s) -> Mạch bị kẹt (Latch) -> TEST FAIL.
        // ------------------------------------------------------------------
        print_header("TC-13 (Black-box): Anti-Latch Limiter Verification");
        do_reset;
        ns_density = 4'd15; ew_density = 4'd0; camera_valid = 1;

        // Chu kỳ 1
        wait_for_state(`S_NS_GREEN, (T_GREEN_MAX + 10) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(duration_out === T_GREEN_MAX, "Chu kỳ 1: NS nhận MAX (45s)");

        // Chu kỳ 2
        wait_for_state(`S_EW_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);
        wait_for_state(`S_NS_GREEN, (T_GREEN_MIN + 10) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(duration_out === T_GREEN_MAX, "Chu kỳ 2: NS nhận MAX (45s)");

        // Chu kỳ 3
        wait_for_state(`S_NS_YELLOW, (T_GREEN_MAX + 5) * TICK_CYCLES);
        wait_for_state(`S_EW_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);
        wait_for_state(`S_NS_GREEN, (T_GREEN_MIN + 10) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(duration_out === T_GREEN_BASE, "Chu kỳ 3: NS nhận BASE (15s) do đã bằng MAX-BONUS_LIMIT");
        
        // ------------------------------------------------------------------
        // TC-14 (Black-box): lỗi Starvation Watchdog
        // Kịch bản: EW đông xe (15) nhận 45s. EW Vàng 5s. 
        // Trong 50s đó, NS bị kẹt xe (15) phải chờ.
        // Kỳ vọng: Khi mạch chuyển lại sang EW, do NS từng phải chờ 50s, EW phải bị cắt xuống MIN (8s).
        // Thực tế RTL: EW vẫn nhận thời gian bình thường -> TEST FAIL.
        // ------------------------------------------------------------------
        print_header("TC-14 (Black-box): Starvation Watchdog Verification");
        do_reset;
        
        // Ban đầu cho EW đông, NS vắng để EW lấy đủ 45s
        ns_density = 4'd0; ew_density = 4'd15; camera_valid = 1;

        // Đợi tới lúc EW bắt đầu đèn Xanh
        wait_for_state(`S_EW_GREEN, (T_YELLOW_FIXED + 10) * TICK_CYCLES);
        
        // Ngay khi EW đang Xanh, đột ngột NS kẹt cứng.
        // NS bắt đầu đếm thời gian chờ. Chờ EW Xanh(45s) + Vàng(5s) = đúng 50s.
        ns_density = 4'd15; 
        
        // NS nhận đèn Xanh (Nhận 8s vì lúc này ew_density cũng bằng 15)
        wait_for_state(`S_NS_GREEN, (T_GREEN_MAX + 15) * TICK_CYCLES);
        
        // NS chạy xong Xanh và Vàng, chuẩn bị quay lại EW_GREEN
        wait_for_state(`S_EW_GREEN, (T_GREEN_MAX + 15) * TICK_CYCLES);
        @(posedge clk); #1;
        
        assert_check(duration_out === T_GREEN_MIN, "TC14: EW green cut to T_GREEN_MIN after NS held EW starved for 50s");
        // ======================================================================
        // Summary
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
    // Watchdog — prevent infinite hang on unexpected bugs
    // =========================================================================
    initial begin
        #80_000_000;
        $display("[WATCHDOG] 80ms sim limit reached. Forcing stop.");
        $finish;
    end

endmodule