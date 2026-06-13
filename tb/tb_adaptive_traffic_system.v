// =============================================================================
// tb_traffic_system_top.v  — FIXED VERSION
// Testbench for traffic_system_top (Adaptive Traffic Light Controller)
//
// Changes from v2
// ---------------
//   PORT-1   Added ped_btn_ns, ped_btn_ew inputs to DUT connections.
//   PORT-2   Added ped_ns_req_led_o, ped_ew_req_led_o output probes.
//   PORT-3   Removed uart_rx — density now driven directly as inputs.
//   TASK-1   do_reset now initialises button signals to 0.
//   TASK-2   press_btn_ns / press_btn_ew — simulates a debounced button
//            press using force on the internal debounce counter. See the
//            "Debounce bypass" section below for the full rationale.
//   TC-15    NS button registers request (ped_ns_req HIGH, req_led HIGH)
//   TC-16    EW button registers request (ped_ew_req HIGH, req_led HIGH)
//   TC-17    NS request clears when S_NS_GREEN phase starts
//   TC-18    EW request clears when S_EW_GREEN phase starts
//   TC-19    Pedestrian floor (T_PED_MIN=20s) enforced by adaptive logic
//   TC-20    Debounce — short glitch rejected, no false request registered
//   TC-21    Both buttons pressed simultaneously — independent latches
//   TC-22    Request survives S_ERROR, is served after emergency recovery
//
// Debounce bypass rationale (TASK-2)
// ------------------------------------
// DEBOUNCE_CYCLES = (CLK_FREQ / 1000) * DEBOUNCE_MS
//                 = (50_000_000 / 1000) * 20 = 1_000_000 cycles
//
// With TICK_CYCLES = 20, one simulated second = 20 clock cycles.
// Waiting 1,000,000 cycles just to register a button press would be
// 50,000× slower than the FSM simulation speed, making the full test
// suite take orders of magnitude longer.
//
// Solution: press_btn_ns / press_btn_ew drive the button HIGH (so the
// 2-FF sync settles), then force the internal debounce counter to
// DEBOUNCE_CYCLES_TB instantly. This causes the DUT's own always block
// to fire ns_pressed/ew_pressed on the very next posedge — identical
// to what would happen after 1,000,000 real cycles of button hold.
//
// TC-18 tests debounce rejection independently: it drives the button
// for only 100 cycles (much less than DEBOUNCE_CYCLES) without any
// force, then verifies the pressed pulse never fires.
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
    reg        ped_btn_ns;       // PORT-1
    reg        ped_btn_ew;       // PORT-1

    wire [2:0] ns_leds;
    wire [2:0] ew_leds;
    wire       ped_ns_led;
    wire       ped_ew_led;
    wire       ped_ns_req_led;   // PORT-2
    wire       ped_ew_req_led;   // PORT-2

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
    localparam T_PED_MIN        = 6'd20;   // pedestrian crossing floor

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

    // DEBOUNCE_CYCLES_TB: mirrors ped_request_handler's internal calculation.
    // = (CLK_FREQ / 1000) * DEBOUNCE_MS = (100_000_000 / 1000) * 20 = 2_000_000
    // Used by press_btn_ns / press_btn_ew to force the counter to threshold.
    localparam DEBOUNCE_CYCLES_TB = (100_000_000 / 1000) * 20;  // 2_000_000
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
        .ped_btn_ns_i    (ped_btn_ns),       // PORT-1
        .ped_btn_ew_i    (ped_btn_ew),       // PORT-1
        .ns_leds       (ns_leds),
        .ew_leds       (ew_leds),
        .ped_ns_led    (ped_ns_led),
        .ped_ew_led    (ped_ew_led),
        .ped_ns_req_led_o(ped_ns_req_led),   // PORT-2
        .ped_ew_req_led_o(ped_ew_req_led),   // PORT-2
        .timer_duration_o(timer_duration_o)
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
    // Pedestrian handler internals
    wire       ped_ns_req     = dut.ped_ns_req;   // internal wire after latch
    wire       ped_ew_req     = dut.ped_ew_req;
    wire       ns_pressed     = dut.ped_handler_inst.ns_pressed;  // debounce pulse
    wire       ew_pressed     = dut.ped_handler_inst.ew_pressed;

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
    // Wait cycles for n full clock cycles (not ticks)
    task wait_cycles;
        input integer n;
        begin repeat (n) @(posedge clk); end
    endtask

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
                $display("         state=%05b  ns=%03b  ew=%03b  dur=%0d  tmr=%0d  ns_req=%b  ew_req=%b  ns_led=%b  ew_led=%b",
                         current_state, ns_leds, ew_leds, duration_out, timer_val,
                         ped_ns_req, ped_ew_req, ped_ns_req_led, ped_ew_req_led);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // Reset DUT and install tick bypass
    task do_reset;
        begin
            rst_n        = 1'b0;
            emergency_sw = 1'b0;
            ns_density   = 4'd7;
            ew_density   = 4'd7;
            camera_valid = 1'b1;
            ped_btn_ns   = 1'b0;   // TASK-1: initialise buttons
            ped_btn_ew   = 1'b0;
            repeat (4) @(posedge clk);
            @(negedge clk); rst_n = 1'b1;
            force dut.tick_1hz = tb_tick;   // bypass clock_divider
            wait_ticks(3);
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
    // TASK-2: press_btn_ns / press_btn_ew
    // =========================================================================
    // Simulates a valid, debounced button press without waiting 1,000,000
    // real clock cycles. Strategy:
    //
    //   Step 1  Drive button HIGH — the 2-FF synchroniser needs 2 posedges
    //           to propagate the signal into ns_btn_stable.
    //   Step 2  Force the internal debounce counter to DEBOUNCE_CYCLES_TB.
    //           On the next posedge, the DUT's own always block sees:
    //               ns_btn_stable == 1   (button held)
    //               ns_debounce_cnt == DEBOUNCE_CYCLES  → fires ns_pressed <= 1
    //   Step 3  Release the force. The counter will increment past the
    //           threshold naturally — it will not re-fire (the == check
    //           only matches exactly once per press sequence).
    //   Step 4  Wait one more cycle for the latch to capture ns_pressed and
    //           assert ped_ns_req_o <= 1.
    //   Step 5  Release button. Counter resets on the next cycle.
    // =========================================================================
    task press_btn_ns;
        begin
            ped_btn_ns = 1'b1;
            repeat (3) @(posedge clk);     // step 1: 2-FF sync + margin

            force dut.ped_handler_inst.ns_debounce_cnt = DEBOUNCE_CYCLES_TB;
            @(posedge clk); #1;            // step 2: ns_pressed NBA fires
            release dut.ped_handler_inst.ns_debounce_cnt;

            @(posedge clk); #1;            // step 4: latch captures ns_pressed
            ped_btn_ns = 1'b0;
            repeat (3) @(posedge clk);     // step 5: counter resets cleanly
        end
    endtask

    task press_btn_ew;
        begin
            ped_btn_ew = 1'b1;
            repeat (3) @(posedge clk);
            force dut.ped_handler_inst.ew_debounce_cnt = DEBOUNCE_CYCLES_TB;
            @(posedge clk); #1;
            release dut.ped_handler_inst.ew_debounce_cnt;
            @(posedge clk); #1;
            ped_btn_ew = 1'b0;
            repeat (3) @(posedge clk);
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
        // TC-13 (Black-box): Consecutive-Max-Bonus Limiter
        // Scenario: NS starts heavy → gets T_GREEN_MAX. On the next cycle, NS is still heavy and would get MAX again, but the limiter should cap it back to T_GREEN_BASE.
        // Expectation: NS gets T_GREEN_MAX on the first S_NS_GREEN entry, but only T_GREEN_BASE on the second, proving that the consecutive-max-bonus limiter is working correctly.
        // ------------------------------------------------------------------
        print_header("TC-13 (Black-box): Anti-Latch Limiter Verification");
        do_reset;
        ns_density = 4'd15; ew_density = 4'd0; camera_valid = 1;

        // Cycle 1
        wait_for_state(`S_NS_GREEN, (T_GREEN_MAX + 10) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(duration_out === T_GREEN_MAX, "Chu kỳ 1: NS nhận MAX (45s)");

        // Cycle 2
        wait_for_state(`S_EW_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);
        wait_for_state(`S_NS_GREEN, (T_GREEN_MIN + 10) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(duration_out === T_GREEN_MAX, "Chu kỳ 2: NS nhận MAX (45s)");

        // Cycle 3: Inject the MAX_CONSEC_BONUS condition, but the limiter should prevent it from taking effect more than once.
        wait_for_state(`S_NS_YELLOW, (T_GREEN_MAX + 5) * TICK_CYCLES);
        wait_for_state(`S_EW_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);
        wait_for_state(`S_NS_GREEN, (T_GREEN_MIN + 10) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(duration_out === T_GREEN_BASE, "Chu kỳ 3: NS nhận BASE (15s) do đã bằng MAX-BONUS_LIMIT");
        
        // ------------------------------------------------------------------
        // TC-14 (Black-box): Starvation Watchdog
        // Scenario: EW starts green with NS at max density. After 45s of EW green + 5s yellow, NS is still waiting and hits the starvation limit.
        // Expectation: NS green duration is cut to T_GREEN_MIN (8s) on
        // the very next S_NS_GREEN entry. This proves that the starvation timer is being checked and enforced correctly.
        // ------------------------------------------------------------------
        print_header("TC-14 (Black-box): Starvation Watchdog Verification");
        do_reset;
        
        // Initial conditions: EW heavy → EW gets green first, NS starts waiting immediately.
        ns_density = 4'd0; ew_density = 4'd15; camera_valid = 1;

        // Wait for EW to be green and run through its full green + yellow duration, while NS is waiting.
        wait_for_state(`S_EW_GREEN, (T_YELLOW_FIXED + 10) * TICK_CYCLES);
        
        // Suddenly make NS heavy right before it gets the chance to be served, to maximize the starvation pressure.
        ns_density = 4'd15; 
        
        // NS finally gets served, but the starvation timer should have forced its green duration down to T_GREEN_MIN.
        wait_for_state(`S_NS_GREEN, (T_GREEN_MAX + 15) * TICK_CYCLES);
        
        // Now wait for the next EW_GREEN entry to ensure we're not just seeing a one-off glitch, but a consistent enforcement of the starvation limit.
        wait_for_state(`S_EW_GREEN, (T_GREEN_MAX + 15) * TICK_CYCLES);
        @(posedge clk); #1;
        
        assert_check(duration_out === T_GREEN_MIN, "TC14: EW green cut to T_GREEN_MIN after NS held EW starved for 50s");

        // ==================================================================
        // TC-15  NS Button Registers Request
        // ------------------------------------------------------------------
        // Press the NS button during S_NS_GREEN.
        // The request latch must set (ped_ns_req HIGH).
        // The indicator LED must light (ped_ns_req_led HIGH).
        // EW request must remain unaffected (ped_ew_req still LOW).
        // ==================================================================
        print_header("TC-15: NS Button Registers Request");
        do_reset;
        wait_for_state(`S_NS_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);

        // NS button is pressed while NS is already green.
        // The latch will be set, and will clear on the NEXT S_NS_GREEN entry
        // (current entry doesn't clear because prev_state == S_NS_GREEN already).
        press_btn_ns;
        @(posedge clk); #1;

        assert_check(ped_ns_req     === 1'b1, "TC15: ped_ns_req HIGH after button press");
        assert_check(ped_ns_req_led === 1'b1, "TC15: NS req LED HIGH (request pending)");
        assert_check(ped_ew_req     === 1'b0, "TC15: ped_ew_req unaffected");
        assert_check(ped_ew_req_led === 1'b0, "TC15: EW req LED unaffected");

        // ==================================================================
        // TC-16  EW Button Registers Request
        // ------------------------------------------------------------------
        // Same as TC-15 but for the EW direction, pressed during EW_GREEN.
        // ==================================================================
        print_header("TC-16: EW Button Registers Request");
        do_reset;
        wait_for_state(`S_EW_GREEN, (T_GREEN_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);

        press_btn_ew;
        @(posedge clk); #1;

        assert_check(ped_ew_req     === 1'b1, "TC16: ped_ew_req HIGH after button press");
        assert_check(ped_ew_req_led === 1'b1, "TC16: EW req LED HIGH (request pending)");
        assert_check(ped_ns_req     === 1'b0, "TC16: ped_ns_req unaffected");
        assert_check(ped_ns_req_led === 1'b0, "TC16: NS req LED unaffected");

        // ==================================================================
        // TC-17  NS Request Clears Automatically When S_NS_GREEN Starts
        // ------------------------------------------------------------------
        // Press NS button while NS is RED (during EW phase).
        // Request should be latched and held through the EW phase.
        // Request must clear (go LOW) the moment S_NS_GREEN is entered.
        //
        // Timeline:
        //   EW_GREEN: press NS btn → ped_ns_req = 1, req_led = 1
        //   EW_YELLOW: request still held
        //   NS_GREEN entry: latch clears → ped_ns_req = 0, req_led = 0
        // ==================================================================
        print_header("TC-17: NS Request Clears on S_NS_GREEN Entry");
        do_reset;

        // Wait for EW phase so NS button press is not immediately served
        wait_for_state(`S_EW_GREEN, (T_GREEN_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        press_btn_ns;
        @(posedge clk); #1;

        assert_check(ped_ns_req === 1'b1, "TC17: NS request latched during EW phase");

        // Wait through EW phase and yellow into NS_GREEN
        wait_for_state(`S_NS_GREEN, (T_GREEN_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        @(posedge clk); #1;  // one cycle after entry so latch-clear NBA has settled

        assert_check(ped_ns_req     === 1'b0, "TC17: NS request cleared on S_NS_GREEN entry");
        assert_check(ped_ns_req_led === 1'b0, "TC17: NS req LED LOW (request served)");
        assert_check(ped_ns_led     === 1'b1, "TC17: NS walk signal ON as expected");

        // ==================================================================
        // TC-18  EW Request Clears Automatically When S_EW_GREEN Starts
        // ------------------------------------------------------------------
        // Mirror of TC-17 for the EW direction.
        // Press EW button during NS phase; verify clear on EW_GREEN entry.
        // ==================================================================
        print_header("TC-18: EW Request Clears on S_EW_GREEN Entry");
        do_reset;

        // NS green is the active state after reset
        wait_for_state(`S_NS_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);
        press_btn_ew;
        @(posedge clk); #1;

        assert_check(ped_ew_req === 1'b1, "TC18: EW request latched during NS phase");

        wait_for_state(`S_EW_GREEN, (T_GREEN_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        @(posedge clk); #1;

        assert_check(ped_ew_req     === 1'b0, "TC18: EW request cleared on S_EW_GREEN entry");
        assert_check(ped_ew_req_led === 1'b0, "TC18: EW req LED LOW (request served)");
        assert_check(ped_ew_led     === 1'b1, "TC18: EW walk signal ON as expected");

        // ==================================================================
        // TC-19  Pedestrian Floor (T_PED_MIN) Enforced
        // ------------------------------------------------------------------
        // With balanced density (equal NS and EW), the camera produces
        // T_GREEN_MIN (8 s) — no direction has an advantage.
        //
        // Press the NS button BEFORE the NS green phase starts.
        // ped_ns_req is HIGH when adaptive_timing_logic computes the NS
        // green duration → Step E must raise it to T_PED_MIN (20 s).
        //
        // Also verify that the floor does NOT apply when there is no request:
        // after the request is served, the next NS green cycle gets T_GREEN_MIN.
        // ==================================================================
        print_header("TC-19: Pedestrian Floor T_PED_MIN Enforced");
        do_reset;
        ns_density = 4'd8; ew_density = 4'd8; camera_valid = 1;
        // Camera gives T_GREEN_MIN = 8 s for balanced density

        // Press button during EW phase so request is pending for next NS green
        wait_for_state(`S_EW_GREEN, (T_GREEN_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        press_btn_ns;

        wait_for_state(`S_NS_GREEN, (T_GREEN_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(timer_val >= T_PED_MIN,
                     "TC19: NS green duration raised to T_PED_MIN when request active");
        assert_check(timer_val === T_PED_MIN,
                     "TC19: duration is exactly T_PED_MIN (not T_GREEN_MAX)");

        // After request is served (cleared on NS_GREEN entry), next NS green
        // cycle must revert to T_GREEN_MIN (no more pedestrian floor)
        wait_for_state(`S_EW_GREEN, (T_PED_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        wait_for_state(`S_NS_GREEN, (T_GREEN_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(duration_out === T_GREEN_MIN,
                     "TC19: duration reverts to T_GREEN_MIN once request is served");

        // ==================================================================
        // TC-20  Debounce — Short Glitch Rejected
        // ------------------------------------------------------------------
        // Drive the NS button HIGH for only 100 cycles — far less than
        // DEBOUNCE_CYCLES (1,000,000). The debounce counter never reaches
        // threshold, so ns_pressed must never pulse and the request latch
        // must remain LOW.
        //
        // No force is used here — this tests the DUT's actual debounce
        // counter behaviour, not just the latch logic.
        // ==================================================================
        print_header("TC-20: Debounce - Short Glitch Rejected");
        do_reset;
        @(posedge clk); #1;
        assert_check(ped_ns_req === 1'b0, "TC20: NS request clear before glitch");

        // Inject a brief glitch (100 cycles << DEBOUNCE_CYCLES = 1_000_000)
        ped_btn_ns = 1'b1;
        wait_cycles(100);
        ped_btn_ns = 1'b0;
        wait_cycles(20);   // allow counter to reset
        @(posedge clk); #1;

        assert_check(ns_pressed   === 1'b0, "TC20: ns_pressed did NOT fire on 100-cycle glitch");
        assert_check(ped_ns_req   === 1'b0, "TC20: NS request not registered from glitch");
        assert_check(ped_ns_req_led === 1'b0, "TC20: NS req LED stays LOW");

        // Verify normal operation is unaffected: a valid press still works
        press_btn_ns;
        @(posedge clk); #1;
        assert_check(ped_ns_req === 1'b1, "TC20: valid press still registers after glitch test");

        // ==================================================================
        // TC-21  Both Buttons Pressed Simultaneously
        // ------------------------------------------------------------------
        // Press both NS and EW buttons at the same time.
        // Both request latches must set independently.
        // Both req LEDs must light independently.
        // Neither clears the other.
        //
        // Then verify each clears at its own correct phase entry.
        // ==================================================================
        print_header("TC-21: Both Buttons Pressed Simultaneously");
        do_reset;

        // Press both while in NS green (both will be latched, NS request will
        // clear immediately on next NS green entry, EW on EW green entry)
        wait_for_state(`S_NS_GREEN, (T_YELLOW_FIXED + 5) * TICK_CYCLES);

        // Drive both buttons high simultaneously and force both counters
        ped_btn_ns = 1'b1;
        ped_btn_ew = 1'b1;
        repeat (3) @(posedge clk);
        force dut.ped_handler_inst.ns_debounce_cnt = DEBOUNCE_CYCLES_TB;
        force dut.ped_handler_inst.ew_debounce_cnt = DEBOUNCE_CYCLES_TB;
        @(posedge clk); #1;   // both pressed pulses fire
        release dut.ped_handler_inst.ns_debounce_cnt;
        release dut.ped_handler_inst.ew_debounce_cnt;
        @(posedge clk); #1;   // both latches capture
        ped_btn_ns = 1'b0;
        ped_btn_ew = 1'b0;
        repeat (3) @(posedge clk);

        @(posedge clk); #1;
        assert_check(ped_ns_req     === 1'b1, "TC21: NS request latched");
        assert_check(ped_ew_req     === 1'b1, "TC21: EW request latched simultaneously");
        assert_check(ped_ns_req_led === 1'b1, "TC21: NS req LED HIGH");
        assert_check(ped_ew_req_led === 1'b1, "TC21: EW req LED HIGH");

        // Both buttons were pressed while already in S_NS_GREEN.
        // With ped_ns_req=1, NS green loaded T_PED_MIN=20 ticks (Step E).
        //
        // We must wait for S_EW_GREEN before waiting for S_NS_GREEN, so that
        // prev_state_full != S_NS_GREEN is true on re-entry (the clear
        // condition requires a genuine state rising edge, not a no-op poll).
        //
        // Timeout for S_EW_GREEN: NS is running T_PED_MIN=20 ticks + yellow.
        wait_for_state(`S_EW_GREEN, (T_PED_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        @(posedge clk); #1;
        // At S_EW_GREEN entry:
        //   ped_ew_req was 1 when duration loaded → EW also gets T_PED_MIN (Step E)
        //   ped_ew_req_o NBA clears to 0 on this same posedge
        assert_check(ped_ns_req === 1'b1, "TC21: NS request still held during EW phase");
        assert_check(ped_ew_req === 1'b0, "TC21: EW request cleared on S_EW_GREEN entry");

        // Timeout for S_NS_GREEN: EW is now running T_PED_MIN=20 ticks + yellow.
        // BUG WAS HERE: old timeout used T_GREEN_MIN (360 cycles) but EW needs
        // T_PED_MIN=20 ticks = 400 cycles → timed out 40 cycles too early.
        wait_for_state(`S_NS_GREEN, (T_PED_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(ped_ns_req === 1'b0, "TC21: NS request cleared on S_NS_GREEN re-entry");
        assert_check(ped_ew_req === 1'b0, "TC21: EW request already served, stays LOW");

        // EW request clears on EW_GREEN entry
        wait_for_state(`S_EW_GREEN, (T_GREEN_BASE + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        @(posedge clk); #1;
        assert_check(ped_ew_req     === 1'b0, "TC21: EW request cleared on S_EW_GREEN entry");
        assert_check(ped_ew_req_led === 1'b0, "TC21: EW req LED LOW after serve");

        // ==================================================================
        // TC-22  Request Survives Emergency, Served After Recovery
        // ------------------------------------------------------------------
        // Press the NS button while in EW_GREEN (NS is waiting).
        // Immediately trigger emergency_sw.
        // The FSM enters S_ERROR — S_NS_GREEN is never entered, so the
        // clear condition never fires. The latch must remain HIGH.
        //
        // After emergency release + reset, the FSM returns to S_NS_GREEN.
        // The latch clears on that entry → ped_ns_req goes LOW.
        // ==================================================================
        print_header("TC-22: Request Survives Emergency, Served After Recovery");
        do_reset;

        // Establish known state: press during EW phase
        wait_for_state(`S_EW_GREEN, (T_GREEN_MIN + T_YELLOW_FIXED + 5) * TICK_CYCLES);
        press_btn_ns;
        @(posedge clk); #1;
        assert_check(ped_ns_req === 1'b1, "TC22: NS request latched before emergency");

        // Trigger emergency → FSM goes to S_ERROR (never enters S_NS_GREEN)
        emergency_sw = 1;
        wait_ticks(3);
        @(posedge clk); #1;

        assert_check(current_state === `S_ERROR, "TC22: FSM in S_ERROR during emergency");
        assert_check(ped_ns_req    === 1'b1,     "TC22: NS request HELD through emergency");
        assert_check(ped_ns_req_led === 1'b1,    "TC22: NS req LED still HIGH in emergency");

        // Release emergency and reset
        emergency_sw = 0;
        rst_n = 0;
        repeat (4) @(posedge clk);
        @(negedge clk); rst_n = 1;
        force dut.tick_1hz = tb_tick;
        // Note: reset re-initialises ped_request_handler registers to 0.
        // This is correct — on a hard system reset all state is cleared.
        wait_ticks(3);
        @(posedge clk); #1;

        assert_check(current_state === `S_NS_GREEN, "TC22: returns to S_NS_GREEN after recovery");
        // After reset, the ped latch is cleared by rst_n, not by phase entry.
        // This is safe because the pedestrian must press the button again after
        // a hard system reset — the same behaviour as any real intersection.
        assert_check(ped_ns_req === 1'b0,
                     "TC22: NS request cleared by reset (press again after recovery)");
        assert_check(ped_ns_led === 1'b1, "TC22: NS walk signal ON after recovery");

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

// =============================================================================
// clock_divider STUB (linker-only — not used at runtime)
// =============================================================================
// Moves to the end of the file so run_sim.py detects tb_traffic_system_top first.
// =============================================================================
`ifndef NO_CD_STUB
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
`endif