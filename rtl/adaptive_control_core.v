// ============================================================================
// adaptive_control_core.v  (module name: adaptive_timing_logic)
// Adaptive Timing ALU — computes optimal phase durations from density data
//
// Purpose
// -------
//   Pure computational module that determines how long each traffic phase
//   (green or yellow) should last, based on real-time camera density data,
//   pedestrian requests, starvation prevention, and safety limits.
//
//   This module does NOT contain any FSM state transitions — it only
//   computes a duration value consumed by traffic_controller_core.
//
// Algorithm Pipeline (5 steps, combinational)
// -------------------------------------------
//   Step A: Raw Duration from Camera Density
//           If camera is valid → scale green time by density difference.
//           If camera is invalid → fall back to T_GREEN_BASE (15 s).
//           Yellow phases always use T_YELLOW_FIXED (5 s).
//
//   Step B: Hard Safety Clamp
//           Enforce minimum (8 s) and maximum (45 s) green duration.
//           Yellow is always fixed at 5 s.
//
//   Step C: Starvation Override
//           If the opposing direction has waited ≥ STARVATION_LIMIT (50 s)
//           without receiving green, force the current green to T_GREEN_MIN
//           so the starved direction gets its turn quickly.
//
//   Step D: Consecutive Max-Bonus Limiter
//           If a direction has received T_GREEN_MAX for MAX_CONSEC_BONUS (2)
//           consecutive cycles, cap it at T_GREEN_BASE to prevent one
//           direction from monopolising the intersection.
//
//   Step E: Pedestrian Minimum Floor
//           If a pedestrian request is active for the upcoming green phase,
//           ensure at least T_PED_MIN (20 s) to allow safe crossing.
//           This can only RAISE the duration, never lower it.
//
// Sequential State
// ----------------
//   - ns_wait_timer / ew_wait_timer: count seconds each direction has waited
//   - ns_consec_max / ew_consec_max: track consecutive max-duration cycles
//   - prev_state: edge detection for state transitions
//
// Timing Parameters (defaults)
// ----------------------------
//   T_GREEN_MIN      = 8 s     Minimum green phase duration
//   T_GREEN_BASE     = 15 s    Default duration when no camera data
//   T_GREEN_MAX      = 45 s    Maximum green phase duration
//   T_YELLOW_FIXED   = 5 s     Fixed yellow clearance interval
//   T_GREEN_STEP     = 2 s     Duration bonus per density unit difference
//   MAX_BONUS        = 30 s    Maximum total density bonus
//   STARVATION_LIMIT = 50 s    Wait time before starvation override kicks in
//   MAX_CONSEC_BONUS = 2       Max consecutive cycles at T_GREEN_MAX
//   T_PED_MIN        = 20 s    Minimum green when pedestrian request is active
//
// Ports
// -----
//   clk, rst_n                — System clock and reset
//   tick_1hz                  — 1 Hz pulse (drives wait timers)
//   next_state_i[4:0]         — The state the FSM is about to transition into
//   current_state_i[4:0]      — Current FSM state
//   ns_density_i[3:0]         — N-S vehicle density from camera (0-15)
//   ew_density_i[3:0]         — E-W vehicle density from camera (0-15)
//   camera_valid_i            — HIGH = camera data is fresh
//   ped_ns_req_i              — N-S pedestrian crossing request pending
//   ped_ew_req_i              — E-W pedestrian crossing request pending
//   duration_o[5:0]           — Computed phase duration (consumed by core FSM)
// ============================================================================
module adaptive_timing_logic (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_1hz,

    // FSM state interface (from traffic_controller_core)
    input  wire [4:0]  next_state_i,
    input  wire [4:0]  current_state_i,

    // Camera density interface (from uart_camera_rx)
    input  wire [3:0]  ns_density_i,
    input  wire [3:0]  ew_density_i,
    input  wire        camera_valid_i,

    // Pedestrian request interface (from ped_request_handler)
    input  wire        ped_ns_req_i,
    input  wire        ped_ew_req_i,

    // Mode and loop sensor inputs
    input  wire        mode_select_i,    // 0 = Camera Density, 1 = Loop Sensor Gap-Out
    input  wire        sensor_ns_i,      // N-S loop sensor
    input  wire        sensor_ew_i,      // E-W loop sensor
    input  wire [5:0]  timer_count_i,    // Current countdown timer value

    // Computed duration output (to traffic_controller_core)
    output reg  [5:0]  duration_o,
    output reg         reload_en_o       // Timer reload enable pulse
);

    // =========================================================================
    // Timing Parameters
    // =========================================================================
    parameter T_GREEN_MIN      = 6'd8;
    parameter T_GREEN_BASE     = 6'd15;
    parameter T_GREEN_MAX      = 6'd45;
    parameter T_YELLOW_FIXED   = 6'd5;
    parameter T_GREEN_STEP     = 6'd2;
    parameter MAX_BONUS        = 6'd30;
    parameter STARVATION_LIMIT = 7'd50;
    parameter MAX_CONSEC_BONUS = 2'd2;
    parameter T_PED_MIN        = 6'd20;
    parameter EXT_TIME         = 6'd3;   // Extension time step for Gap-Out mode

    // =========================================================================
    // State Encoding (mirrors traffic_controller_core)
    // =========================================================================
    parameter S_NS_GREEN  = 5'b00001;
    parameter S_NS_YELLOW = 5'b00010;
    parameter S_EW_GREEN  = 5'b00100;
    parameter S_EW_YELLOW = 5'b01000;
    parameter S_ERROR     = 5'b10000;

    // =========================================================================
    // Sequential State Registers
    // =========================================================================
    reg [6:0] ns_wait_timer;   // Seconds N-S has been waiting (caps at 127)
    reg [6:0] ew_wait_timer;   // Seconds E-W has been waiting (caps at 127)
    reg [1:0] ns_consec_max;   // Consecutive cycles N-S got T_GREEN_MAX
    reg [1:0] ew_consec_max;   // Consecutive cycles E-W got T_GREEN_MAX
    reg [4:0] prev_state;      // Previous FSM state (for edge detection)
    reg [5:0] active_green_timer; // Counts total green phase duration (seconds)

    // =========================================================================
    // Active Green Timer Logic (Sequential)
    //   Tracks total elapsed green time for the active phase. Resets on entry
    //   to a green state, increments every 1Hz tick during green state.
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_green_timer <= 0;
        end else begin
            if ((current_state_i == S_NS_GREEN && prev_state != S_NS_GREEN) ||
                (current_state_i == S_EW_GREEN && prev_state != S_EW_GREEN)) begin
                active_green_timer <= 0;
            end else if (tick_1hz && (current_state_i == S_NS_GREEN || current_state_i == S_EW_GREEN)) begin
                active_green_timer <= active_green_timer + 1;
            end
        end
    end

    // =========================================================================
    // Section 1: Wait Timers & Consecutive-Max Counters (Sequential)
    //
    //   Wait timers: increment every second while the OPPOSING direction
    //   has the active phase. Reset when their own green phase starts.
    //
    //   Consecutive-max counters: track how many times in a row a direction
    //   received the maximum green duration. Used by Step D to prevent
    //   monopolisation.
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ns_wait_timer <= 0;
            ew_wait_timer <= 0;
            ns_consec_max <= 0;
            ew_consec_max <= 0;
            prev_state    <= S_NS_GREEN;
        end else begin
            prev_state <= current_state_i;

            // --- Wait timer increment (1 Hz) ---
            if (tick_1hz) begin
                // N-S waits while E-W is active
                if (current_state_i == S_EW_GREEN || current_state_i == S_EW_YELLOW)
                    if (ns_wait_timer < 7'd127) ns_wait_timer <= ns_wait_timer + 1;
                // E-W waits while N-S is active
                if (current_state_i == S_NS_GREEN || current_state_i == S_NS_YELLOW)
                    if (ew_wait_timer < 7'd127) ew_wait_timer <= ew_wait_timer + 1;
            end

            // --- Wait timer reset on green entry ---
            if (current_state_i == S_NS_GREEN && prev_state != S_NS_GREEN)
                ns_wait_timer <= 0;
            if (current_state_i == S_EW_GREEN && prev_state != S_EW_GREEN)
                ew_wait_timer <= 0;

            // --- Consecutive-max tracking ---
            if (current_state_i == S_NS_GREEN && prev_state != S_NS_GREEN) begin
                if (duration_o == T_GREEN_MAX)
                    ns_consec_max <= (ns_consec_max < MAX_CONSEC_BONUS)
                                      ? ns_consec_max + 1 : MAX_CONSEC_BONUS;
                else
                    ns_consec_max <= 0;
            end

            if (current_state_i == S_EW_GREEN && prev_state != S_EW_GREEN) begin
                if (duration_o == T_GREEN_MAX)
                    ew_consec_max <= (ew_consec_max < MAX_CONSEC_BONUS)
                                      ? ew_consec_max + 1 : MAX_CONSEC_BONUS;
                else
                    ew_consec_max <= 0;
            end
        end
    end

    // =========================================================================
    // Section 2: Duration Calculation Pipeline (Combinational)
    //
    //   Five sequential processing steps refine the output duration.
    //   Each step can only narrow or adjust — they form a priority chain.
    // =========================================================================
    reg [5:0] raw_duration;      // After Step A
    reg [5:0] clamped_duration;  // After Step B
    reg [5:0] final_duration;    // After Steps C, D, E

    wire [3:0] ns_den = ns_density_i;
    wire [3:0] ew_den = ew_density_i;

    always @(*) begin

        // -----------------------------------------------------------------
        // Step A: Raw duration from camera density data
        //
        //   Green phases: scale duration based on which direction has more
        //   traffic. The busier direction gets a longer green.
        //   Formula: BASE + (density_difference × STEP), capped at MAX_BONUS.
        //   If camera is invalid: use T_GREEN_BASE (fixed fallback).
        //   Yellow phases: always T_YELLOW_FIXED (no adaptation).
        // -----------------------------------------------------------------
        if (!camera_valid_i) begin
            raw_duration = T_GREEN_BASE;
        end else begin
            case (next_state_i)
                S_NS_GREEN: begin
                    if (ns_den > ew_den) begin
                        raw_duration = T_GREEN_BASE + ((ns_den - ew_den) * T_GREEN_STEP);
                        if (raw_duration > T_GREEN_BASE + MAX_BONUS)
                            raw_duration = T_GREEN_BASE + MAX_BONUS;
                    end else begin
                        raw_duration = T_GREEN_MIN;
                    end
                end
                S_EW_GREEN: begin
                    if (ew_den > ns_den) begin
                        raw_duration = T_GREEN_BASE + ((ew_den - ns_den) * T_GREEN_STEP);
                        if (raw_duration > T_GREEN_BASE + MAX_BONUS)
                            raw_duration = T_GREEN_BASE + MAX_BONUS;
                    end else begin
                        raw_duration = T_GREEN_MIN;
                    end
                end
                S_NS_YELLOW, S_EW_YELLOW: raw_duration = T_YELLOW_FIXED;
                default:                   raw_duration = T_GREEN_BASE;
            endcase
        end

        // -----------------------------------------------------------------
        // Step B: Hard MIN / MAX safety clamp
        //
        //   Ensures green duration is always within [T_GREEN_MIN, T_GREEN_MAX].
        //   Yellow phases bypass this clamp (always exactly T_YELLOW_FIXED).
        // -----------------------------------------------------------------
        if (next_state_i == S_NS_YELLOW || next_state_i == S_EW_YELLOW) begin
            clamped_duration = T_YELLOW_FIXED;
        end else if (raw_duration < T_GREEN_MIN) begin
            clamped_duration = T_GREEN_MIN;
        end else if (raw_duration > T_GREEN_MAX) begin
            clamped_duration = T_GREEN_MAX;
        end else begin
            clamped_duration = raw_duration;
        end

        // -----------------------------------------------------------------
        // Step C: Starvation override
        //
        //   If the opposing direction has been waiting too long, force
        //   the current direction's green to minimum so it finishes quickly.
        // -----------------------------------------------------------------
        final_duration = clamped_duration;

        if (next_state_i == S_NS_GREEN && ew_wait_timer >= STARVATION_LIMIT)
            final_duration = T_GREEN_MIN;
        if (next_state_i == S_EW_GREEN && ns_wait_timer >= STARVATION_LIMIT)
            final_duration = T_GREEN_MIN;

        // -----------------------------------------------------------------
        // Step D: Consecutive-max-bonus limiter
        //
        //   Prevents one direction from hogging T_GREEN_MAX indefinitely
        //   when it has very high density every cycle.
        // -----------------------------------------------------------------
        if (next_state_i == S_NS_GREEN &&
            ns_consec_max >= MAX_CONSEC_BONUS &&
            final_duration == T_GREEN_MAX)
            final_duration = T_GREEN_BASE;

        if (next_state_i == S_EW_GREEN &&
            ew_consec_max >= MAX_CONSEC_BONUS &&
            final_duration == T_GREEN_MAX)
            final_duration = T_GREEN_BASE;

        // -----------------------------------------------------------------
        // Step E: Pedestrian minimum floor
        //
        //   If a pedestrian request is pending for the upcoming green phase,
        //   guarantee at least T_PED_MIN seconds so they can cross safely.
        //   This step can only RAISE final_duration, never lower it.
        //   Yellow phases are unaffected (always fixed).
        // -----------------------------------------------------------------
        if (next_state_i == S_NS_GREEN && ped_ns_req_i) begin
            if (final_duration < T_PED_MIN)
                final_duration = T_PED_MIN;
        end

        if (next_state_i == S_EW_GREEN && ped_ew_req_i) begin
            if (final_duration < T_PED_MIN)
                final_duration = T_PED_MIN;
        end

    end

    // =========================================================================
    // Section 3: Output Assignment with Mode Selection (Gap-Out vs Density)
    // =========================================================================
    always @(*) begin
        // Default outputs
        reload_en_o = 1'b0;

        if (mode_select_i) begin
            // -----------------------------------------------------------------
            // Gap-Out Mode (Loop Sensor based)
            // -----------------------------------------------------------------
            // 1. Reload enable pulse (depends only on register states: current state and timer)
            if (current_state_i == S_NS_GREEN && timer_count_i == 0) begin
                if (sensor_ns_i && active_green_timer < T_GREEN_MAX) begin
                    reload_en_o = 1'b1;
                end
            end else if (current_state_i == S_EW_GREEN && timer_count_i == 0) begin
                if (sensor_ew_i && active_green_timer < T_GREEN_MAX) begin
                    reload_en_o = 1'b1;
                end
            end

            // 2. Duration output (depends on next_state_i for initial loads, and reload_en_o for extensions)
            if (reload_en_o) begin
                duration_o = EXT_TIME;
            end else begin
                case (next_state_i)
                    S_NS_GREEN:  duration_o = ped_ns_req_i ? T_PED_MIN : T_GREEN_MIN;
                    S_EW_GREEN:  duration_o = ped_ew_req_i ? T_PED_MIN : T_GREEN_MIN;
                    S_NS_YELLOW, 
                    S_EW_YELLOW: duration_o = T_YELLOW_FIXED;
                    default:     duration_o = T_GREEN_BASE;
                endcase
            end
        end else begin
            // -----------------------------------------------------------------
            // Camera Density Mode (Original Pre-calculation)
            // -----------------------------------------------------------------
            duration_o  = final_duration;
            reload_en_o = 1'b0;
        end
    end

endmodule