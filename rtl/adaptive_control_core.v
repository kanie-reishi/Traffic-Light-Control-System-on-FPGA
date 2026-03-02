// =============================================================================
// adaptive_timing_logic.v
// Adaptive Traffic Light Timing Controller
//
// Replaces fixed_timing_logic. Accepts camera AI density data and computes
// safe, bounded green durations. Includes:
//   - Density-proportional green time calculation
//   - Hard MIN / MAX safety clamps (anti-latch)
//   - Fixed yellow time (non-negotiable safety rule)
//   - Starvation watchdog (anti red-latch for low-density direction)
//   - Camera validity fallback to fixed timing
//   - Consecutive-max-extension limiter
// =============================================================================

// -----------------------------------------------------------------------
// MODULE: adaptive_timing_logic
// Drop-in replacement for fixed_timing_logic in traffic_system_top
// -----------------------------------------------------------------------
module adaptive_timing_logic (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_1hz,           // 1 Hz pulse for watchdog counter

    // --- State interface from traffic_controller_core ---
    input  wire [4:0]  next_state_i,       // Lookahead: state being ENTERED next
    input  wire [4:0]  current_state_i,    // State core is currently IN

    // --- Camera AI inputs ---
    // Density: 0 = empty road, 15 = fully congested
    input  wire [3:0]  ns_density_i,       // North-South vehicle density
    input  wire [3:0]  ew_density_i,       // East-West vehicle density
    input  wire        camera_valid_i,     // 1 = camera data is trustworthy

    // --- Output to traffic_controller_core ---
    output reg  [5:0]  duration_o          // Duration (seconds) for next state
);

    // =========================================================================
    // Timing Parameters
    // =========================================================================
    parameter T_GREEN_MIN      = 6'd8;    // Minimum green (safety floor, seconds)
    parameter T_GREEN_BASE     = 6'd15;   // Green time when traffic is balanced
    parameter T_GREEN_MAX      = 6'd45;   // Maximum green (anti-latch ceiling)
    parameter T_YELLOW_FIXED   = 6'd5;    // Yellow is ALWAYS fixed — not adaptive
    parameter T_GREEN_STEP     = 6'd2;    // Extra seconds per density-unit of advantage
    parameter MAX_BONUS        = 6'd30;   // Cap on density bonus before clamping

    // Anti-starvation: if the waiting direction has been red for this many
    // seconds, the serving direction's green is clamped to T_GREEN_MIN.
    parameter STARVATION_LIMIT = 7'd50;   // seconds (needs 7 bits: up to 127)

    // Anti-latch: max consecutive cycles where T_GREEN_MAX is awarded to the
    // same direction before it is forced down to T_GREEN_BASE.
    parameter MAX_CONSEC_BONUS = 2'd2;    // 0,1,2 = up to 3 maximum-bonus cycles

    // =========================================================================
    // State Encoding (mirrors traffic_controller_core)
    // =========================================================================
    parameter S_NS_GREEN  = 5'b00001;
    parameter S_NS_YELLOW = 5'b00010;
    parameter S_EW_GREEN  = 5'b00100;
    parameter S_EW_YELLOW = 5'b01000;
    parameter S_ERROR     = 5'b10000;

    // =========================================================================
    // Internal Registers
    // =========================================================================

    // Starvation watchdog: counts real seconds each direction has been waiting.
    // Resets when that direction's green phase begins.
    reg [6:0] ns_wait_timer;   // Seconds NS has been waiting (red/yellow)
    reg [6:0] ew_wait_timer;   // Seconds EW has been waiting (red/yellow)

    // Consecutive maximum-bonus limiter
    reg [1:0] ns_consec_max;   // Consecutive T_GREEN_MAX cycles for NS
    reg [1:0] ew_consec_max;   // Consecutive T_GREEN_MAX cycles for EW

    reg [4:0] prev_state;      // One-cycle delayed state for edge detection

    // =========================================================================
    // 1. Wait Timer and Anti-Latch Counter Logic (Sequential)
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ns_wait_timer  <= 0;
            ew_wait_timer  <= 0;
            ns_consec_max  <= 0;
            ew_consec_max  <= 0;
            prev_state     <= S_NS_GREEN;
        end else begin
            prev_state <= current_state_i;

            // --- Starvation watchdog: tick up wait timers ---
            // NS is "waiting" whenever EW is being served (green/yellow)
            // EW is "waiting" whenever NS is being served (green/yellow)
            if (tick_1hz) begin
                // NS is waiting when EW has the green or yellow
                if (current_state_i == S_EW_GREEN || current_state_i == S_EW_YELLOW) begin
                    if (ns_wait_timer < 7'd127) ns_wait_timer <= ns_wait_timer + 1;
                end

                // EW is waiting when NS has the green or yellow
                if (current_state_i == S_NS_GREEN || current_state_i == S_NS_YELLOW) begin
                    if (ew_wait_timer < 7'd127) ew_wait_timer <= ew_wait_timer + 1;
                end
            end

            // Reset a direction's wait timer when it just entered its green phase
            if (current_state_i == S_NS_GREEN && prev_state != S_NS_GREEN)
                ns_wait_timer <= 0;
            if (current_state_i == S_EW_GREEN && prev_state != S_EW_GREEN)
                ew_wait_timer <= 0;

            // --- Consecutive max-bonus tracking ---
            // Increment when the computed duration hits T_GREEN_MAX; reset otherwise.
            // This is updated in the combinational block output; we latch it here
            // one cycle later (safe because duration is loaded at state change).
            if (current_state_i == S_NS_GREEN && prev_state != S_NS_GREEN) begin
                // NS just entered green; did it receive max bonus?
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
    // 2. Duration Calculation (Combinational)
    // =========================================================================

    // Internal wires for clarity
    reg [5:0] raw_duration;      // Density-based, before safety clamping
    reg [5:0] clamped_duration;  // After MIN/MAX clamp
    reg [5:0] final_duration;    // After starvation & consecutive-max override

    // Sanitise camera inputs: clamp to valid 4-bit density range (redundant but
    // defensive — protects against glitches on the input bus).
    wire [3:0] ns_den = ns_density_i;   // 0-15, already 4-bit
    wire [3:0] ew_den = ew_density_i;

    always @(*) begin

        // ----------------------------------------------------------------
        // Step A: Raw duration from density data
        // ----------------------------------------------------------------
        if (!camera_valid_i) begin
            // Camera offline / invalid → fall back to fixed base timing
            raw_duration = T_GREEN_BASE;
        end else begin
            case (next_state_i)
                S_NS_GREEN: begin
                    // Advantage = how much MORE NS traffic than EW
                    if (ns_den > ew_den) begin
                        raw_duration = T_GREEN_BASE
                                       + ((ns_den - ew_den) * T_GREEN_STEP);
                        // Prevent multiplication overflow before clamp
                        if (raw_duration > T_GREEN_BASE + MAX_BONUS)
                            raw_duration = T_GREEN_BASE + MAX_BONUS;
                    end else begin
                        // EW has equal or more traffic — give NS only minimum
                        // time so EW gets served quickly.
                        raw_duration = T_GREEN_MIN;
                    end
                end

                S_EW_GREEN: begin
                    if (ew_den > ns_den) begin
                        raw_duration = T_GREEN_BASE
                                       + ((ew_den - ns_den) * T_GREEN_STEP);
                        if (raw_duration > T_GREEN_BASE + MAX_BONUS)
                            raw_duration = T_GREEN_BASE + MAX_BONUS;
                    end else begin
                        raw_duration = T_GREEN_MIN;
                    end
                end

                // Yellow and error states: duration is always fixed
                S_NS_YELLOW, S_EW_YELLOW: raw_duration = T_YELLOW_FIXED;
                default:                   raw_duration = T_GREEN_BASE;
            endcase
        end

        // ----------------------------------------------------------------
        // Step B: Hard MIN / MAX safety clamp
        //   - MIN prevents intersection from being blocked too briefly
        //   - MAX prevents indefinite green (anti-latch)
        //   - Yellow bypasses clamp — it is already fixed above
        // ----------------------------------------------------------------
        if (next_state_i == S_NS_YELLOW || next_state_i == S_EW_YELLOW) begin
            clamped_duration = T_YELLOW_FIXED;          // Yellow: always fixed
        end else if (raw_duration < T_GREEN_MIN) begin
            clamped_duration = T_GREEN_MIN;
        end else if (raw_duration > T_GREEN_MAX) begin
            clamped_duration = T_GREEN_MAX;
        end else begin
            clamped_duration = raw_duration;
        end

        // ----------------------------------------------------------------
        // Step C: Starvation override
        // If the direction that is ABOUT TO WAIT has already been waiting
        // too long, force the current green phase to T_GREEN_MIN so the
        // other direction is served sooner.
        // ----------------------------------------------------------------
        //
        //   next_state = S_NS_GREEN → EW is about to start waiting
        //                            check ew_wait_timer
        //   next_state = S_EW_GREEN → NS is about to start waiting
        //                            check ns_wait_timer
        //
        final_duration = clamped_duration; // default: keep clamped value

        if (next_state_i == S_NS_GREEN &&
            ew_wait_timer >= STARVATION_LIMIT) begin
            // EW has waited too long; cut NS green to minimum
            final_duration = T_GREEN_MIN;
        end

        if (next_state_i == S_EW_GREEN &&
            ns_wait_timer >= STARVATION_LIMIT) begin
            // NS has waited too long; cut EW green to minimum
            final_duration = T_GREEN_MIN;
        end

        // ----------------------------------------------------------------
        // Step D: Consecutive-max-bonus limiter (anti-latch reinforcement)
        // If a direction has received T_GREEN_MAX for MAX_CONSEC_BONUS
        // cycles in a row, force it back to T_GREEN_BASE this cycle.
        // ----------------------------------------------------------------
        if (next_state_i == S_NS_GREEN &&
            ns_consec_max >= MAX_CONSEC_BONUS &&
            final_duration == T_GREEN_MAX) begin
            final_duration = T_GREEN_BASE;
        end

        if (next_state_i == S_EW_GREEN &&
            ew_consec_max >= MAX_CONSEC_BONUS &&
            final_duration == T_GREEN_MAX) begin
            final_duration = T_GREEN_BASE;
        end

    end // always (combinational)

    // =========================================================================
    // 3. Output Register
    // =========================================================================
    always @(*) begin
        duration_o = final_duration;
    end

endmodule