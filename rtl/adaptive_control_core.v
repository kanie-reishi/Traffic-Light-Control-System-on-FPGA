module adaptive_timing_logic (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick_1hz,

    input  wire [4:0]  next_state_i,
    input  wire [4:0]  current_state_i,

    input  wire [3:0]  ns_density_i,
    input  wire [3:0]  ew_density_i,
    input  wire        camera_valid_i,

    // Pedestrian request inputs (from ped_request_handler)
    input  wire        ped_ns_req_i,
    input  wire        ped_ew_req_i,

    output reg  [5:0]  duration_o
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

    // T_PED_MIN: minimum green time when a pedestrian crossing request is
    // active for the phase about to start. Must be >= T_GREEN_MIN.
    // Set to 20 s — enough time for a pedestrian to cross a standard road.
    // Must be <= T_GREEN_MAX (enforced by the clamp in Step B).
    parameter T_PED_MIN        = 6'd20;

    // =========================================================================
    // State Encoding
    // =========================================================================
    parameter S_NS_GREEN  = 5'b00001;
    parameter S_NS_YELLOW = 5'b00010;
    parameter S_EW_GREEN  = 5'b00100;
    parameter S_EW_YELLOW = 5'b01000;
    parameter S_ERROR     = 5'b10000;

    // =========================================================================
    // Internal Registers (unchanged from previous version)
    // =========================================================================
    reg [6:0] ns_wait_timer;
    reg [6:0] ew_wait_timer;
    reg [1:0] ns_consec_max;
    reg [1:0] ew_consec_max;
    reg [4:0] prev_state;

    // =========================================================================
    // 1. Wait Timer and Anti-Latch Counter (Sequential) — unchanged
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

            if (tick_1hz) begin
                if (current_state_i == S_EW_GREEN || current_state_i == S_EW_YELLOW)
                    if (ns_wait_timer < 7'd127) ns_wait_timer <= ns_wait_timer + 1;
                if (current_state_i == S_NS_GREEN || current_state_i == S_NS_YELLOW)
                    if (ew_wait_timer < 7'd127) ew_wait_timer <= ew_wait_timer + 1;
            end

            if (current_state_i == S_NS_GREEN && prev_state != S_NS_GREEN)
                ns_wait_timer <= 0;
            if (current_state_i == S_EW_GREEN && prev_state != S_EW_GREEN)
                ew_wait_timer <= 0;

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
    // 2. Duration Calculation (Combinational)
    //    Steps A–D unchanged. Step E is new (pedestrian floor).
    // =========================================================================
    reg [5:0] raw_duration;
    reg [5:0] clamped_duration;
    reg [5:0] final_duration;

    wire [3:0] ns_den = ns_density_i;
    wire [3:0] ew_den = ew_density_i;

    always @(*) begin

        // ----------------------------------------------------------------
        // Step A: Raw duration from camera density data
        // ----------------------------------------------------------------
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

        // ----------------------------------------------------------------
        // Step B: Hard MIN / MAX safety clamp
        // ----------------------------------------------------------------
        if (next_state_i == S_NS_YELLOW || next_state_i == S_EW_YELLOW) begin
            clamped_duration = T_YELLOW_FIXED;
        end else if (raw_duration < T_GREEN_MIN) begin
            clamped_duration = T_GREEN_MIN;
        end else if (raw_duration > T_GREEN_MAX) begin
            clamped_duration = T_GREEN_MAX;
        end else begin
            clamped_duration = raw_duration;
        end

        // ----------------------------------------------------------------
        // Step C: Starvation override
        // ----------------------------------------------------------------
        final_duration = clamped_duration;

        if (next_state_i == S_NS_GREEN && ew_wait_timer >= STARVATION_LIMIT)
            final_duration = T_GREEN_MIN;
        if (next_state_i == S_EW_GREEN && ns_wait_timer >= STARVATION_LIMIT)
            final_duration = T_GREEN_MIN;

        // ----------------------------------------------------------------
        // Step D: Consecutive-max-bonus limiter
        // ----------------------------------------------------------------
        if (next_state_i == S_NS_GREEN &&
            ns_consec_max >= MAX_CONSEC_BONUS &&
            final_duration == T_GREEN_MAX)
            final_duration = T_GREEN_BASE;

        if (next_state_i == S_EW_GREEN &&
            ew_consec_max >= MAX_CONSEC_BONUS &&
            final_duration == T_GREEN_MAX)
            final_duration = T_GREEN_BASE;

        // ----------------------------------------------------------------
        // Step E: Pedestrian minimum floor  ← NEW
        //
        // If a pedestrian crossing request is active for the direction
        // that is about to receive its green phase, the duration must be
        // at least T_PED_MIN (20 s) regardless of camera density data.
        //
        // This step can only RAISE final_duration, never lower it:
        //   final_duration = max(final_duration, T_PED_MIN)
        //
        // It is applied LAST so that starvation (Step C) and
        // consecutive-max (Step D) take precedence. This means:
        //   - A starved direction still gets T_GREEN_MIN even if there
        //     is a ped request. T_PED_MIN > T_GREEN_MIN so the pedestrian
        //     floor is still respected — but a direction will never be held
        //     at T_GREEN_MAX indefinitely just because a ped button is held.
        //
        // Yellow phases bypass this step entirely (they are always fixed).
        // ----------------------------------------------------------------
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
    // 3. Output Register
    // =========================================================================
    always @(*) begin
        duration_o = final_duration;
    end

endmodule