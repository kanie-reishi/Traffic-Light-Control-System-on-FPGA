// ============================================================================
// traffic_controller_core.v
// Core Finite State Machine — Traffic Light Controller
//
// Purpose
// -------
//   Implements the central Moore FSM that sequences through the four
//   normal traffic phases (NS_GREEN → NS_YELLOW → EW_GREEN → EW_YELLOW)
//   and one emergency override state (S_ERROR). The module is responsible
//   for:
//     1. Maintaining a countdown timer loaded from req_duration_i
//     2. Advancing to the next state when the timer expires
//     3. Driving the traffic light LED outputs based on current state
//     4. Driving the pedestrian walk/don't-walk signals
//     5. Flashing yellow lights at 0.5 Hz during emergency mode
//
// State Encoding (One-Hot)
// ------------------------
//   S_NS_GREEN  = 5'b00001  — North-South direction has green
//   S_NS_YELLOW = 5'b00010  — North-South direction has yellow (clearance)
//   S_EW_GREEN  = 5'b00100  — East-West direction has green
//   S_EW_YELLOW = 5'b01000  — East-West direction has yellow (clearance)
//   S_ERROR     = 5'b10000  — Emergency/hazard: all-red + flashing yellow
//
// Light Encoding (3-bit)
// ----------------------
//   LIGHT_RED    = 3'b100
//   LIGHT_YELLOW = 3'b010
//   LIGHT_GREEN  = 3'b001
//   LIGHT_OFF    = 3'b000
//
// Timer Behaviour
// ---------------
//   - On RESET or first cycle: timer loads req_duration_i immediately
//   - On STATE CHANGE: timer reloads req_duration_i for the new phase
//   - On each tick_1hz: timer decrements by 1
//   - When timer reaches 0: timer_done asserts, triggering next_state logic
//
// Ports
// -----
//   clk, rst_n          — System clock and active-low async reset
//   emergency_mode      — External switch: HIGH forces S_ERROR instantly
//   tick_1hz            — 1 Hz single-cycle pulse from clock_divider
//   req_duration_i[5:0] — Phase duration (seconds) from adaptive_timing_logic
//   current_state_o[4:0]— Current FSM state (feeds adaptive_timing_logic, etc.)
//   next_state_o[4:0]   — Next FSM state (look-ahead for timing calculation)
//   ns_light_o[2:0]     — North-South traffic light {R, Y, G}
//   ew_light_o[2:0]     — East-West traffic light {R, Y, G}
//   ped_ns_walk_o       — Pedestrian walk signal for N-S crossing
//   ped_ew_walk_o       — Pedestrian walk signal for E-W crossing
//   timer_count_o[5:0]  — Live countdown value (feeds 7-segment display)
// ============================================================================

// Light encoding definitions (active-one-bit)
`define LIGHT_RED    3'b100
`define LIGHT_YELLOW 3'b010
`define LIGHT_GREEN  3'b001
`define LIGHT_OFF    3'b000

// State encoding definitions (one-hot)
`define S_NS_GREEN   5'b00001
`define S_NS_YELLOW  5'b00010
`define S_EW_GREEN   5'b00100
`define S_EW_YELLOW  5'b01000
`define S_ERROR      5'b10000

module traffic_controller_core (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       emergency_mode,
    input  wire       tick_1hz,

    input  wire       reload_en,

    // Duration interface (from adaptive_timing_logic)
    input  wire [5:0] req_duration_i,

    // State interface (to adaptive_timing_logic, ped_request_handler, etc.)
    output wire [4:0] current_state_o,
    output wire [4:0] next_state_o,

    // Traffic light outputs
    output reg  [2:0] ns_light_o,
    output reg  [2:0] ew_light_o,

    // Pedestrian walk signals
    output reg        ped_ns_walk_o,
    output reg        ped_ew_walk_o,

    // Live countdown timer (to 7-segment display via traffic_system_top)
    output wire [5:0] timer_count_o
);

    // =========================================================================
    // Local state parameters (mirrors `define for use in case statements)
    // =========================================================================
    parameter S_NS_GREEN  = 5'b00001;
    parameter S_NS_YELLOW = 5'b00010;
    parameter S_EW_GREEN  = 5'b00100;
    parameter S_EW_YELLOW = 5'b01000;
    parameter S_ERROR     = 5'b10000;

    // =========================================================================
    // Internal registers
    // =========================================================================
    reg [4:0] current_state;  // Current FSM state
    reg [4:0] next_state;     // Combinational next-state output
    reg [5:0] timer;          // Countdown timer (max 63 seconds)
    reg       timer_done;     // Asserts when timer reaches 0
    reg       first_cycle;    // Ensures timer loads on the very first cycle

    // =========================================================================
    // Section 1: Countdown Timer Logic
    //
    //   The timer counts down from req_duration_i to 0, decrementing once
    //   per tick_1hz pulse. It reloads when:
    //     a) The FSM transitions to a new state (current_state != next_state)
    //     b) The first cycle after reset (first_cycle flag)
    //     c) The timing logic requests a reload (reload_en is asserted)
    //   In S_ERROR, the timer is held at 0 (no countdown during emergency).
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer       <= 0;
            first_cycle <= 1;
        end else if (current_state == S_ERROR) begin
            timer       <= 0;                   // No countdown in emergency
        end else begin
            if (first_cycle) begin
                timer       <= req_duration_i;  // Force-load on first cycle
                first_cycle <= 0;
            end else if (current_state != next_state || reload_en) begin
                timer       <= req_duration_i;  // Reload on state transition or reload request
            end else if (tick_1hz && timer > 0) begin
                timer       <= timer - 1;       // Normal countdown
            end
        end
    end

    // Timer done: asserts when countdown has reached 0 (and not on first cycle)
    always @(*) begin
        timer_done = (timer == 0) && (!first_cycle);
    end

    // =========================================================================
    // Section 2: State Transition Logic (Combinational)
    //
    //   Standard Moore FSM next-state logic:
    //     NS_GREEN  → NS_YELLOW  (when timer expires)
    //     NS_YELLOW → EW_GREEN   (when timer expires)
    //     EW_GREEN  → EW_YELLOW  (when timer expires)
    //     EW_YELLOW → NS_GREEN   (when timer expires, cycle repeats)
    //
    //   Emergency mode overrides: any state → S_ERROR immediately.
    //   S_ERROR is sticky — remains until emergency_mode is de-asserted,
    //   at which point the FSM returns to the default S_ERROR case which
    //   will exit on next timer expiry.
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            current_state <= S_NS_GREEN;
        else
            current_state <= next_state;
    end

    always @(*) begin
        next_state = current_state;           // Default: hold current state

        if (emergency_mode) begin
            next_state = S_ERROR;             // Override: emergency takes priority
        end else begin
            case (current_state)
                S_NS_GREEN:  if (timer_done && !reload_en) next_state = S_NS_YELLOW;
                S_NS_YELLOW: if (timer_done) next_state = S_EW_GREEN;
                S_EW_GREEN:  if (timer_done && !reload_en) next_state = S_EW_YELLOW;
                S_EW_YELLOW: if (timer_done) next_state = S_NS_GREEN;
                S_ERROR:     next_state = S_ERROR;  // Stays until reset
                default:     next_state = S_ERROR;  // Unknown → safe fallback
            endcase
        end
    end

    // =========================================================================
    // Section 3: Emergency Flash Toggle
    //
    //   In S_ERROR, both traffic directions display flashing yellow lights.
    //   This toggle register flips every tick_1hz, producing a clean
    //   0.5 Hz square wave (1 second ON, 1 second OFF = 50% duty cycle).
    // =========================================================================
    reg flash_toggle;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            flash_toggle <= 0;
        else if (tick_1hz)
            flash_toggle <= ~flash_toggle;
    end

    wire flash_pulse = flash_toggle;

    // =========================================================================
    // Section 4: Output Logic (Combinational)
    //
    //   Drives traffic lights and pedestrian walk signals based purely
    //   on the current state (Moore machine — outputs depend only on state).
    //
    //   Pedestrian walk signals:
    //     - NS pedestrians can walk during NS_GREEN
    //     - EW pedestrians can walk during EW_GREEN
    //     - All other states: don't walk (safety)
    // =========================================================================
    always @(*) begin
        // Safe defaults: all red, no walk
        ns_light_o    = `LIGHT_RED;
        ew_light_o    = `LIGHT_RED;
        ped_ns_walk_o = 1'b0;
        ped_ew_walk_o = 1'b0;

        case (current_state)
            S_NS_GREEN: begin
                ns_light_o    = `LIGHT_GREEN;
                ew_light_o    = `LIGHT_RED;
                ped_ns_walk_o = 1'b1;   // N-S pedestrians may cross
            end

            S_NS_YELLOW: begin
                ns_light_o = `LIGHT_YELLOW;
                ew_light_o = `LIGHT_RED;
                // No walk during yellow — clearance interval
            end

            S_EW_GREEN: begin
                ns_light_o    = `LIGHT_RED;
                ew_light_o    = `LIGHT_GREEN;
                ped_ew_walk_o = 1'b1;   // E-W pedestrians may cross
            end

            S_EW_YELLOW: begin
                ns_light_o = `LIGHT_RED;
                ew_light_o = `LIGHT_YELLOW;
                // No walk during yellow — clearance interval
            end

            S_ERROR: begin
                // Flashing yellow in both directions (hazard warning)
                ns_light_o = flash_pulse ? `LIGHT_YELLOW : `LIGHT_OFF;
                ew_light_o = flash_pulse ? `LIGHT_YELLOW : `LIGHT_OFF;
                // No walk — emergency mode, all pedestrians stop
            end
        endcase
    end

    // =========================================================================
    // Section 5: Output Assignments
    // =========================================================================
    assign next_state_o    = next_state;
    assign current_state_o = current_state;
    assign timer_count_o   = timer;

endmodule