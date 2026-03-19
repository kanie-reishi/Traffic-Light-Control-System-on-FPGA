// ============================================================================
// traffic_system_top.v
// System Integrator — wires together all traffic light sub-modules
//
// Purpose
// -------
//   Instantiates and interconnects every functional block of the traffic
//   light controller. This module does NOT contain any FSM or timing
//   logic itself — it is purely structural (wiring only), plus a small
//   combinational block that routes the live countdown timer to the
//   correct 7-segment display pair based on the current FSM state.
//
// Sub-module Hierarchy
// --------------------
//   traffic_system_top (this module)
//     ├── clock_divider             — Generates a 1 Hz tick from 100 MHz
//     ├── ped_request_handler       — Debounces pedestrian buttons, latches requests
//     ├── adaptive_timing_logic     — Computes green/yellow durations from density
//     ├── traffic_controller_core   — Core FSM: state transitions, countdown, LED outputs
//     └── seg7_mux_driver           — 4-digit multiplexed 7-segment display driver
//
// Display Routing Logic
// ---------------------
//   The FSM produces a single 6-bit countdown timer. This module routes it
//   to the correct display pair:
//     - During NS_GREEN / NS_YELLOW → left  two digits (AN3:AN2) show the countdown
//     - During EW_GREEN / EW_YELLOW → right two digits (AN1:AN0) show the countdown
//     - The idle pair always shows "00"
//
// Ports
// -----
//   clk, rst_n, emergency_sw     — System clock, reset, emergency switch
//   ns_density_i[3:0]            — N-S vehicle density from camera (0-15)
//   ew_density_i[3:0]            — E-W vehicle density from camera (0-15)
//   camera_valid_i               — HIGH = camera density data is fresh
//   ped_btn_ns_i, ped_btn_ew_i   — Physical pedestrian push buttons
//   ns_leds[2:0], ew_leds[2:0]   — Traffic light LED outputs {R, Y, G}
//   ped_ns_led, ped_ew_led       — Pedestrian walk indicator LEDs
//   ped_ns_req_led_o             — Pedestrian NS request pending LED
//   ped_ew_req_led_o             — Pedestrian EW request pending LED
//   timer_duration_o[5:0]        — Current phase duration (debug/expansion)
//   seg_o[6:0], an_o[3:0]        — 7-segment display interface
// ============================================================================
module traffic_system_top #(
    parameter CLK_FREQ = 100_000_000  // 100 MHz clock frequency
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       emergency_sw,

    // Camera AI density inputs (from uart_camera_rx)
    input  wire [3:0] ns_density_i,
    input  wire [3:0] ew_density_i,
    input  wire       camera_valid_i,

    // Pedestrian push buttons (active HIGH, momentary)
    input  wire       ped_btn_ns_i,
    input  wire       ped_btn_ew_i,

    // Vehicle traffic light outputs
    output wire [2:0] ns_leds,       // {Red, Yellow, Green}
    output wire [2:0] ew_leds,       // {Red, Yellow, Green}

    // Pedestrian walk / don't-walk signals (from core FSM)
    output wire       ped_ns_led,
    output wire       ped_ew_led,

    // Pedestrian request-pending indicator LEDs (from ped_request_handler)
    output wire       ped_ns_req_led_o,
    output wire       ped_ew_req_led_o,

    // Debug / expansion port
    output wire [5:0] timer_duration_o,

    // 7-segment display interface (active-low, Basys 3)
    output wire [6:0] seg_o,
    output wire [3:0] an_o
);

    // =========================================================================
    // 1. Clock Divider — 100 MHz → 1 Hz tick
    // =========================================================================
    wire tick_1hz;

    clock_divider #(
        .FREQ(CLK_FREQ)
    ) clk_div_inst (
        .clk     (clk),
        .rst_n   (rst_n),
        .tick_out(tick_1hz)
    );

    // =========================================================================
    // 2. Internal interconnect signals
    // =========================================================================
    wire [4:0] current_state;          // Current FSM state (one-hot encoded)
    wire [4:0] next_state_lookahead;   // Next state the FSM will transition to
    wire [5:0] timer_duration;         // Computed phase duration (from adaptive logic)
    wire [5:0] timer_count;            // Live countdown value (from core FSM)
    wire       ped_ns_req;             // Latched NS pedestrian request
    wire       ped_ew_req;             // Latched EW pedestrian request

    // =========================================================================
    // 3. Pedestrian Request Handler
    //    Debounces physical buttons and latches requests until served.
    // =========================================================================
    ped_request_handler #(
        .CLK_FREQ   (CLK_FREQ),
        .DEBOUNCE_MS(20)
    ) ped_handler_inst (
        .clk             (clk),
        .rst_n           (rst_n),
        .current_state_i (current_state),
        .ped_btn_ns_i    (ped_btn_ns_i),
        .ped_btn_ew_i    (ped_btn_ew_i),
        .ped_ns_req_o    (ped_ns_req),
        .ped_ew_req_o    (ped_ew_req),
        .ped_ns_req_led_o(ped_ns_req_led_o),
        .ped_ew_req_led_o(ped_ew_req_led_o)
    );

    // =========================================================================
    // 4. Adaptive Timing Logic
    //    Takes density data + pedestrian requests and computes optimal
    //    durations for each upcoming green/yellow phase.
    // =========================================================================
    adaptive_timing_logic timing_inst (
        .clk            (clk),
        .rst_n          (rst_n),
        .tick_1hz       (tick_1hz),
        .next_state_i   (next_state_lookahead),
        .current_state_i(current_state),
        .ns_density_i   (ns_density_i),
        .ew_density_i   (ew_density_i),
        .camera_valid_i (camera_valid_i),
        .ped_ns_req_i   (ped_ns_req),
        .ped_ew_req_i   (ped_ew_req),
        .duration_o     (timer_duration)
    );

    // =========================================================================
    // 5. Traffic Controller Core FSM
    //    State machine that drives light transitions, countdown timer,
    //    walk signals, and emergency flashing.
    // =========================================================================
    traffic_controller_core core_inst (
        .clk            (clk),
        .rst_n          (rst_n),
        .emergency_mode (emergency_sw),
        .tick_1hz       (tick_1hz),
        .req_duration_i (timer_duration),
        .current_state_o(current_state),
        .next_state_o   (next_state_lookahead),
        .ns_light_o     (ns_leds),
        .ew_light_o     (ew_leds),
        .ped_ns_walk_o  (ped_ns_led),
        .ped_ew_walk_o  (ped_ew_led),
        .timer_count_o  (timer_count)
    );

    // =========================================================================
    // 6. Display Routing — route countdown to correct digit pair
    //
    //    The core FSM produces one countdown timer. We route it to
    //    the N-S or E-W display pair depending on which direction
    //    currently has an active (green or yellow) phase.
    //
    //    State encoding (one-hot):
    //      S_NS_GREEN  = 5'b00001    S_EW_GREEN  = 5'b00100
    //      S_NS_YELLOW = 5'b00010    S_EW_YELLOW = 5'b01000
    //      S_ERROR     = 5'b10000
    // =========================================================================
    localparam S_NS_GREEN  = 5'b00001;
    localparam S_NS_YELLOW = 5'b00010;
    localparam S_EW_GREEN  = 5'b00100;
    localparam S_EW_YELLOW = 5'b01000;
    localparam S_ERROR     = 5'b10000;

    reg [5:0] ns_countdown;  // Value shown on AN3:AN2
    reg [5:0] ew_countdown;  // Value shown on AN1:AN0

    always @(*) begin
        case (current_state)
            S_NS_GREEN, S_NS_YELLOW: begin
                ns_countdown = timer_count;  // Active N-S phase → show countdown
                ew_countdown = 6'd0;         // E-W idle → show "00"
            end
            S_EW_GREEN, S_EW_YELLOW: begin
                ns_countdown = 6'd0;         // N-S idle → show "00"
                ew_countdown = timer_count;  // Active E-W phase → show countdown
            end
            default: begin                   // S_ERROR or unknown
                ns_countdown = 6'd0;
                ew_countdown = 6'd0;
            end
        endcase
    end

    // =========================================================================
    // 7. 7-Segment Multiplexed Display Driver
    //    Converts two 6-bit binary countdown values into a time-division
    //    multiplexed 4-digit 7-segment display output.
    // =========================================================================
    seg7_mux_driver seg7_inst (
        .clk           (clk),
        .rst_n         (rst_n),
        .ns_countdown_i(ns_countdown),
        .ew_countdown_i(ew_countdown),
        .seg_o         (seg_o),
        .an_o          (an_o)
    );

endmodule