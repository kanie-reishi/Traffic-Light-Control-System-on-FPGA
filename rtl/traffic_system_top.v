module traffic_system_top (
    input wire clk, // System Clock
    input wire rst_n, // Button reset
    input wire emergency_sw, // Switch emergency

    // Output to FPGA LEDs / External Pins
    output wire [2:0] ns_leds,
    output wire [2:0] ew_leds,
    output wire ped_ns_led,
    output wire ped_ew_led
);
    // 1. Clock Divider (Create 1Hz pulse from 50MHz)
    wire tick_1hz;
    clock_divider #(.FREQ(50000000)) clk_div_inst (
        .clk(clk),
        .rst_n(rst_n),
        .tick_out(tick_1hz)
    );
    // Wires connect Timing Logic and Core FSM
    wire [4:0] current_state;
    wire [4:0] next_stage_lookahead;
    wire [5:0] timer_duration;

    // 2. Timing Logic (fixed_timing_logic)
    fixed_timing_logic timing_inst (
        .current_state_i(next_stage_lookahead),
        .duration_o(timer_duration)
    );

    // 3. Core FSM (traffic_controller_core)
    traffic_controller_core core_inst (
        .clk(clk),
        .rst_n(rst_n),
        .emergency_mode(emergency_sw),
        .tick_1hz(tick_1hz),

        // Handshake
        .req_duration_i(timer_duration),
        .current_state_o(current_state),
        .next_state_o(next_stage_lookahead),

        // Outputs
        .ns_light_o(ns_leds),
        .ew_light_o(ew_leds),
        .ped_ns_walk_o(ped_ns_led),
        .ped_ew_walk_o(ped_ew_led)
    );
endmodule