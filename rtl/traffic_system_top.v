module traffic_system_top (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       emergency_sw,
    input wire [3:0] ns_density_i,
    input wire [3:0] ew_density_i,
    input wire       camera_valid_i,

    output wire [2:0] ns_leds,
    output wire [2:0] ew_leds,
    output wire       ped_ns_led,
    output wire       ped_ew_led
);
    wire tick_1hz;
    clock_divider #(.FREQ(50_000_000)) clk_div_inst (
        .clk(clk), .rst_n(rst_n), .tick_out(tick_1hz)
    );

    wire [4:0] current_state, next_state_lookahead;
    wire [5:0] timer_duration;

    adaptive_timing_logic timing_inst (
        .clk            (clk),   .rst_n          (rst_n),
        .tick_1hz       (tick_1hz),
        .next_state_i   (next_state_lookahead),
        .current_state_i(current_state),
        .ns_density_i   (ns_density_i),
        .ew_density_i   (ew_density_i),
        .camera_valid_i (camera_valid_i),
        .duration_o     (timer_duration)
    );

    traffic_controller_core core_inst (
        .clk            (clk),   .rst_n          (rst_n),
        .emergency_mode (emergency_sw),
        .tick_1hz       (tick_1hz),
        .req_duration_i (timer_duration),
        .current_state_o(current_state),
        .next_state_o   (next_state_lookahead),
        .ns_light_o     (ns_leds),
        .ew_light_o     (ew_leds),
        .ped_ns_walk_o  (ped_ns_led),
        .ped_ew_walk_o  (ped_ew_led)
    );
endmodule