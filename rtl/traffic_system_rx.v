module traffic_system_rx #(
    parameter CLK_FREQ = 100_000_000 // 100MHz clock frequency
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       emergency_sw,
    input  wire       uart_rx,        // ← NEW: wire from Camera AI TX pin

    output wire [2:0] ns_leds,
    output wire [2:0] ew_leds,
    output wire       ped_ns_led,
    output wire       ped_ew_led
);
    wire [3:0] ns_density, ew_density;
    wire       camera_valid;
    // NEW: UART receiver — decodes Camera AI packets
    uart_camera_rx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (115_200),
        .TIMEOUT_MS(500)
    ) uart_inst (
        .clk           (clk),
        .rst_n         (rst_n),
        .uart_rx       (uart_rx),
        .ns_density_o  (ns_density),
        .ew_density_o  (ew_density),
        .camera_valid_o(camera_valid)
    );
    traffic_system_top #(
        .CLK_FREQ(CLK_FREQ)
    )top_inst (
        .clk(clk), .rst_n(rst_n), .emergency_sw(emergency_sw),
        .ns_density_i(ns_density), .ew_density_i(ew_density), .camera_valid_i(camera_valid),
        .ns_leds(ns_leds), .ew_leds(ew_leds),
        .ped_ns_led(ped_ns_led), .ped_ew_led(ped_ew_led)
    );
endmodule