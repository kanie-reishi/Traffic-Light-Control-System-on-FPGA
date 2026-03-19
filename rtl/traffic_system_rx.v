// ============================================================================
// traffic_system_rx.v
// Top-Level FPGA Wrapper — UART-equipped Traffic Light System
//
// Purpose
// -------
//   This is the TOP MODULE instantiated by Vivado and mapped to physical
//   FPGA I/O pins via the XDC constraints file. It glues the UART camera
//   receiver (uart_camera_rx) to the main traffic system (traffic_system_top).
//
//   Hierarchy:
//     traffic_system_rx  (this module — top-level for synthesis)
//       ├── uart_camera_rx       — Decodes serial density packets from AI camera
//       └── traffic_system_top   — Core traffic system (FSM + timing + display)
//
// Ports
// -----
//   clk          — 100 MHz system clock (Basys 3 W5)
//   rst_n        — Active-low master reset (active = switch DOWN)
//   emergency_sw — Emergency override switch (active HIGH)
//   uart_rx      — Serial RX line from external Camera AI module
//   ns_leds[2:0] — North-South traffic light LEDs {Red, Yellow, Green}
//   ew_leds[2:0] — East-West traffic light LEDs  {Red, Yellow, Green}
//   ped_ns_led   — North-South pedestrian walk indicator
//   ped_ew_led   — East-West pedestrian walk indicator
//   seg_o[6:0]   — 7-segment display cathodes (active-low, countdown timer)
//   an_o[3:0]    — 7-segment display anodes   (active-low, digit select)
// ============================================================================
module traffic_system_rx #(
    parameter CLK_FREQ = 100_000_000  // 100 MHz system clock (Basys 3)
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       emergency_sw,
    input  wire       uart_rx,

    output wire [2:0] ns_leds,
    output wire [2:0] ew_leds,
    output wire       ped_ns_led,
    output wire       ped_ew_led,
    output wire [6:0] seg_o,
    output wire [3:0] an_o
);

    // =========================================================================
    // Internal wires: Camera AI → Traffic System data path
    // =========================================================================
    wire [3:0] ns_density;    // Decoded N-S vehicle density (0-15)
    wire [3:0] ew_density;    // Decoded E-W vehicle density (0-15)
    wire       camera_valid;  // HIGH when camera data is fresh and trustworthy

    // =========================================================================
    // UART Camera Receiver
    //   Continuously listens for 3-byte packets [0xAA, data, 0x55].
    //   If no valid packet arrives within TIMEOUT_MS, camera_valid drops LOW
    //   and the traffic system falls back to default fixed-time green phases.
    // =========================================================================
    uart_camera_rx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (115_200),
        .TIMEOUT_MS(5000)        // 5 s timeout — allows manual PC testing
    ) uart_inst (
        .clk           (clk),
        .rst_n         (rst_n),
        .uart_rx       (uart_rx),
        .ns_density_o  (ns_density),
        .ew_density_o  (ew_density),
        .camera_valid_o(camera_valid)
    );

    // =========================================================================
    // Traffic System Core
    //   Receives density data and drives all physical outputs (LEDs, display).
    // =========================================================================
    traffic_system_top #(
        .CLK_FREQ(CLK_FREQ)
    ) top_inst (
        .clk            (clk),
        .rst_n          (rst_n),
        .emergency_sw   (emergency_sw),
        .ns_density_i   (ns_density),
        .ew_density_i   (ew_density),
        .camera_valid_i (camera_valid),
        .ns_leds        (ns_leds),
        .ew_leds        (ew_leds),
        .ped_ns_led     (ped_ns_led),
        .ped_ew_led     (ped_ew_led),
        .seg_o          (seg_o),
        .an_o           (an_o)
    );

endmodule