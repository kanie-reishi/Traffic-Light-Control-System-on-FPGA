// ============================================================================
// clock_divider.v
// Programmable Clock Divider — generates a single-cycle 1 Hz tick pulse
//
// Purpose
// -------
//   Divides the high-speed system clock (default 100 MHz on Basys 3) down
//   to a 1 Hz tick. The output `tick_out` pulses HIGH for exactly ONE
//   system clock cycle every second. All downstream timing logic
//   (traffic_controller_core, adaptive_timing_logic, ped_request_handler)
//   uses this tick as a time base.
//
// Ports
// -----
//   clk      — High-speed system clock input (e.g. 100 MHz)
//   rst_n    — Active-low asynchronous reset
//   tick_out — 1 Hz single-cycle pulse output (HIGH for 10 ns every 1 s)
//
// Parameters
// ----------
//   FREQ — System clock frequency in Hz. Determines the terminal count
//          of the internal counter. Overridden by parent modules to
//          match the actual board clock (e.g. 100_000_000 for Basys 3).
// ============================================================================
module clock_divider #(
    parameter FREQ = 50_000_000  // System clock frequency (Hz)
)(
    input  wire clk,
    input  wire rst_n,
    output reg  tick_out
);

    // Counter width: needs to hold values up to FREQ-1.
    // log2(100_000_000) ≈ 26.6 → 27 bits is sufficient for up to 134 MHz.
    reg [26:0] counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter  <= 0;
            tick_out <= 0;
        end else begin
            tick_out <= 0;                    // default: tick is LOW
            if (counter == FREQ - 1) begin
                counter  <= 0;               // wrap around
                tick_out <= 1;               // emit single-cycle pulse
            end else begin
                counter  <= counter + 1;
            end
        end
    end

endmodule