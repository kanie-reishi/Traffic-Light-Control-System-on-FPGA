// =============================================================================
// ped_request_handler.v
// Pedestrian Crossing Request Handler
//
// Responsibilities
// ----------------
//   1. DEBOUNCE  — Filters mechanical button bounce using a counter-based
//                  approach. A press is only registered after the button
//                  has been held continuously for DEBOUNCE_MS milliseconds.
//
//   2. LATCH     — Holds the request HIGH after a valid press, even if the
//                  pedestrian releases the button immediately. A single tap
//                  is enough to register a full crossing request.
//
//   3. CLEAR     — Clears the latch automatically when the corresponding
//                  direction enters its GREEN phase (i.e. the request has
//                  been served). NS request clears on S_NS_GREEN entry.
//                  EW request clears on S_EW_GREEN entry.
//
//   4. REQUEST LED — ped_ns_req_led_o / ped_ew_req_led_o:
//                    HIGH while request is pending (button pressed but not
//                    yet served). Gives physical feedback to pedestrians
//                    that their button press was registered.
//                    Goes LOW when the walk phase begins (request served).
//
// Safety Rules
// ------------
//   - Request latches NEVER directly control the walk signal.
//     Walk signal ownership stays entirely in traffic_controller_core.
//   - In S_ERROR (emergency), all latches are held but not cleared.
//     They will be served on the next normal green phase after recovery.
//   - Button inputs pass through a 2-FF synchroniser before any logic
//     (buttons are mechanical — asynchronous to the FPGA clock).
//
// Port Summary
// ------------
//   clk, rst_n              — System clock and active-low reset
//   tick_1hz                — 1 Hz pulse used by debounce timer
//   current_state_i [4:0]   — From traffic_controller_core.current_state_o
//   ped_btn_ns_i            — Physical NS pedestrian button (active HIGH)
//   ped_btn_ew_i            — Physical EW pedestrian button (active HIGH)
//   ped_ns_req_o            — NS request pending (feeds adaptive_timing_logic)
//   ped_ew_req_o            — EW request pending (feeds adaptive_timing_logic)
//   ped_ns_req_led_o        — NS request indicator LED output
//   ped_ew_req_led_o        — EW request indicator LED output
// =============================================================================

module ped_request_handler #(
    parameter CLK_FREQ     = 50_000_000,  // System clock frequency (Hz)
    parameter DEBOUNCE_MS  = 20           // Button debounce window (ms)
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [4:0] current_state_i,    // From core FSM

    // Physical button inputs (active HIGH, momentary push)
    input  wire       ped_btn_ns_i,
    input  wire       ped_btn_ew_i,

    // Request outputs — latched until served
    output reg        ped_ns_req_o,
    output reg        ped_ew_req_o,

    // Indicator LED outputs — mirrors request state
    output wire       ped_ns_req_led_o,
    output wire       ped_ew_req_led_o
);

    // =========================================================================
    // State Encoding (mirrors traffic_controller_core)
    // =========================================================================
    localparam S_NS_GREEN  = 5'b00001;
    localparam S_NS_YELLOW = 5'b00010;
    localparam S_EW_GREEN  = 5'b00100;
    localparam S_EW_YELLOW = 5'b01000;
    localparam S_ERROR     = 5'b10000;

    // =========================================================================
    // Debounce counter size
    // DEBOUNCE_CYCLES: number of clock cycles button must be held stable.
    // = CLK_FREQ / 1000 * DEBOUNCE_MS = 50_000_000 / 1000 * 20 = 1_000_000
    // =========================================================================
    localparam DEBOUNCE_CYCLES = (CLK_FREQ / 1000) * DEBOUNCE_MS;

    // =========================================================================
    // 1. Two-FF Input Synchronisers
    //    Mechanical buttons are asynchronous. The 2-FF chain reduces
    //    metastability probability to a negligible level before any logic
    //    reads the button state.
    // =========================================================================
    reg ns_sync0, ns_sync1;
    reg ew_sync0, ew_sync1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ns_sync0 <= 1'b0; ns_sync1 <= 1'b0;
            ew_sync0 <= 1'b0; ew_sync1 <= 1'b0;
        end else begin
            ns_sync0 <= ped_btn_ns_i; ns_sync1 <= ns_sync0;
            ew_sync0 <= ped_btn_ew_i; ew_sync1 <= ew_sync0;
        end
    end

    // Stable, synchronised button values — use these everywhere below
    wire ns_btn_stable = ns_sync1;
    wire ew_btn_stable = ew_sync1;

    // =========================================================================
    // 2. Debounce Logic
    //    Pattern: increment a counter while the button is held.
    //    When the counter reaches DEBOUNCE_CYCLES, register a single
    //    "pressed" pulse. Reset the counter immediately if the button
    //    is released before the threshold (bounce or accidental touch).
    //
    //    This produces a single-clock-cycle pulse (ns_pressed / ew_pressed)
    //    that is used to SET the request latch.
    // =========================================================================
    integer ns_debounce_cnt;
    integer ew_debounce_cnt;
    reg     ns_pressed;   // single-cycle pulse: valid press confirmed
    reg     ew_pressed;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ns_debounce_cnt <= 0;
            ew_debounce_cnt <= 0;
            ns_pressed      <= 1'b0;
            ew_pressed      <= 1'b0;
        end else begin
            // Default: no press event this cycle
            ns_pressed <= 1'b0;
            ew_pressed <= 1'b0;

            // --- NS button debounce ---
            if (!ns_btn_stable) begin
                // Button released / bouncing — reset counter
                ns_debounce_cnt <= 0;
            end else if (ns_debounce_cnt < DEBOUNCE_CYCLES) begin
                // Button held but threshold not yet reached
                ns_debounce_cnt <= ns_debounce_cnt + 1;
            end else if (ns_debounce_cnt == DEBOUNCE_CYCLES) begin
                // Threshold just reached — fire single press event
                ns_pressed      <= 1'b1;
                ns_debounce_cnt <= ns_debounce_cnt + 1; // go to CYCLES+1, won't re-fire
            end
            // ns_debounce_cnt > DEBOUNCE_CYCLES: button held, already registered
            // Will reset when button is released (ns_btn_stable goes LOW)

            // --- EW button debounce ---
            if (!ew_btn_stable) begin
                ew_debounce_cnt <= 0;
            end else if (ew_debounce_cnt < DEBOUNCE_CYCLES) begin
                ew_debounce_cnt <= ew_debounce_cnt + 1;
            end else if (ew_debounce_cnt == DEBOUNCE_CYCLES) begin
                ew_pressed      <= 1'b1;
                ew_debounce_cnt <= ew_debounce_cnt + 1;
            end
        end
    end

    // =========================================================================
    // 3. Request Latch Logic
    //
    //    SET   — on ns_pressed / ew_pressed (debounce confirmed press)
    //    CLEAR — when the corresponding direction enters its GREEN phase
    //            (detected as a rising edge on current_state == S_NS/EW_GREEN)
    //
    //    Priority: CLEAR > SET. If by some race the press and the green
    //    entry happen in the same cycle, the request is cleared (safe default:
    //    the pedestrian will see the walk signal start immediately).
    //
    //    In S_ERROR: neither SET nor CLEAR is blocked. The pedestrian can
    //    still register a request during emergency, and it will be served
    //    when normal operation resumes.
    // =========================================================================
    reg prev_state;   // used only for edge detection on state transitions
    reg [4:0] prev_state_full;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ped_ns_req_o    <= 1'b0;
            ped_ew_req_o    <= 1'b0;
            prev_state_full <= S_NS_GREEN;
        end else begin
            prev_state_full <= current_state_i;

            // --- NS request latch ---
            // Clear takes priority: entering NS_GREEN serves the request
            if (current_state_i == S_NS_GREEN && prev_state_full != S_NS_GREEN) begin
                ped_ns_req_o <= 1'b0;   // served — clear latch
            end else if (ns_pressed) begin
                ped_ns_req_o <= 1'b1;   // new request — set latch
            end

            // --- EW request latch ---
            if (current_state_i == S_EW_GREEN && prev_state_full != S_EW_GREEN) begin
                ped_ew_req_o <= 1'b0;   // served — clear latch
            end else if (ew_pressed) begin
                ped_ew_req_o <= 1'b1;
            end
        end
    end

    // =========================================================================
    // 4. Indicator LED Outputs
    //    Simply mirrors the request latch.
    //    HIGH = request pending (button press registered, waiting for green)
    //    LOW  = no pending request (or request just served)
    //
    //    Note: The LED goes LOW as soon as the GREEN phase starts — before
    //    the walk signal turns OFF at the end of the green phase. This is
    //    intentional: it tells the pedestrian "your request was received and
    //    the crossing is now active" rather than blinking during the walk.
    // =========================================================================
    assign ped_ns_req_led_o = ped_ns_req_o;
    assign ped_ew_req_led_o = ped_ew_req_o;

endmodule