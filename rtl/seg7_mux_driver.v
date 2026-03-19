// ============================================================================
// seg7_mux_driver.v
// 4-Digit Time-Division Multiplexed 7-Segment Display Driver (Basys 3)
//
// Purpose
// -------
//   Displays two 6-bit binary countdown values (0-63) on four digits of
//   the Basys 3 onboard 7-segment display. Each value is split into
//   tens and ones via simple integer division:
//
//     AN3 : AN2  →  N-S countdown (tens : ones)
//     AN1 : AN0  →  E-W countdown (tens : ones)
//
// How It Works
// ------------
//   Time-division multiplexing rapidly cycles through all four digits,
//   activating one at a time. The human eye perceives all four digits
//   as simultaneously lit. A refresh counter's top 2 bits select which
//   digit is currently active.
//
//   Anti-ghosting: A brief blanking period (BLANK_CYCLES clock cycles)
//   is inserted at the start of each digit's time slot. During blanking,
//   ALL anodes are turned OFF. This prevents "ghosting" — where leftover
//   charge from the previous digit's segments bleeds into the next digit,
//   causing faint phantom characters.
//
// Refresh Rate
// ------------
//   With REFR_BITS = 17 at 100 MHz:
//     Full cycle = 2^17 = 131,072 clocks = 1.31 ms
//     Per-digit  = 131,072 / 4 = 32,768 clocks ≈ 328 μs
//     Refresh    = 100 MHz / 131,072 ≈ 763 Hz (well above flicker threshold)
//
// Parameters
// ----------
//   REFR_BITS    — Width of the refresh counter (default 17 for ~763 Hz).
//                  Reduce for simulation (e.g. 4 for fast testbench cycles).
//   BLANK_CYCLES — Number of clock cycles at the start of each phase where
//                  all anodes are OFF. Auto-scaled: 255 for hardware, 1 for
//                  simulation (when REFR_BITS ≤ 8).
//
// Ports
// -----
//   clk              — System clock (100 MHz)
//   rst_n            — Active-low asynchronous reset
//   ns_countdown_i[5:0] — N-S countdown value (0-63)
//   ew_countdown_i[5:0] — E-W countdown value (0-63)
//   seg_o[6:0]       — Cathode outputs {G,F,E,D,C,B,A} active-low
//   an_o[3:0]        — Anode outputs  {AN3,AN2,AN1,AN0} active-low
// ============================================================================
module seg7_mux_driver #(
    parameter REFR_BITS    = 17,
    parameter BLANK_CYCLES = (REFR_BITS > 8) ? 255 : 1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [5:0]  ns_countdown_i,
    input  wire [5:0]  ew_countdown_i,
    output reg  [6:0]  seg_o,
    output reg  [3:0]  an_o
);

    // =========================================================================
    // Section 1: Binary → BCD Conversion
    //
    //   For values 0-63, simple division and modulo by 10 are fully
    //   synthesisable. Vivado infers a small combinational divider.
    // =========================================================================
    wire [3:0] ns_tens = ns_countdown_i / 10;
    wire [3:0] ns_ones = ns_countdown_i % 10;
    wire [3:0] ew_tens = ew_countdown_i / 10;
    wire [3:0] ew_ones = ew_countdown_i % 10;

    // =========================================================================
    // Section 2: Refresh Counter
    //
    //   Free-running counter. The top 2 bits select the active digit.
    // =========================================================================
    reg [REFR_BITS-1:0] refr_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            refr_counter <= 0;
        else
            refr_counter <= refr_counter + 1;
    end

    wire [1:0] digit_sel = refr_counter[REFR_BITS-1 -: 2];

    // =========================================================================
    // Section 3: Digit Multiplexer
    //
    //   Selects which BCD digit to display based on the current digit_sel.
    // =========================================================================
    reg [3:0] current_bcd;

    always @(*) begin
        case (digit_sel)
            2'b00:   current_bcd = ew_ones;  // AN0 — E-W ones digit
            2'b01:   current_bcd = ew_tens;  // AN1 — E-W tens digit
            2'b10:   current_bcd = ns_ones;  // AN2 — N-S ones digit
            2'b11:   current_bcd = ns_tens;  // AN3 — N-S tens digit
            default: current_bcd = 4'd0;
        endcase
    end

    // =========================================================================
    // Section 4: 7-Segment Decoder Instance
    // =========================================================================
    wire [6:0] seg_decoded;

    seg7_hex_decoder u_decoder (
        .bcd_i(current_bcd),
        .seg_o(seg_decoded)
    );

    // =========================================================================
    // Section 5: Anode Driver with Anti-Ghosting Blanking
    //
    //   During the first BLANK_CYCLES of each digit's time slot, all
    //   anodes are kept OFF (4'b1111). This allows the previous digit's
    //   segment drivers to fully discharge before the new digit activates,
    //   preventing ghosting/bleeding artefacts.
    // =========================================================================
    reg [3:0] an_next;

    always @(*) begin
        if (refr_counter[REFR_BITS-3 : 0] < BLANK_CYCLES) begin
            an_next = 4'b1111;               // Blanking: all digits OFF
        end else begin
            case (digit_sel)
                2'b00:   an_next = 4'b1110;  // AN0 active
                2'b01:   an_next = 4'b1101;  // AN1 active
                2'b10:   an_next = 4'b1011;  // AN2 active
                2'b11:   an_next = 4'b0111;  // AN3 active
                default: an_next = 4'b1111;  // Safety: all OFF
            endcase
        end
    end

    // =========================================================================
    // Section 6: Output Registers
    //
    //   Both cathode (seg_o) and anode (an_o) outputs are registered to
    //   ensure they switch in the same clock cycle, preventing brief
    //   glitches where the anode selects a new digit but the cathodes
    //   still show the old digit's pattern.
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seg_o <= 7'b111_1111;  // All segments OFF on reset
            an_o  <= 4'b1111;      // All digits OFF on reset
        end else begin
            seg_o <= seg_decoded;
            an_o  <= an_next;
        end
    end

endmodule
