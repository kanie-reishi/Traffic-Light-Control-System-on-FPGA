// ============================================================================
// fixed_timing_logic.v
// Combinational duration lookup table for a 4-phase traffic light controller
//
// Maps the current one-hot FSM state from the core module to a fixed countdown
// duration (in seconds), which the core loads into its timer on every state
// transition.
//
// Supported phases:
//   S_NS_GREEN  → T_GREEN  (10 s) — North-South green,  East-West red
//   S_NS_YELLOW → T_YELLOW ( 5 s) — North-South yellow, East-West red
//   S_EW_GREEN  → T_GREEN  (10 s) — East-West green,    North-South red
//   S_EW_YELLOW → T_YELLOW ( 5 s) — East-West yellow,   North-South red
//
// All timing constants are defined as localparams and must be updated here to
// change phase durations — no parameters are exposed to the instantiating core.
//
// Inputs:
//   current_state_i [4:0] — one-hot FSM state from core module
// Outputs:
//   duration_o      [5:0] — countdown duration in seconds (max 63)
// ============================================================================
module fixed_timing_logic (
    input  wire [4:0] current_state_i,  // Current state from core module
    output reg  [5:0] duration_o        // Duration output for core module
);

    // =========================================================================
    // Timing parameters (seconds)
    // =========================================================================
    localparam [5:0] T_GREEN   = 6'd10;
    localparam [5:0] T_YELLOW  = 6'd5;
    localparam [5:0] T_RED     = 6'd10;  // Reserved for future all-red phase
    localparam [5:0] T_DEFAULT = 6'd1;   // Safety fallback

    // =========================================================================
    // One-hot state encoding
    // =========================================================================
    localparam [4:0] S_NS_GREEN  = 5'b0_0001;
    localparam [4:0] S_NS_YELLOW = 5'b0_0010;
    localparam [4:0] S_EW_GREEN  = 5'b0_0100;
    localparam [4:0] S_EW_YELLOW = 5'b0_1000;

    // =========================================================================
    // Duration lookup — purely combinational
    // =========================================================================
    always @(*) begin
        case (current_state_i)
            S_NS_GREEN  : duration_o = T_GREEN;
            S_NS_YELLOW : duration_o = T_YELLOW;
            S_EW_GREEN  : duration_o = T_GREEN;   // was mislabelled "5 seconds"
            S_EW_YELLOW : duration_o = T_YELLOW;
            default     : duration_o = T_DEFAULT;
        endcase
    end

endmodule