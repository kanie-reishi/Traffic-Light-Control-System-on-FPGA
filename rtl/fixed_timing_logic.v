module fixed_timing_logic (
    input wire [4:0] current_state_i, // Current state input from core module
    output reg [5:0] duration_o // Duration output for core module
);
    // Time parameter
    parameter T_GREEN = 6'd10; // 10 seconds
    parameter T_YELLOW = 6'd5; // 5 seconds
    parameter T_RED = 6'd10; // 10 seconds

    // Encoding states
    parameter S_NS_GREEN = 5'b00001;
    parameter S_NS_YELLOW = 5'b00010;
    parameter S_EW_GREEN = 5'b00100;
    parameter S_EW_YELLOW = 5'b01000;

    always @(*) begin
        case (current_state_i)
            S_NS_GREEN: begin
                duration_o = T_GREEN; // Core will count down 10 seconds
            end
            S_NS_YELLOW: begin
                duration_o = T_YELLOW;// Core will count down 5 seconds
            end
            S_EW_GREEN: begin
                duration_o = T_GREEN;// Core will count down 5 seconds
            end
            S_EW_YELLOW: begin
                duration_o = T_YELLOW;// Core will count down 5 seconds
            end
            default: begin
                duration_o = 1; // Default to 1 second for safety
            end
        endcase
    end
endmodule