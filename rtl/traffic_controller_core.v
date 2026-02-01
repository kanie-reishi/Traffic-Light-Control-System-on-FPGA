// Encoding for traffic lights (3 bit: Red, Yellow, Green)
`define LIGHT_RED    3'b100
`define LIGHT_YELLOW 3'b010
`define LIGHT_GREEN  3'b001
`define LIGHT_OFF    3'b000
// Encoding for state FSM (One-Hot Encoding)
`define S_NS_GREEN   5'b00001
`define S_NS_YELLOW  5'b00010
`define S_EW_GREEN   5'b00100
`define S_EW_YELLOW  5'b01000
`define S_ERROR      5'b10000 // Emergency/Error state
module traffic_controller_core (
    input wire clk,
    input wire rst_n,   // Active low reset
    input wire emergency_mode, // Emergency mode signal
    input wire tick_1hz, // 1 Hz clock tick (for timing control)

    // Interface for expansion modules
    input wire [5:0] req_duration_i, // Time until next state change request
    output wire [4:0] current_state_o, // Current state output for expansion modules
    output wire [4:0] next_state_o, // Next state output for expansion modules

    // Outputs to traffic lights
    output reg [2:0] ns_light_o, // North-South traffic light state
    output reg [2:0] ew_light_o,  // East-West traffic light state
    output reg       ped_ns_walk_o, // Pedestrian walk signal for North-South
    output reg       ped_ew_walk_o // Pedestrian walk signal for East-West
);
    // Use defines for state encoding
    parameter S_NS_GREEN   = 5'b00001;
    parameter S_NS_YELLOW  = 5'b00010;
    parameter S_EW_GREEN   = 5'b00100;
    parameter S_EW_YELLOW  = 5'b01000;
    parameter S_ERROR      = 5'b10000;

    reg [4:0] current_state, next_state;
    reg [5:0] timer; // Timer for state duration (max 63 seconds)
    reg       timer_done;

    reg first_cycle; // First cycle flag for initialization and reset
    // ---------------------------------------------------------
    // 1.   Timer Logic
    // Core timer: counts down from req_duration_i to 0
    // req_duration_i is set by expansion modules
    // ---------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            timer <= 0;
            first_cycle <= 1;
        end else if (current_state == S_ERROR) begin
            timer <= 0;
        end else begin
            if (first_cycle) begin
                timer <= req_duration_i; // Force load on first cycle
                first_cycle <= 0; // Reset first_cycle flag
            end else if (current_state != next_state) begin
            // Load new duration on state change
            timer <= req_duration_i;
            end else if (tick_1hz && timer > 0) begin
            timer <= timer - 1;
            end
        end
    end
    // Timer done signal
    always @(*) begin
        timer_done = (timer == 0) && (!first_cycle);
    end
    // ---------------------------------------------------------
    // 2.   State Transition Logic
    // ---------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            current_state <= S_NS_GREEN; // reset to NS Green
        end else begin
            current_state <= next_state;
        end
    end

    always @(*) begin
        // Default next state is current state
        next_state = current_state;
        // Emergency mode overrides normal operation
        if (emergency_mode) begin
            next_state = S_ERROR;
        end else begin
            case (current_state)
                S_NS_GREEN: begin
                    if (timer_done) begin
                        next_state = S_NS_YELLOW;
                    end
                end
                S_NS_YELLOW: begin
                    if (timer_done) begin
                        next_state = S_EW_GREEN;
                    end
                end
                S_EW_GREEN: begin
                    if (timer_done) begin
                        next_state = S_EW_YELLOW;
                    end
                end
                S_EW_YELLOW: begin
                    if (timer_done) begin
                        next_state = S_NS_GREEN;
                    end
                end
                S_ERROR: begin
                    // Remain in error state until reset
                    next_state = S_ERROR;
                end
                default: begin
                    next_state = S_ERROR; // Fallback to error state
                end
            endcase
        end
    end
    // ---------------------------------------------------------
    // 3.   Output Logic
    // ---------------------------------------------------------
    wire flash_pulse = tick_1hz; // 1 Hz pulse for flashing lights 

    always @(*) begin
        // Default outputs
        ns_light_o = `LIGHT_RED;
        ew_light_o = `LIGHT_RED;
        ped_ns_walk_o = 1'b0; // Default don't walk signal
        ped_ew_walk_o = 1'b0; // Default don't walk signal

        case (current_state)
            S_NS_GREEN: begin
                ns_light_o = `LIGHT_GREEN;
                ew_light_o = `LIGHT_RED;
                ped_ns_walk_o = 1'b1; // Pedestrian walk signal for North-South
                ped_ew_walk_o = 1'b0; // Pedestrian don't walk signal for East-
            end
            S_NS_YELLOW: begin
                ns_light_o = `LIGHT_YELLOW;
                ew_light_o = `LIGHT_RED;
                ped_ns_walk_o = 1'b0; // Stop walk signal for North-South
            end
            S_EW_GREEN: begin
                ns_light_o = `LIGHT_RED;
                ew_light_o = `LIGHT_GREEN;
                ped_ns_walk_o = 1'b0; // Stop walk signal for North-South
                ped_ew_walk_o = 1'b1; // Pedestrian walk signal for East-West
            end
            S_EW_YELLOW: begin
                ns_light_o = `LIGHT_RED;
                ew_light_o = `LIGHT_YELLOW;
                ped_ew_walk_o = 1'b0; // Stop walk signal for East-West
            end
            S_ERROR: begin
                // Flashing Yellow Light
                ns_light_o = (flash_pulse) ? `LIGHT_YELLOW : `LIGHT_OFF;
                ew_light_o = (flash_pulse) ? `LIGHT_YELLOW : `LIGHT_OFF;
                ped_ns_walk_o = 1'b0; // Stop walk signal for North-South
                ped_ew_walk_o = 1'b0; // Stop walk signal for East-West
            end
        endcase
    end
    // Output current state and next state for expansion modules
    assign next_state_o = next_state;
    assign current_state_o = current_state;
endmodule