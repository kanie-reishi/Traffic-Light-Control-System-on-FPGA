// ============================================================================
// uart_camera_tx.v
// Telemetry Packet UART Transmitter (FPGA → PC / Camera AI)
//
// Purpose
// -------
//   Sends real-time status updates back over the UART serial interface (TX).
//   Allows external monitoring applications (e.g. host PC) to track the FPGA's
//   active states, density parameters, countdown timers, and override modes.
//
// Packet Format (5 bytes):
//   Byte 0: 0xBB        (start marker)
//   Byte 1: [7:3]=state  [2]=emergency  [1]=mode_select  [0]=0
//   Byte 2: [5:0]=timer_count
//   Byte 3: [7:4]=NS_density  [3:0]=EW_density
//   Byte 4: 0x55        (end marker)
//
// Settings
// --------
//   Baud Rate: 115200 (default, 8N1)
// ============================================================================

module uart_camera_tx #(
    parameter CLK_FREQ  = 100_000_000, // System clock Hz (default 100 MHz)
    parameter BAUD_RATE = 115_200      // Serial baud rate
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tx_trigger,     // Pulse HIGH to trigger one full 5-byte packet

    // Telemetry data inputs
    input  wire [4:0]  current_state_i,
    input  wire        emergency_sw_i,
    input  wire        mode_select_i,
    input  wire [5:0]  timer_count_i,
    input  wire [3:0]  ns_density_i,
    input  wire [3:0]  ew_density_i,

    output reg         uart_tx_o,      // Serial TX output line
    output reg         tx_busy_o       // HIGH while packet transmission is in progress
);

    // Number of clock cycles per bit
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    // TX state machine encoding
    localparam TX_STATE_IDLE  = 2'd0;
    localparam TX_STATE_START = 2'd1;
    localparam TX_STATE_DATA  = 2'd2;
    localparam TX_STATE_STOP  = 2'd3;

    reg [1:0]  tx_state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;     // Bits 0-7
    reg [2:0]  byte_idx;    // Bytes 0-4
    reg [7:0]  tx_byte;     // Current byte shift register

    // Internal status packet buffer registers
    reg [7:0]  tx_buffer [4:0];
    reg        tx_trigger_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_tx_o    <= 1'b1;       // UART idle line is HIGH
            tx_busy_o    <= 1'b0;
            tx_state     <= TX_STATE_IDLE;
            clk_cnt      <= 0;
            bit_idx      <= 0;
            byte_idx     <= 0;
            tx_byte      <= 0;
            tx_trigger_d <= 1'b0;
        end else begin
            tx_trigger_d <= tx_trigger;

            // Start transmission on rising edge of trigger if transmitter is free
            if (tx_trigger && !tx_trigger_d && !tx_busy_o) begin
                tx_busy_o <= 1'b1;
                byte_idx  <= 0;
                tx_state  <= TX_STATE_IDLE; // Pre-load step

                // Pack state telemetry
                tx_buffer[0] <= 8'hBB;  // Telemetry Start Marker
                tx_buffer[1] <= {current_state_i, emergency_sw_i, mode_select_i, 1'b0};
                tx_buffer[2] <= {2'b00, timer_count_i};
                tx_buffer[3] <= {ns_density_i, ew_density_i};
                tx_buffer[4] <= 8'h55;  // End Marker
            end

            if (tx_busy_o) begin
                case (tx_state)
                    TX_STATE_IDLE: begin
                        // Load next byte from packet buffer
                        tx_byte  <= tx_buffer[byte_idx];
                        tx_state <= TX_STATE_START;
                        clk_cnt  <= 0;
                    end

                    TX_STATE_START: begin
                        uart_tx_o <= 1'b0; // Start bit is always LOW
                        if (clk_cnt == CLKS_PER_BIT - 1) begin
                            clk_cnt  <= 0;
                            tx_state <= TX_STATE_DATA;
                            bit_idx  <= 0;
                        end else begin
                            clk_cnt  <= clk_cnt + 1;
                        end
                    end

                    TX_STATE_DATA: begin
                        uart_tx_o <= tx_byte[bit_idx]; // LSB first
                        if (clk_cnt == CLKS_PER_BIT - 1) begin
                            clk_cnt <= 0;
                            if (bit_idx == 7) begin
                                tx_state <= TX_STATE_STOP;
                            end else begin
                                bit_idx  <= bit_idx + 1;
                            end
                        end else begin
                            clk_cnt  <= clk_cnt + 1;
                        end
                    end

                    TX_STATE_STOP: begin
                        uart_tx_o <= 1'b1; // Stop bit is always HIGH
                        if (clk_cnt == CLKS_PER_BIT - 1) begin
                            clk_cnt <= 0;
                            if (byte_idx == 3'd4) begin
                                // Completed all 5 bytes
                                tx_busy_o <= 1'b0;
                                tx_state  <= TX_STATE_IDLE;
                            end else begin
                                // Advance to transmit the next byte
                                byte_idx <= byte_idx + 1;
                                tx_state <= TX_STATE_IDLE;
                            end
                        end else begin
                            clk_cnt  <= clk_cnt + 1;
                        end
                    end
                endcase
            end else begin
                uart_tx_o <= 1'b1;       // Guarantee line stays idle HIGH
            end
        end
    end

endmodule
