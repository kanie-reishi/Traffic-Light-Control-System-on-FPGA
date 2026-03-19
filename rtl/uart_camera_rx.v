// ============================================================================
// uart_camera_rx.v
// UART Receiver — decodes density packets from an external Camera AI module
//
// Purpose
// -------
//   Receives asynchronous serial data from a Camera AI module over a
//   standard UART link (115200 baud, 8N1). Extracts vehicle density
//   values for both traffic directions and provides a validity flag
//   for downstream adaptive timing logic.
//
// Packet Protocol
// ---------------
//   3-byte packet format:
//     Byte 0: 0xAA        — Start marker
//     Byte 1: [7:4] = NS density (0-15), [3:0] = EW density (0-15)
//     Byte 2: 0x55        — End marker
//
//   If a packet is malformed (wrong markers), it is silently discarded
//   and the last valid density values are retained (safe default).
//
// Timeout Watchdog
// ----------------
//   A hardware watchdog counts system clock cycles since the last valid
//   packet. If TIMEOUT_MS elapses without a new valid packet,
//   camera_valid_o drops LOW, signalling that the density data is stale
//   and the traffic system should fall back to fixed-time phases.
//
// UART Implementation Details
// ---------------------------
//   - 16x oversampling for robust bit sampling in the presence of noise
//   - Samples at the centre of each data bit (tick 7 for start, 15 for data)
//   - 2-stage flip-flop synchroniser on the RX input (prevents metastability)
//   - Start bit validation: if the line is HIGH at the mid-start-bit sample
//     point, the frame is rejected as a glitch
//
// Ports
// -----
//   clk              — System clock
//   rst_n            — Active-low asynchronous reset
//   uart_rx          — Raw serial RX line from Camera AI
//   ns_density_o[3:0]— Decoded N-S density (0-15), held until next valid packet
//   ew_density_o[3:0]— Decoded E-W density (0-15), held until next valid packet
//   camera_valid_o   — HIGH = density data is fresh and trustworthy
//
// Parameters
// ----------
//   CLK_FREQ   — System clock frequency in Hz (must match board)
//   BAUD_RATE  — UART baud rate (must match Camera AI setting)
//   TIMEOUT_MS — Milliseconds of silence before camera_valid drops LOW
// ============================================================================
module uart_camera_rx #(
    parameter CLK_FREQ   = 50_000_000,
    parameter BAUD_RATE  = 115200,
    parameter TIMEOUT_MS = 500
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       uart_rx,

    output reg  [3:0] ns_density_o,
    output reg  [3:0] ew_density_o,
    output reg        camera_valid_o
);

    // =========================================================================
    // Section 1: Baud Rate Generator
    //
    //   Generates a "baud tick" at 16x the baud rate. This oversampling
    //   allows the receiver to find the centre of each bit reliably.
    //   At 115200 baud with 16x oversampling:
    //     tick_rate = 115200 × 16 = 1,843,200 Hz
    //     CLKS_PER_TICK = CLK_FREQ / tick_rate
    // =========================================================================
    localparam OVERSAMPLE    = 16;
    localparam CLKS_PER_TICK = CLK_FREQ / (BAUD_RATE * OVERSAMPLE);
    localparam TIMEOUT_TICKS = (CLK_FREQ / 1000) * TIMEOUT_MS;

    reg [$clog2(CLKS_PER_TICK)-1:0] baud_cnt;
    reg                              baud_tick;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt  <= 0;
            baud_tick <= 0;
        end else begin
            baud_tick <= 0;                          // default: no tick
            if (baud_cnt == CLKS_PER_TICK - 1) begin
                baud_cnt  <= 0;
                baud_tick <= 1;                      // emit tick
            end else begin
                baud_cnt <= baud_cnt + 1;
            end
        end
    end

    // =========================================================================
    // Section 2: RX Input Synchroniser (2-FF chain)
    //
    //   The UART RX line is asynchronous to our system clock. Two flip-flops
    //   in series reduce metastability probability to negligible levels.
    // =========================================================================
    reg rx_sync0, rx_sync1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync0 <= 1;   // Idle state is HIGH
            rx_sync1 <= 1;
        end else begin
            rx_sync0 <= uart_rx;
            rx_sync1 <= rx_sync0;
        end
    end

    wire rx = rx_sync1;  // Synchronised RX — use this everywhere below

    // =========================================================================
    // Section 3: UART Byte Receiver State Machine
    //
    //   Standard 8N1 UART receiver with 16x oversampling:
    //     RX_IDLE  → Wait for start bit (line goes LOW)
    //     RX_START → Validate start bit at centre (tick 7)
    //     RX_DATA  → Sample 8 data bits at centre (tick 15), LSB first
    //     RX_STOP  → Validate stop bit at centre (tick 15)
    //
    //   On valid stop bit: latches the byte into rx_byte and pulses
    //   byte_ready for 1 clock cycle. On framing error: discards silently.
    // =========================================================================
    localparam RX_IDLE  = 2'd0;
    localparam RX_START = 2'd1;
    localparam RX_DATA  = 2'd2;
    localparam RX_STOP  = 2'd3;

    reg [1:0]  rx_state;
    reg [3:0]  tick_cnt;    // Counts 16 ticks per bit (0-15)
    reg [2:0]  bit_idx;     // Which data bit we are receiving (0-7)
    reg [7:0]  shift_reg;   // Incoming byte shift register
    reg        byte_ready;  // Pulses HIGH for 1 clock when a byte is complete
    reg [7:0]  rx_byte;     // Latched complete byte

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state   <= RX_IDLE;
            tick_cnt   <= 0;
            bit_idx    <= 0;
            shift_reg  <= 0;
            byte_ready <= 0;
            rx_byte    <= 0;
        end else begin
            byte_ready <= 0;  // Default: no byte this cycle

            case (rx_state)
                // ---------------------------------------------------------
                // IDLE: Wait for the start bit (line drops from HIGH to LOW)
                // ---------------------------------------------------------
                RX_IDLE: begin
                    if (!rx) begin
                        rx_state <= RX_START;
                        tick_cnt <= 0;
                    end
                end

                // ---------------------------------------------------------
                // START: Wait until the centre of the start bit (tick 7).
                //        Confirm the line is still LOW — if not, it was
                //        a glitch and we return to IDLE.
                // ---------------------------------------------------------
                RX_START: begin
                    if (baud_tick) begin
                        if (tick_cnt == 7) begin
                            if (!rx) begin
                                rx_state <= RX_DATA;  // Valid start confirmed
                                tick_cnt <= 0;
                                bit_idx  <= 0;
                            end else begin
                                rx_state <= RX_IDLE;  // Glitch — abort
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end
                    end
                end

                // ---------------------------------------------------------
                // DATA: Sample each of the 8 data bits at tick 15 (centre).
                //       Bits arrive LSB first and are shifted into shift_reg.
                // ---------------------------------------------------------
                RX_DATA: begin
                    if (baud_tick) begin
                        if (tick_cnt == 15) begin
                            tick_cnt  <= 0;
                            shift_reg <= {rx, shift_reg[7:1]};  // LSB first
                            if (bit_idx == 7)
                                rx_state <= RX_STOP;
                            else
                                bit_idx <= bit_idx + 1;
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end
                    end
                end

                // ---------------------------------------------------------
                // STOP: Validate the stop bit (must be HIGH).
                //       If valid: latch byte and pulse byte_ready.
                //       If invalid: framing error — discard silently.
                // ---------------------------------------------------------
                RX_STOP: begin
                    if (baud_tick) begin
                        if (tick_cnt == 15) begin
                            rx_state <= RX_IDLE;
                            if (rx) begin
                                rx_byte    <= shift_reg;
                                byte_ready <= 1;
                            end
                            // Framing error (stop bit wrong) → silently discard
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // Section 4: Packet Assembler
    //
    //   Assembles individual bytes into complete 3-byte packets:
    //     [0xAA] → [data_byte] → [0x55]
    //
    //   On valid packet: updates ns_density_o and ew_density_o.
    //   On malformed packet: discards and resets to hunt for next 0xAA.
    //   Last valid data is always preserved (safe hold behaviour).
    // =========================================================================
    localparam PKT_WAIT_START = 2'd0;
    localparam PKT_WAIT_DATA  = 2'd1;
    localparam PKT_WAIT_END   = 2'd2;

    reg [1:0] pkt_state;
    reg [7:0] data_latch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pkt_state    <= PKT_WAIT_START;
            data_latch   <= 0;
            ns_density_o <= 4'd7;  // Safe default: balanced density
            ew_density_o <= 4'd7;
        end else if (byte_ready) begin
            case (pkt_state)
                PKT_WAIT_START: begin
                    if (rx_byte == 8'hAA)
                        pkt_state <= PKT_WAIT_DATA;
                    // Else: ignore — re-sync to next 0xAA
                end

                PKT_WAIT_DATA: begin
                    data_latch <= rx_byte;
                    pkt_state  <= PKT_WAIT_END;
                end

                PKT_WAIT_END: begin
                    if (rx_byte == 8'h55) begin
                        // Valid packet — extract density nibbles
                        ns_density_o <= data_latch[7:4];
                        ew_density_o <= data_latch[3:0];
                    end
                    // Whether valid or not, return to hunting for next packet
                    pkt_state <= PKT_WAIT_START;
                end
            endcase
        end
    end

    // =========================================================================
    // Section 5: Timeout Watchdog
    //
    //   Counts system clock cycles since the last valid packet was received.
    //   If TIMEOUT_MS elapses without a new valid packet, camera_valid_o
    //   drops LOW, telling adaptive_timing_logic to use fixed-time defaults.
    //   Resets the counter and reasserts camera_valid_o on each valid packet.
    // =========================================================================
    reg [$clog2(TIMEOUT_TICKS)-1:0] timeout_cnt;

    // Detect the exact cycle a valid packet completes
    wire valid_packet_received = (byte_ready && rx_byte == 8'h55 &&
                                  pkt_state == PKT_WAIT_END);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timeout_cnt    <= 0;
            camera_valid_o <= 0;  // Invalid until first packet arrives
        end else begin
            if (valid_packet_received) begin
                timeout_cnt    <= 0;
                camera_valid_o <= 1;
            end else if (timeout_cnt < TIMEOUT_TICKS) begin
                timeout_cnt <= timeout_cnt + 1;
            end else begin
                camera_valid_o <= 0;  // Camera silent too long → fallback
            end
        end
    end

endmodule