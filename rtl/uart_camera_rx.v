// =============================================================
// uart_camera_rx.v
// Receives density packets from Camera AI over UART
//
// Packet format (3 bytes):
//   Byte 0: 0xAA        (start marker)
//   Byte 1: [7:4]=NS    [3:0]=EW   (density nibbles, 0-15)
//   Byte 2: 0x55        (end marker)
//
// If packet is malformed, last valid data is held (safe default)
// camera_valid_o goes LOW for one full timeout period if no
// packet is received within TIMEOUT_MS milliseconds.
// =============================================================

module uart_camera_rx #(
    parameter CLK_FREQ   = 50_000_000,  // System clock Hz
    parameter BAUD_RATE  = 115200,      // Must match Camera AI setting
    parameter TIMEOUT_MS = 500          // ms before camera_valid goes LOW
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       uart_rx,          // Serial line from Camera AI TX

    output reg  [3:0] ns_density_o,     // North-South density (0-15)
    output reg  [3:0] ew_density_o,     // East-West density (0-15)
    output reg        camera_valid_o    // 1 = data is fresh and trustworthy
);

    // =========================================================
    // Baud rate generator
    // Samples the RX line at 16x the baud rate for robustness.
    // Sampling in the middle of each bit avoids edge noise.
    // =========================================================
    localparam OVERSAMPLE    = 16;
    localparam CLKS_PER_TICK = CLK_FREQ / (BAUD_RATE * OVERSAMPLE);
    localparam TIMEOUT_TICKS = (CLK_FREQ / 1000) * TIMEOUT_MS;

    reg [$clog2(CLKS_PER_TICK)-1:0] baud_cnt;
    reg                              baud_tick;  // pulses at 16x baud

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt  <= 0;
            baud_tick <= 0;
        end else begin
            baud_tick <= 0;
            if (baud_cnt == CLKS_PER_TICK - 1) begin
                baud_cnt  <= 0;
                baud_tick <= 1;
            end else begin
                baud_cnt <= baud_cnt + 1;
            end
        end
    end

    // =========================================================
    // RX input synchroniser (2-FF) — prevents metastability
    // The UART line is asynchronous to our clock. Feeding it
    // directly into logic risks metastability. Two flip-flops
    // in series reduce the probability to negligible levels.
    // =========================================================
    reg rx_sync0, rx_sync1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rx_sync0 <= 1; rx_sync1 <= 1; end
        else        begin rx_sync0 <= uart_rx; rx_sync1 <= rx_sync0; end
    end
    wire rx = rx_sync1;  // use this everywhere below

    // =========================================================
    // UART RX state machine (1 start bit + 8 data bits + 1 stop)
    // =========================================================
    localparam RX_IDLE  = 2'd0;
    localparam RX_START = 2'd1;
    localparam RX_DATA  = 2'd2;
    localparam RX_STOP  = 2'd3;

    reg [1:0]  rx_state;
    reg [3:0]  tick_cnt;   // counts 16 ticks per bit (0-15)
    reg [2:0]  bit_idx;    // which data bit we are receiving (0-7)
    reg [7:0]  shift_reg;  // incoming byte shift register
    reg        byte_ready; // pulses for 1 clock when a byte is complete
    reg [7:0]  rx_byte;    // latched complete byte

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state   <= RX_IDLE;
            tick_cnt   <= 0;
            bit_idx    <= 0;
            shift_reg  <= 0;
            byte_ready <= 0;
            rx_byte    <= 0;
        end else begin
            byte_ready <= 0; // default: no byte this cycle

            case (rx_state)
                RX_IDLE: begin
                    // Wait for start bit (line goes LOW from idle HIGH)
                    if (!rx) begin
                        rx_state <= RX_START;
                        tick_cnt <= 0;
                    end
                end

                RX_START: begin
                    // Wait until middle of start bit (tick 7 of 16)
                    // to confirm it's a real start, not a glitch
                    if (baud_tick) begin
                        if (tick_cnt == 7) begin
                            if (!rx) begin
                                // Valid start bit confirmed
                                rx_state <= RX_DATA;
                                tick_cnt <= 0;
                                bit_idx  <= 0;
                            end else begin
                                // Glitch — back to idle
                                rx_state <= RX_IDLE;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end
                    end
                end

                RX_DATA: begin
                    if (baud_tick) begin
                        if (tick_cnt == 15) begin
                            // Sample at tick 15 = centre of data bit
                            tick_cnt             <= 0;
                            shift_reg[bit_idx]   <= rx;  // LSB first
                            if (bit_idx == 7) begin
                                rx_state <= RX_STOP;
                            end else begin
                                bit_idx <= bit_idx + 1;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end
                    end
                end

                RX_STOP: begin
                    if (baud_tick) begin
                        if (tick_cnt == 15) begin
                            rx_state <= RX_IDLE;
                            if (rx) begin
                                // Valid stop bit — publish the byte
                                rx_byte    <= shift_reg;
                                byte_ready <= 1;
                            end
                            // If stop bit is wrong (framing error), discard silently
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end
                    end
                end
            endcase
        end
    end

    // =========================================================
    // Packet assembler
    // Expects: 0xAA | data_byte | 0x55
    // Rejects malformed packets — holds last valid values
    // =========================================================
    localparam PKT_WAIT_START = 2'd0;
    localparam PKT_WAIT_DATA  = 2'd1;
    localparam PKT_WAIT_END   = 2'd2;

    reg [1:0] pkt_state;
    reg [7:0] data_latch;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pkt_state    <= PKT_WAIT_START;
            data_latch   <= 0;
            ns_density_o <= 4'd7;  // safe default: balanced
            ew_density_o <= 4'd7;
        end else if (byte_ready) begin
            case (pkt_state)
                PKT_WAIT_START: begin
                    if (rx_byte == 8'hAA)
                        pkt_state <= PKT_WAIT_DATA;
                    // else: ignore — re-sync to next 0xAA
                end

                PKT_WAIT_DATA: begin
                    data_latch <= rx_byte;
                    pkt_state  <= PKT_WAIT_END;
                end

                PKT_WAIT_END: begin
                    if (rx_byte == 8'h55) begin
                        // Valid packet — update outputs
                        ns_density_o <= data_latch[7:4];
                        ew_density_o <= data_latch[3:0];
                    end
                    // Whether valid or not, go back to hunting for next packet
                    pkt_state <= PKT_WAIT_START;
                end
            endcase
        end
    end

    // =========================================================
    // Timeout watchdog — camera_valid_o
    // Counts clocks since last valid packet. If no packet
    // arrives within TIMEOUT_MS, pulls camera_valid LOW so
    // adaptive_timing_logic falls back to fixed timing.
    // =========================================================
    reg [$clog2(TIMEOUT_TICKS)-1:0] timeout_cnt;
    reg                              valid_packet_received;

    // valid_packet_received pulses when a complete packet commits
    always @(*) begin
        valid_packet_received = (byte_ready && rx_byte == 8'h55 &&
                                 pkt_state == PKT_WAIT_END);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timeout_cnt    <= 0;
            camera_valid_o <= 0;  // invalid until first packet arrives
        end else begin
            if (valid_packet_received) begin
                timeout_cnt    <= 0;
                camera_valid_o <= 1;
            end else if (timeout_cnt < TIMEOUT_TICKS) begin
                timeout_cnt <= timeout_cnt + 1;
            end else begin
                camera_valid_o <= 0;  // camera silent too long → fallback
            end
        end
    end
endmodule