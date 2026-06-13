`timescale 1ns/1ps

module tb_traffic_system_rx_tx;

    // =========================================================================
    // Simulation Parameters
    // We use a 10 MHz clock frequency for simulation so UART rates are reasonable.
    // =========================================================================
    localparam CLK_FREQ_TB   = 100_000_000;
    localparam BAUD_RATE_TB  = 115_200;
    localparam CLK_PERIOD    = 10; // 10 ns -> 100 MHz
    
    // UART bit duration: 100M / 115200 = 868.05 cycles -> ~868 cycles
    localparam BIT_CYCLES    = CLK_FREQ_TB / BAUD_RATE_TB; // 868
    
    // =========================================================================
    // Testbench Signals
    // =========================================================================
    reg        clk;
    reg        rst_n;
    reg        emergency_sw;
    reg        uart_rx;
    reg        ped_btn_ns_i;
    reg        ped_btn_ew_i;
    reg        sensor_ns_i;
    reg        sensor_ew_i;
    reg        mode_select_i;
    
    wire [2:0] ns_leds;
    wire [2:0] ew_leds;
    wire       ped_ns_led;
    wire       ped_ew_led;
    wire       ped_ns_req_led_o;
    wire       ped_ew_req_led_o;
    wire       buzzer_o;
    wire       uart_tx;
    wire [6:0] seg_o;
    wire [3:0] an_o;

    // =========================================================================
    // Instantiate UUT (Unit Under Test)
    // =========================================================================
    traffic_system_rx #(
        .CLK_FREQ(CLK_FREQ_TB)
    ) uut (
        .clk             (clk),
        .rst_n           (rst_n),
        .emergency_sw    (emergency_sw),
        .uart_rx         (uart_rx),
        .ped_btn_ns_i    (ped_btn_ns_i),
        .ped_btn_ew_i    (ped_btn_ew_i),
        .ped_ns_req_led_o(ped_ns_req_led_o),
        .ped_ew_req_led_o(ped_ew_req_led_o),
        .sensor_ns_i     (sensor_ns_i),
        .sensor_ew_i     (sensor_ew_i),
        .mode_select_i   (mode_select_i),
        .buzzer_o        (buzzer_o),
        .uart_tx         (uart_tx),
        .ns_leds         (ns_leds),
        .ew_leds         (ew_leds),
        .ped_ns_led      (ped_ns_led),
        .ped_ew_led      (ped_ew_led),
        .seg_o           (seg_o),
        .an_o            (an_o)
    );

    // =========================================================================
    // Overrides for Fast Simulation
    // Override the clock divider FREQ inside uut.top_inst so 1Hz tick is faster.
    // 1 tick = 200 clock cycles instead of 10,000,000.
    // =========================================================================
    defparam uut.top_inst.clk_div_inst.FREQ = 200;
    defparam uut.top_inst.BUZZER_PULSE_LIMIT = 500;
    defparam uut.top_inst.BUZZER_TONE_LIMIT = 50;
    
    // DEBOUNCE_CYCLES_TB for forcing pedestrian buttons
    // uut.top_inst.ped_handler_inst.DEBOUNCE_CYCLES = (100_000_000 / 1000) * 20 = 2_000_000 cycles
    localparam DEBOUNCE_CYCLES_TB = 2_000_000;

    // =========================================================================
    // Clock Generator
    // =========================================================================
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // UART RX Byte Sender
    // =========================================================================
    task send_uart_byte;
        input [7:0] byte_data;
        integer i;
        begin
            // Start bit (LOW)
            @(negedge clk); uart_rx = 1'b0;
            repeat (BIT_CYCLES) @(posedge clk);
            
            // 8 data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                @(negedge clk); uart_rx = byte_data[i];
                repeat (BIT_CYCLES) @(posedge clk);
            end
            
            // Stop bit (HIGH)
            @(negedge clk); uart_rx = 1'b1;
            repeat (BIT_CYCLES) @(posedge clk);
        end
    endtask

    // Send valid camera density packet: 0xAA | {ns, ew} | 0x55
    task send_camera_packet;
        input [3:0] ns;
        input [3:0] ew;
        begin
            send_uart_byte(8'hAA);
            send_uart_byte({ns, ew});
            send_uart_byte(8'h55);
            repeat (100) @(posedge clk); // Gap
        end
    endtask

    // =========================================================================
    // UART TX Receiver / Decoder
    // Monitors the output uart_tx and decodes the 5-byte packet
    // =========================================================================
    reg [7:0] rx_pkt [4:0];
    reg       tx_decoded_flag;
    
    task receive_status_packet;
        integer b, i;
        begin
            tx_decoded_flag = 0;
            for (b = 0; b < 5; b = b + 1) begin
                // Wait for start of bit (line goes LOW)
                @(negedge uart_tx);
                // Wait to center of start bit
                repeat (BIT_CYCLES / 2) @(posedge clk);
                // Sample 8 data bits
                for (i = 0; i < 8; i = i + 1) begin
                    repeat (BIT_CYCLES) @(posedge clk);
                    rx_pkt[b][i] = uart_tx;
                end
                // Wait for stop bit
                repeat (BIT_CYCLES) @(posedge clk);
            end
            tx_decoded_flag = 1;
        end
    endtask

    // =========================================================================
    // Pedestrian Request Helpers
    // =========================================================================
    task press_ped_btn_ns;
        begin
            ped_btn_ns_i = 1'b1;
            repeat (5) @(posedge clk);
            force uut.top_inst.ped_handler_inst.ns_debounce_cnt = DEBOUNCE_CYCLES_TB;
            @(posedge clk); #1;
            release uut.top_inst.ped_handler_inst.ns_debounce_cnt;
            @(posedge clk); #1;
            ped_btn_ns_i = 1'b0;
            repeat (5) @(posedge clk);
        end
    endtask

    // =========================================================================
    // Main Stimulus
    // =========================================================================
    integer pass_count = 0;
    integer fail_count = 0;
    
    task assert_check;
        input condition;
        input [255:0] label;
        begin
            if (condition) begin
                $display("  [PASS] %s", label);
                pass_count = pass_count + 1;
            end else begin
                $display("  [FAIL] %s", label);
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("tb_traffic_system_rx_tx.vcd");
        $dumpvars(0, tb_traffic_system_rx_tx);
        
        $display("============================================================");
        $display("  STARTING TOP WRAPPER RX/TX & GAP-OUT SIMULATION");
        $display("============================================================");

        // --- Reset Setup ---
        rst_n         = 1'b0;
        emergency_sw  = 1'b0;
        uart_rx       = 1'b1; // Idle HIGH
        ped_btn_ns_i  = 1'b0;
        ped_btn_ew_i  = 1'b0;
        sensor_ns_i   = 1'b0;
        sensor_ew_i   = 1'b0;
        mode_select_i = 1'b0; // Start in Camera Density mode
        
        repeat (10) @(posedge clk);
        @(negedge clk); rst_n = 1'b1;
        repeat (10) @(posedge clk);

        // ---------------------------------------------------------------------
        // Test Case 1: Initial state (Camera Density Mode)
        // ---------------------------------------------------------------------
        $display("\n--- TC-01: Verify Default Startup (Camera Density Mode) ---");
        assert_check(uut.top_inst.current_state == 5'b00001, "Starts in NS_GREEN");
        assert_check(ns_leds == 3'b001, "NS light is GREEN");
        assert_check(ew_leds == 3'b100, "EW light is RED");
        assert_check(ped_ns_led == 1'b1, "NS ped walk ON");
        assert_check(ped_ew_led == 1'b0, "EW ped walk OFF");
        assert_check(ped_ns_req_led_o == 1'b0, "NS ped request LED LOW");
        assert_check(ped_ew_req_led_o == 1'b0, "EW ped request LED LOW");
        assert_check(buzzer_o == 1'b0, "Buzzer initially silent");

        // ---------------------------------------------------------------------
        // Test Case 2: UART RX - Camera Packet decoding
        // ---------------------------------------------------------------------
        $display("\n--- TC-02: UART RX Camera Packet decoding ---");
        // Send a packet: NS = 12 (4'd12), EW = 3 (4'd3)
        send_camera_packet(4'd12, 4'd3);
        
        assert_check(uut.ns_density == 4'd12, "Decoded NS density = 12");
        assert_check(uut.ew_density == 4'd3, "Decoded EW density = 3");
        assert_check(uut.camera_valid == 1'b1, "camera_valid set HIGH");

        // ---------------------------------------------------------------------
        // Test Case 3: UART TX - Status packet transmission
        // ---------------------------------------------------------------------
        $display("\n--- TC-03: UART TX Status packet decoding ---");
        // Disable automatic ticks to prevent continuous transmission
        force uut.top_inst.clk_div_inst.tick_out = 1'b0;
        // Wait for any ongoing transmission to finish
        while (uut.top_inst.uart_tx_inst.tx_busy_o) begin
            @(posedge clk);
        end
        // Small delay to ensure line is stabilized idle HIGH
        repeat (100) @(posedge clk);

        // Trigger a status packet and capture it
        fork
            receive_status_packet;
            begin
                // Pulse the tx_trigger inside the TX module
                @(negedge clk);
                force uut.top_inst.uart_tx_inst.tx_trigger = 1'b1;
                @(negedge clk);
                release uut.top_inst.uart_tx_inst.tx_trigger;
            end
        join

        // Restore automatic ticks
        release uut.top_inst.clk_div_inst.tick_out;
        
        if (tx_decoded_flag) begin
            assert_check(rx_pkt[0] == 8'hBB, "TX packet start byte 0xBB correct");
            assert_check(rx_pkt[1][7:3] == uut.top_inst.current_state, "TX packet state matches core");
            assert_check(rx_pkt[1][2] == emergency_sw, "TX packet emergency switch matches");
            assert_check(rx_pkt[1][1] == mode_select_i, "TX packet mode select matches");
            assert_check(rx_pkt[4] == 8'h55, "TX packet end byte 0x55 correct");
        end else begin
            $display("  [FAIL] Failed to decode UART TX status packet");
            fail_count = fail_count + 1;
        end

        // ---------------------------------------------------------------------
        // Test Case 4: Pedestrian Button Request & Buzzer Output
        // ---------------------------------------------------------------------
        $display("\n--- TC-04: Pedestrian Button and Buzzer Output ---");
        // Press NS button during S_NS_GREEN (wait, serving is done next cycle or resets on entry)
        // Let's transition to S_EW_GREEN first
        while (uut.top_inst.current_state != 5'b00100) begin
            @(posedge clk);
        end
        $display("Entered EW_GREEN, pressing NS pedestrian button");
        press_ped_btn_ns;
        assert_check(ped_ns_req_led_o == 1'b1, "NS ped request LED lit");
        
        // Wait until it returns to NS_GREEN
        while (uut.top_inst.current_state != 5'b00001) begin
            @(posedge clk);
        end
        @(posedge clk); #1;
        assert_check(ped_ns_req_led_o == 1'b0, "NS ped request LED cleared on entry");
        assert_check(ped_ns_led == 1'b1, "NS ped walk ON");
        
        // Verify buzzer output (buzzer_o should be pulsing because ped_ns_led is active)
        // We wait a few cycles to see if buzzer toggles
        begin
            integer b_cnt;
            reg buzzer_saw_high, buzzer_saw_low;
            buzzer_saw_high = 0;
            buzzer_saw_low  = 0;
            for (b_cnt = 0; b_cnt < 1000; b_cnt = b_cnt + 1) begin
                @(posedge clk);
                if (buzzer_o === 1'b1) buzzer_saw_high = 1;
                if (buzzer_o === 1'b0) buzzer_saw_low = 1;
            end
            assert_check(buzzer_saw_high && buzzer_saw_low, "Buzzer output is pulsing (toggles between 0 and 1)");
        end

        // ---------------------------------------------------------------------
        // Test Case 5: Mode Switch & Gap-Out Loop Sensor extensions
        // ---------------------------------------------------------------------
        $display("\n--- TC-05: Gap-Out Logic & Loop Sensor Extension ---");
        // Switch to Gap-Out mode
        mode_select_i = 1'b1;
        sensor_ns_i   = 1'b1; // Hold NS sensor active (car waiting/passing)
        
        // Wait until we are in NS_GREEN
        while (uut.top_inst.current_state != 5'b00001) begin
            @(posedge clk);
        end
        $display("Entered NS_GREEN in Gap-Out Mode, sensor_ns_i = 1");
        
        // Timer count should start at T_GREEN_MIN (8s).
        // Let's watch the countdown timer reach 0, and verify it gets reloaded to EXT_TIME (3s)
        // since sensor_ns_i is active and active_green_timer < T_GREEN_MAX.
        while (uut.top_inst.timer_count != 0) begin
            @(posedge clk);
        end
        // At timer == 0, on the next clock cycle, reload_en should be asserted
        // and uut.top_inst.timer_count should be loaded to 3.
        repeat (2) @(posedge clk); #1;
        assert_check(uut.top_inst.timer_count == 3, "Timer reloaded to EXT_TIME (3s) when sensor active");
        
        // Deassert sensor and watch it gap-out to Yellow
        sensor_ns_i = 1'b0;
        while (uut.top_inst.current_state == 5'b00001) begin
            @(posedge clk);
        end
        assert_check(uut.top_inst.current_state == 5'b00010, "Gapped-out to NS_YELLOW when sensor went inactive");

        // ---------------------------------------------------------------------
        // Test Case 6: Emergency Mode
        // ---------------------------------------------------------------------
        $display("\n--- TC-06: Emergency Mode override ---");
        emergency_sw = 1'b1;
        repeat (10) @(posedge clk);
        assert_check(uut.top_inst.current_state == 5'b10000, "FSM entered S_ERROR");
        assert_check(ped_ns_led == 1'b0 && ped_ew_led == 1'b0, "Walk lights disabled");
        
        // Release emergency
        emergency_sw = 1'b0;
        repeat (10) @(posedge clk);
        
        // Note: FSM is sticky in S_ERROR until rst_n is asserted. Verify this.
        assert_check(uut.top_inst.current_state == 5'b10000, "FSM remains in S_ERROR without reset");
        
        // Reset to recover
        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);
        assert_check(uut.top_inst.current_state == 5'b00001, "FSM recovered to NS_GREEN after reset");

        // =====================================================================
        // Final Results Summary
        // =====================================================================
        $display("\n============================================================");
        $display("  TEST RESULTS");
        $display("============================================================");
        $display("  PASS : %0d", pass_count);
        $display("  FAIL : %0d", fail_count);
        $display("  TOTAL: %0d", pass_count + fail_count);
        if (fail_count == 0)
            $display("  *** ALL WRAPPER TESTS PASSED ***");
        else
            $display("  *** %0d TEST(S) FAILED — check logs above ***", fail_count);
        $display("============================================================\n");

        $finish;
    end

endmodule
