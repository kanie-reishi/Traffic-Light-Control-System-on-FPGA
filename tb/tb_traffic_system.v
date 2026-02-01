`timescale 1ns / 1ps

module tb_traffic_system;

    // 1. Khai báo tín hiệu (Inputs là reg, Outputs là wire)
    reg clk;
    reg rst_n;
    reg emergency_sw;

    wire [2:0] ns_leds;
    wire [2:0] ew_leds;
    wire ped_ns_led;
    wire ped_ew_led;

    // Định nghĩa các tham số màu để dễ đọc code kiểm tra (Assertion)
    parameter RED    = 3'b100;
    parameter YELLOW = 3'b010;
    parameter GREEN  = 3'b001;

    // 2. Instantiate Unit Under Test (UUT)
    traffic_system_top uut (
        .clk(clk),
        .rst_n(rst_n),
        .emergency_sw(emergency_sw),
        .ns_leds(ns_leds),
        .ew_leds(ew_leds),
        .ped_ns_led(ped_ns_led),
        .ped_ew_led(ped_ew_led)
    );

    // ============================================================
    // KỸ THUẬT DV: PARAMETER OVERRIDE (HIERARCHICAL REFERENCE)
    // Thay đổi tham số FREQ của bộ chia xung nằm sâu bên trong thiết kế
    // để mô phỏng nhanh hơn (4 clock = 1 giây thực tế mô phỏng)
    // ============================================================
    defparam uut.clk_div_inst.FREQ = 4; 

    // 3. Clock Generator (50MHz -> Chu kỳ 20ns)
    initial begin
        clk = 0;
        forever #10 clk = ~clk;
    end

    // 4. Main Stimulus (Kịch bản kiểm tra)
    initial begin
        // Thiết lập màn hình console
        $display("==================================================");
        $display("STARTING TRAFFIC LIGHT SYSTEM SIMULATION");
        $display("==================================================");

        // a. Khởi tạo ban đầu
        rst_n = 0;
        emergency_sw = 0;
        #100; // Giữ reset một lúc

        // b. Thả Reset - Hệ thống bắt đầu chạy Normal Mode
        $display("[%0t] Releasing Reset. System should start.", $time);
        rst_n = 1;

        // Chờ đủ lâu để quan sát hết 1 vòng chu trình
        // (10s Xanh + 5s Vàng + 10s Xanh + 5s Vàng) * 4 clock/s * 20ns = số lớn
        #10000; 

        // c. Kiểm tra chế độ khẩn cấp (Emergency Mode)
        $display("[%0t] Activating EMERGENCY MODE (Switch ON)", $time);
        emergency_sw = 1;
        
        #600; // Chờ một chút để xem đèn vàng nhấp nháy

        // d. Tắt chế độ khẩn cấp - Quay lại bình thường
        $display("[%0t] Deactivating EMERGENCY MODE (Switch OFF)", $time);
        emergency_sw = 0;

        #1000; // Chờ hệ thống phục hồi

        $display("==================================================");
        $display("SIMULATION FINISHED");
        $display("==================================================");
        $stop; // Dừng mô phỏng
    end

    // 5. MONITOR & ASSERTION (Tự động kiểm tra lỗi)
    // Đây là phần giúp bạn ghi điểm DV: Máy tự kiểm tra xem có an toàn không
    
    // Monitor: In trạng thái ra màn hình mỗi khi đèn thay đổi
    always @(ns_leds or ew_leds) begin
        if (rst_n) begin
            $display("Time: %0t | NS Lights: %b | EW Lights: %b", 
                     $time, ns_leds, ew_leds);
        end
    end

    // Assertion (Simple): Kiểm tra an toàn giao thông
    // Lỗi nghiêm trọng: Cả hai bên đều Xanh cùng lúc
    always @(posedge clk) begin
        if (ns_leds == GREEN && ew_leds == GREEN) begin
            $display("\n[FATAL ERROR] BOTH DIRECTIONS ARE GREEN AT %0t! COLLISION IMMINENT!\n", $time);
            $stop; // Dừng ngay lập tức nếu lỗi này xảy ra
        end
    end

endmodule