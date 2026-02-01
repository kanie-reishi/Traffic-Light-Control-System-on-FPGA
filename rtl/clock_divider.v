module clock_divider #(
    parameter FREQ = 50000000 // Tần số mặc định 50MHz
)(
    input  wire clk,      // Clock hệ thống tốc độ cao
    input  wire rst_n,    // Reset tích cực thấp
    output reg  tick_out  // Xung 1Hz (chỉ mức cao trong 1 chu kỳ clock)
);

    // Tính toán số bit cần thiết để đếm tới FREQ
    // log2(50,000,000) ~ 25.5 -> Cần 26 bit
    reg [25:0] counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter  <= 0;
            tick_out <= 0;
        end else begin
            if (counter == FREQ - 1) begin
                counter  <= 0;
                tick_out <= 1; // Bật tín hiệu tick
            end else begin
                counter  <= counter + 1;
                tick_out <= 0; // Tắt ngay lập tức
            end
        end
    end

endmodule