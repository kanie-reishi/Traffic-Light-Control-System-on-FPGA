// ============================================================================
// seg7_hex_decoder.v
// Combinational BCD-to-7-Segment Decoder for Common-Anode Displays (Basys 3)
//
// Purpose
// -------
//   Converts a 4-bit BCD digit (0-9) into a 7-bit active-LOW cathode
//   pattern suitable for the Basys 3 common-anode 7-segment displays.
//   Invalid BCD inputs (10-15) produce a dash character for visual
//   indication of an error.
//
// Segment Layout
// --------------
//        AAA
//       F   B
//        GGG
//       E   C
//        DDD
//
// Bit Mapping (matches Basys 3 XDC pin order)
// --------------------------------------------
//   seg_o[0] = Segment A    seg_o[4] = Segment E
//   seg_o[1] = Segment B    seg_o[5] = Segment F
//   seg_o[2] = Segment C    seg_o[6] = Segment G
//   seg_o[3] = Segment D
//
//   Logic: 0 = segment ON (active-low cathode), 1 = segment OFF
//
// Ports
// -----
//   bcd_i[3:0] — Input BCD digit (0-9 valid, 10-15 show dash)
//   seg_o[6:0] — Output cathode pattern {G, F, E, D, C, B, A} active-low
// ============================================================================
module seg7_hex_decoder (
    input  wire [3:0] bcd_i,
    output reg  [6:0] seg_o
);

    always @(*) begin
        case (bcd_i)
            //                   GFEDCBA
            4'd0: seg_o = 7'b100_0000;  // 0: segments A,B,C,D,E,F ON
            4'd1: seg_o = 7'b111_1001;  // 1: segments B,C ON
            4'd2: seg_o = 7'b010_0100;  // 2: segments A,B,D,E,G ON
            4'd3: seg_o = 7'b011_0000;  // 3: segments A,B,C,D,G ON
            4'd4: seg_o = 7'b001_1001;  // 4: segments B,C,F,G ON
            4'd5: seg_o = 7'b001_0010;  // 5: segments A,C,D,F,G ON
            4'd6: seg_o = 7'b000_0010;  // 6: segments A,C,D,E,F,G ON
            4'd7: seg_o = 7'b111_1000;  // 7: segments A,B,C ON
            4'd8: seg_o = 7'b000_0000;  // 8: all segments ON
            4'd9: seg_o = 7'b001_0000;  // 9: segments A,B,C,D,F,G ON
            default: seg_o = 7'b011_1111;  // dash: segment G only (invalid BCD)
        endcase
    end

endmodule
