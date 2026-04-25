// ============================================================
//  FILE   : m3_threshold_cmp.v
//  MODULE : M3 - Threshold Comparator
//  PROJECT: Smart Parking Lot
// ------------------------------------------------------------
//  DESCRIPTION
//  -----------
//  Receives distance_cm and slot_id_out from M2 and decides
//  whether each parking slot is OCCUPIED or FREE by comparing
//  the measured distance against a configurable threshold.
//
//  Classification rule:
//    distance_cm < THRESHOLD_CM  =>  slot_status[n] = 1  (OCCUPIED)
//    distance_cm >= THRESHOLD_CM =>  slot_status[n] = 0  (FREE)
//
//  A typical HC-SR04 installation in a parking bay:
//    Sensor mounted on ceiling / beam above bay.
//    Empty bay: sensor reads ~200 cm (ceiling to floor).
//    Car present: sensor reads ~50-100 cm.
//    => THRESHOLD_CM = 100 is a safe default.
//
//  Each slot has its own status bit in the 3-bit output
//  register.  Only the bit for the incoming slot_id is
//  updated on each dist_valid pulse; the other two bits
//  retain their previous value.
//
//  PORTS
//  -----
//  distance_cm  [8:0] : distance in cm from M2
//  slot_id_in   [1:0] : which slot (0/1/2) from M2
//  dist_valid         : 1-cycle strobe from M2
//  slot_status  [2:0] : occupancy bits to M4
//                       bit 0 = slot 1, bit 1 = slot 2, bit 2 = slot 3
//                       1 = OCCUPIED, 0 = FREE
// ============================================================
`timescale 1ns / 1ps

module m3_threshold_cmp #(
    parameter THRESHOLD_CM = 9'd100    // car closer than 100 cm = occupied
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---------- From M2 ----------
    input  wire [8:0]  distance_cm,
    input  wire [1:0]  slot_id_in,
    input  wire        dist_valid,

    // ---------- To M4 ----------
    output reg  [2:0]  slot_status    // [0]=slot1, [1]=slot2, [2]=slot3
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slot_status <= 3'b000;   // all FREE on reset
        end else begin
            if (dist_valid) begin
                case (slot_id_in)
                    2'd0: slot_status[0] <= (distance_cm < THRESHOLD_CM);
                    2'd1: slot_status[1] <= (distance_cm < THRESHOLD_CM);
                    2'd2: slot_status[2] <= (distance_cm < THRESHOLD_CM);
                    // slot_id 3 is unused (only 3 sensors)
                    default: ;
                endcase
            end
        end
    end

endmodule
