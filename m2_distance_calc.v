// ============================================================
//  FILE   : m2_distance_calc.v
//  MODULE : M2 - Distance Calculator
//  PROJECT: Smart Parking Lot
// ------------------------------------------------------------
//  DESCRIPTION
//  -----------
//  Receives the raw echo_ticks measurement and slot_id from
//  M1's round-robin scheduler and converts the time-of-flight
//  count into a distance in centimetres.
//
//  HC-SR04 distance formula:
//    distance_cm = echo_duration_us / 58
//
//  At CLK_FREQ = 50 MHz, one tick = 20 ns = 0.02 µs.
//  echo_duration_us = echo_ticks * (1_000_000 / CLK_FREQ)
//                   = echo_ticks / 50          (for 50 MHz)
//
//  Therefore:
//    distance_cm = echo_ticks / (50 * 58)
//                = echo_ticks / TICKS_PER_CM
//
//  TICKS_PER_CM = CLK_FREQ / 1_000_000 * 58
//               = 50 * 58 = 2900  (@ 50 MHz)
//
//  If echo_ticks == ECHO_TIMEOUT the sensor returned no echo
//  (slot out of range / no object).  In that case dist_valid
//  is still asserted but distance_cm is forced to MAX_DIST
//  so M3 always classifies the slot as FREE.
//
//  PORTS
//  -----
//  echo_ticks   [20:0] : raw tick count from M1
//  slot_id      [1:0]  : which slot (0/1/2) from M1
//  valid_in            : 1-cycle strobe from M1
//  distance_cm  [8:0]  : computed distance 0-511 cm
//  slot_id_out  [1:0]  : pass-through slot id
//  dist_valid          : 1-cycle strobe to M3
// ============================================================
`timescale 1ns / 1ps

module m2_distance_calc #(
    parameter CLK_FREQ    = 50_000_000,
    parameter ECHO_TIMEOUT = 1_900_000   // must match M1
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---------- From M1 ----------
    input  wire [20:0] echo_ticks,
    input  wire [1:0]  slot_id,
    input  wire        valid_in,

    // ---------- To M3 ----------
    output reg  [8:0]  distance_cm,
    output reg  [1:0]  slot_id_out,
    output reg         dist_valid
);

    // ------------------------------------------------------------------
    // Derived constant: clock ticks per centimetre
    // = (CLK_FREQ / 1_000_000) * 58
    // For 50 MHz: 50 * 58 = 2900
    // ------------------------------------------------------------------
    localparam integer TICKS_PER_CM = (CLK_FREQ / 1_000_000) * 58;

    // Maximum distance we report when there is no echo (out of range).
    // 511 saturates the 9-bit output and ensures M3 sees FREE.
    localparam [8:0] MAX_DIST = 9'd511;

    // ------------------------------------------------------------------
    // Division: distance_cm = echo_ticks / TICKS_PER_CM
    //
    // We use a simple combinational divide. TICKS_PER_CM is a
    // compile-time constant so synthesis will optimise this heavily.
    // A 21-bit / 12-bit divide costs ~12 subtract-and-shift stages -
    // well within one clock cycle at 50 MHz on any modern FPGA.
    // ------------------------------------------------------------------
    wire [8:0] raw_dist = echo_ticks[20:0] / TICKS_PER_CM;

    // ------------------------------------------------------------------
    // Output register - latch result one cycle after valid_in
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            distance_cm  <= 9'd0;
            slot_id_out  <= 2'd0;
            dist_valid   <= 1'b0;
        end else begin
            dist_valid <= 1'b0;     // default: deasserted

            if (valid_in) begin
                slot_id_out <= slot_id;
                dist_valid  <= 1'b1;

                if (echo_ticks >= ECHO_TIMEOUT[20:0])
                    // No echo received - out of range, treat as FREE
                    distance_cm <= MAX_DIST;
                else
                    distance_cm <= (raw_dist > MAX_DIST) ? MAX_DIST : raw_dist;
            end
        end
    end

endmodule
