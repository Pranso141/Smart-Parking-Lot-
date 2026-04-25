// ============================================================
//  FILE   : parking_lot_top.v
//  MODULE : parking_lot_top - Smart Parking Lot Top Level
//  PROJECT: Smart Parking Lot
// ------------------------------------------------------------
//  DESCRIPTION
//  -----------
//  Integrates all four milestone modules into a single
//  synthesisable top-level entity:
//
//    M1  m1_scheduler_top   - Round-robin HC-SR04 scheduler
//    M2  m2_distance_calc   - Echo ticks to cm conversion
//    M3  m3_threshold_cmp   - Occupied / Free classification
//    M4  m4_lcd_top         - HD44780 16x2 LCD display driver
//
//  Signal chain:
//    echo_in[2:0] ---> M1 ---> M2 ---> M3 ---> M4 ---> LCD
//
//  All timing parameters are exposed at this level so a
//  single parameter override on the top module is sufficient
//  for simulation speedup or targeting a different FPGA clock.
//
//  PORTS
//  -----
//  clk          : system clock (50 MHz default)
//  rst_n        : active-low asynchronous reset
//  echo_in[2:0] : HC-SR04 echo pins (bit 0 = slot 1)
//  trig_out[2:0]: HC-SR04 trigger pins (bit 0 = slot 1)
//  lcd_rs       : HD44780 Register Select
//  lcd_rw       : HD44780 Read/Write (always 0)
//  lcd_e        : HD44780 Enable
//  lcd_data[7:0]: HD44780 8-bit data bus
//
//  EXPOSED INTERNAL SIGNALS (for testbench monitoring)
//  ----
//  dbg_echo_ticks[20:0] : raw tick count from M1
//  dbg_slot_id   [1:0]  : slot id from M1
//  dbg_valid            : M1 valid strobe
//  dbg_distance_cm[8:0] : computed distance from M2
//  dbg_dist_valid       : M2 valid strobe
//  dbg_slot_status[2:0] : occupancy register from M3
// ============================================================
`timescale 1ns / 1ps

module parking_lot_top #(
    // ---- Shared ----
    parameter CLK_FREQ     = 50_000_000,

    // ---- M1 parameters ----
    parameter TRIG_TICKS   = 500,          // 10 µs  @ 50 MHz
    parameter ECHO_TIMEOUT = 1_900_000,    // 38 ms  @ 50 MHz
    parameter SETTLE_TICKS = 500_000,      // 10 ms  @ 50 MHz

    // ---- M3 parameter ----
    parameter THRESHOLD_CM = 9'd100,       // <100 cm = occupied

    // ---- M4 parameters ----
    parameter T_PWRON      = 2_500_000,    // 50 ms
    parameter T_EHIGH      = 25,           // 500 ns
    parameter T_CMD        = 2_000,        // 40 µs
    parameter T_CLR        = 80_000        // 1.6 ms
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---------- HC-SR04 Sensor Interface ----------
    input  wire [2:0]  echo_in,
    output wire [2:0]  trig_out,

    // ---------- HD44780 LCD Interface ----------
    output wire        lcd_rs,
    output wire        lcd_rw,
    output wire        lcd_e,
    output wire [7:0]  lcd_data,

    // ---------- Debug / Probe Outputs ----------
    output wire [20:0] dbg_echo_ticks,
    output wire [1:0]  dbg_slot_id,
    output wire        dbg_valid,
    output wire [8:0]  dbg_distance_cm,
    output wire        dbg_dist_valid,
    output wire [2:0]  dbg_slot_status
);

    // ------------------------------------------------------------------
    // Internal wires connecting M1 → M2 → M3 → M4
    // ------------------------------------------------------------------

    // M1 → M2
    wire [20:0] m1_echo_ticks;
    wire [1:0]  m1_slot_id;
    wire        m1_valid;

    // M2 → M3
    wire [8:0]  m2_distance_cm;
    wire [1:0]  m2_slot_id_out;
    wire        m2_dist_valid;

    // M3 → M4
    wire [2:0]  m3_slot_status;

    // ------------------------------------------------------------------
    // M1: Round-Robin Scheduler
    // ------------------------------------------------------------------
    m1_scheduler_top #(
        .CLK_FREQ     (CLK_FREQ),
        .TRIG_TICKS   (TRIG_TICKS),
        .ECHO_TIMEOUT (ECHO_TIMEOUT),
        .SETTLE_TICKS (SETTLE_TICKS)
    ) u_m1 (
        .clk        (clk),
        .rst_n      (rst_n),
        .echo_in    (echo_in),
        .trig_out   (trig_out),
        .echo_ticks (m1_echo_ticks),
        .slot_id    (m1_slot_id),
        .valid      (m1_valid)
    );

    // ------------------------------------------------------------------
    // M2: Distance Calculator
    // ------------------------------------------------------------------
    m2_distance_calc #(
        .CLK_FREQ     (CLK_FREQ),
        .ECHO_TIMEOUT (ECHO_TIMEOUT)
    ) u_m2 (
        .clk         (clk),
        .rst_n       (rst_n),
        .echo_ticks  (m1_echo_ticks),
        .slot_id     (m1_slot_id),
        .valid_in    (m1_valid),
        .distance_cm (m2_distance_cm),
        .slot_id_out (m2_slot_id_out),
        .dist_valid  (m2_dist_valid)
    );

    // ------------------------------------------------------------------
    // M3: Threshold Comparator
    // ------------------------------------------------------------------
    m3_threshold_cmp #(
        .THRESHOLD_CM (THRESHOLD_CM)
    ) u_m3 (
        .clk         (clk),
        .rst_n       (rst_n),
        .distance_cm (m2_distance_cm),
        .slot_id_in  (m2_slot_id_out),
        .dist_valid  (m2_dist_valid),
        .slot_status (m3_slot_status)
    );

    // ------------------------------------------------------------------
    // M4: LCD Formatter + HD44780 Driver
    // ------------------------------------------------------------------
    m4_lcd_top #(
        .CLK_FREQ (CLK_FREQ),
        .T_PWRON  (T_PWRON),
        .T_EHIGH  (T_EHIGH),
        .T_CMD    (T_CMD),
        .T_CLR    (T_CLR)
    ) u_m4 (
        .clk         (clk),
        .rst_n       (rst_n),
        .slot_status (m3_slot_status),
        .lcd_rs      (lcd_rs),
        .lcd_rw      (lcd_rw),
        .lcd_e       (lcd_e),
        .lcd_data    (lcd_data)
    );

    // ------------------------------------------------------------------
    // Debug output assignments
    // ------------------------------------------------------------------
    assign dbg_echo_ticks  = m1_echo_ticks;
    assign dbg_slot_id     = m1_slot_id;
    assign dbg_valid       = m1_valid;
    assign dbg_distance_cm = m2_distance_cm;
    assign dbg_dist_valid  = m2_dist_valid;
    assign dbg_slot_status = m3_slot_status;

endmodule
