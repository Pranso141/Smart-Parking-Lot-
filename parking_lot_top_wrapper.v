// ============================================================
//  FILE   : parking_lot_top_wrapper.v
//  MODULE : parking_lot_top_wrapper
//  BOARD  : EDGE Artix 7 (XC7A35T-1FTG256)
//  PROJECT: Smart Parking Lot
// ------------------------------------------------------------
//  PURPOSE
//  -------
//  Thin board-level wrapper around parking_lot_top that:
//
//  1. RESET POLARITY
//     The EDGE board push button pb[0] is active-HIGH (pressed=1).
//     parking_lot_top expects active-LOW rst_n (released=1).
//     This wrapper inverts: rst_n = ~btn_reset
//
//  2. LCD RW
//     The board ties LCD R/W to GND on-board (J7 pin 5 = GND).
//     parking_lot_top drives lcd_rw as an output but it is
//     always 0. We simply leave it unconnected at this wrapper
//     boundary â€” Vivado will give a harmless "unconnected output"
//     message which you can suppress with set_false_path.
//
//  3. DEBUG LED ROUTING
//     Routes dbg_* internal signals to the 7 onboard LEDs
//     for easy bring-up verification without a logic analyser.
//
//  PORT NAMES match the XDC file exactly.
// ============================================================
`timescale 1ns / 1ps

module parking_lot_top_wrapper (
    // ---- Clock & Reset ----
    input  wire        clk,         // N11 â€” 50 MHz on-board oscillator
    input  wire        btn_reset,   // K13 â€” pb[0] top button, active-HIGH

    // ---- HC-SR04 Sensor Interface ----
    input  wire [2:0]  echo_in,     // J11 pins 6,8,10 (via voltage divider)
    output wire [2:0]  trig_out,    // J11 pins 5,7,9

    // ---- HD44780 LCD Interface ----
    output wire        lcd_rs,      // P5
    output wire        lcd_e,       // T3
    output wire [7:0]  lcd_data,    // P3,M5,N4,R2,R1,R3,T2,T4

    // ---- Debug LEDs ----
    output wire [6:0]  led          // J3,H3,J1,K1,L3,L2,K3
);

    // ------------------------------------------------------------------
    // Reset: invert active-HIGH button to active-LOW rst_n
    // ------------------------------------------------------------------
    wire rst_n = ~btn_reset;

    // ------------------------------------------------------------------
    // Internal debug wires from parking_lot_top
    // ------------------------------------------------------------------
    wire [20:0] dbg_echo_ticks;
    wire [1:0]  dbg_slot_id;
    wire        dbg_valid;
    wire [8:0]  dbg_distance_cm;
    wire        dbg_dist_valid;
    wire [2:0]  dbg_slot_status;
    wire        lcd_rw_unused;  // always 0; board ties R/W to GND anyway

    // ------------------------------------------------------------------
    // Core DUT
    // ------------------------------------------------------------------
    parking_lot_top #(
        // Real 50 MHz clock â€” all timing parameters stay at default
        .CLK_FREQ     (50_000_000),
        .TRIG_TICKS   (500),           // 10 Âµs
        .ECHO_TIMEOUT (1_900_000),     // 38 ms
        .SETTLE_TICKS (500_000),       // 60 ms
        .THRESHOLD_CM (9'd100),        // <100 cm = OCCUPIED
        .T_PWRON      (2_500_000),     // 50 ms LCD power-on delay
        .T_EHIGH      (25),            // 500 ns E-pulse
        .T_CMD        (2_000),         // 40 Âµs command settle
        .T_CLR        (80_000)         // 1.6 ms clear settle
    ) u_core (
        .clk            (clk),
        .rst_n          (rst_n),
        .echo_in        (echo_in),
        .trig_out       (trig_out),
        .lcd_rs         (lcd_rs),
        .lcd_rw         (lcd_rw_unused),
        .lcd_e          (lcd_e),
        .lcd_data       (lcd_data),
        .dbg_echo_ticks (dbg_echo_ticks),
        .dbg_slot_id    (dbg_slot_id),
        .dbg_valid      (dbg_valid),
        .dbg_distance_cm(dbg_distance_cm),
        .dbg_dist_valid (dbg_dist_valid),
        .dbg_slot_status(dbg_slot_status)
    );

    // ------------------------------------------------------------------
    // LED routing
    //
    //  led[0] = Slot 1 OCCUPIED (lights solid when car in bay 1)
    //  led[1] = Slot 2 OCCUPIED (lights solid when car in bay 2)
    //  led[2] = Slot 3 OCCUPIED (lights solid when car in bay 3)
    //  led[3] = M1 valid strobe (brief flash ~every 144ms in real time)
    //  led[4] = trig_out[0] (pulse visible as flicker during sensor 1 poll)
    //  led[5] = trig_out[1]
    //  led[6] = trig_out[2]
    // ------------------------------------------------------------------
    assign led[0] = dbg_slot_status[0];   // Slot 1
    assign led[1] = dbg_slot_status[1];   // Slot 2
    assign led[2] = dbg_slot_status[2];   // Slot 3
    assign led[3] = dbg_valid;            // M1 strobe
    assign led[4] = trig_out[0];          // Sensor 1 trigger active
    assign led[5] = trig_out[1];          // Sensor 2 trigger active
    assign led[6] = trig_out[2];          // Sensor 3 trigger active

endmodule
