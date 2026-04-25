// ============================================================
//  FILE   : m1_scheduler_top.v
//  MODULE : M1 - Round-Robin HC-SR04 Ultrasonic Scheduler
//  PROJECT: Smart Parking Lot
// ------------------------------------------------------------
//  DESCRIPTION
//  -----------
//  Drives three HC-SR04 ultrasonic sensors in strict round-
//  robin order so their echo pulses never overlap:
//
//    1. Assert a 10 µs trigger on the ACTIVE sensor only.
//    2. Wait for the echo pin to go HIGH (rising edge detect).
//       If no rising edge within ECHO_TIMEOUT → treat as
//       "no object / out of range."
//    3. Count clock ticks while echo is HIGH → raw distance
//       proportional to time-of-flight.
//    4. Output {echo_ticks, slot_id} with a 1-cycle valid
//       strobe to M2 (Distance Calculator).
//    5. Enforce a SETTLE_TICKS dead time before the next
//       sensor to prevent acoustic crosstalk.
//    6. Advance slot_ptr: 0 → 1 → 2 → 0 (round-robin).
//
//  PARAMETERS (override for simulation or different clocks)
//  ----------------------------------------------------------
//  CLK_FREQ     : system clock in Hz            (default 50 MHz)
//  TRIG_TICKS   : trigger pulse width in ticks  (default 500 = 10 µs)
//  ECHO_TIMEOUT : max echo wait in ticks         (default 1 900 000 = 38 ms)
//  SETTLE_TICKS : inter-sensor dead time         (default 500 000 = 10 ms)
//
//  PORTS
//  -----
//  echo_in  [2:0] : echo pins from sensors 1/2/3 (bit 0 = slot 1)
//  trig_out [2:0] : trigger pins to sensors 1/2/3 (bit 0 = slot 1)
//  echo_ticks     : captured echo pulse width (clock ticks) → to M2
//  slot_id  [1:0] : 0=slot1, 1=slot2, 2=slot3
//  valid          : 1-cycle strobe - latch echo_ticks + slot_id now
// ============================================================
`timescale 1ns / 1ps

module m1_scheduler_top #(
    parameter CLK_FREQ    = 50_000_000,
    parameter TRIG_TICKS  = 500,          // 10 µs  @ 50 MHz
    parameter ECHO_TIMEOUT= 1_900_000,    // 38 ms  @ 50 MHz
    parameter SETTLE_TICKS= 500_000       // 10 ms  @ 50 MHz
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---------- HC-SR04 Sensor Interface ----------
    input  wire [2:0]  echo_in,     // echo[0]=slot1, [1]=slot2, [2]=slot3
    output reg  [2:0]  trig_out,    // trig[0]=slot1, [1]=slot2, [2]=slot3

    // ---------- Output to M2 ----------------------
    output reg  [20:0] echo_ticks,  // echo pulse duration (clock ticks)
    output reg  [1:0]  slot_id,     // which slot: 0,1,2
    output reg         valid        // 1-cycle strobe: outputs are valid
);

    // ------------------------------------------------------------------
    // FSM State Encoding
    // ------------------------------------------------------------------
    localparam [2:0]
        S_IDLE    = 3'd0,   // reset counters, prepare for trigger
        S_TRIG    = 3'd1,   // assert trigger HIGH for TRIG_TICKS
        S_WAIT_HI = 3'd2,   // wait for echo rising edge (or timeout)
        S_MEASURE = 3'd3,   // count ticks while echo is HIGH
        S_SETTLE  = 3'd4;   // inter-sensor dead time

    reg [2:0]  state;
    reg [1:0]  slot_ptr;    // active sensor index: 0, 1, 2
    reg [20:0] cnt;         // multi-purpose timer/counter
    reg [20:0] meas;        // echo high-time accumulator

    // Mux: select echo signal of the currently active sensor
    wire echo_cur = echo_in[slot_ptr];

    // ------------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            slot_ptr   <= 2'd0;
            cnt        <= 21'd0;
            meas       <= 21'd0;
            trig_out   <= 3'b000;
            echo_ticks <= 21'd0;
            slot_id    <= 2'd0;
            valid      <= 1'b0;
        end else begin
            valid <= 1'b0;          // default: strobe deasserted

            case (state)

                // --------------------------------------------------
                // IDLE: de-assert all triggers, clear counters
                // --------------------------------------------------
                S_IDLE: begin
                    trig_out <= 3'b000;
                    cnt      <= 21'd0;
                    meas     <= 21'd0;
                    state    <= S_TRIG;
                end

                // --------------------------------------------------
                // TRIG: assert trigger on the active sensor for
                //        exactly TRIG_TICKS clock cycles (= 10 µs)
                // --------------------------------------------------
                S_TRIG: begin
                    trig_out[slot_ptr] <= 1'b1;
                    if (cnt == TRIG_TICKS - 1) begin
                        trig_out[slot_ptr] <= 1'b0;
                        cnt   <= 21'd0;
                        state <= S_WAIT_HI;
                    end else
                        cnt <= cnt + 1'b1;
                end

                // --------------------------------------------------
                // WAIT_HI: watch for echo rising edge.
                //   - If echo goes HIGH   → start measurement.
                //   - If ECHO_TIMEOUT hit → no object, emit result.
                // --------------------------------------------------
                S_WAIT_HI: begin
                    if (echo_cur) begin
                        cnt   <= 21'd0;
                        meas  <= 21'd0;
                        state <= S_MEASURE;
                    end else if (cnt == ECHO_TIMEOUT - 1) begin
                        // No echo received - slot is empty / out of range
                        echo_ticks <= ECHO_TIMEOUT[20:0];
                        slot_id    <= slot_ptr;
                        valid      <= 1'b1;
                        cnt        <= 21'd0;
                        state      <= S_SETTLE;
                    end else
                        cnt <= cnt + 1'b1;
                end

                // --------------------------------------------------
                // MEASURE: count cycles while echo pin is HIGH.
                //   - Falling edge of echo → capture measurement.
                //   - If echo stuck HIGH past ECHO_TIMEOUT → timeout.
                // --------------------------------------------------
                S_MEASURE: begin
                    if (!echo_cur) begin
                        // Normal falling edge - valid measurement
                        echo_ticks <= meas;
                        slot_id    <= slot_ptr;
                        valid      <= 1'b1;
                        cnt        <= 21'd0;
                        state      <= S_SETTLE;
                    end else if (meas == ECHO_TIMEOUT - 1) begin
                        // Echo stuck high - force timeout output
                        echo_ticks <= ECHO_TIMEOUT[20:0];
                        slot_id    <= slot_ptr;
                        valid      <= 1'b1;
                        cnt        <= 21'd0;
                        state      <= S_SETTLE;
                    end else
                        meas <= meas + 1'b1;
                end

                // --------------------------------------------------
                // SETTLE: mandatory dead time (10 ms) between sensors.
                //   Prevents reflected echoes from one sensor being
                //   captured by the next sensor's measurement window.
                //   After settling, advance round-robin pointer.
                // --------------------------------------------------
                S_SETTLE: begin
                    if (cnt == SETTLE_TICKS - 1) begin
                        // Round-robin: 0 → 1 → 2 → 0
                        slot_ptr <= (slot_ptr == 2'd2) ? 2'd0
                                                       : slot_ptr + 1'b1;
                        cnt      <= 21'd0;
                        state    <= S_IDLE;
                    end else
                        cnt <= cnt + 1'b1;
                end

                default: state <= S_IDLE;

            endcase
        end
    end

    // ------------------------------------------------------------------
    // Synthesis guard: ensure slot_ptr never exceeds 2
    // (Quartus / Vivado will optimise this away in unused paths)
    // ------------------------------------------------------------------
    // synthesis translate_off
    always @(posedge clk)
        if (slot_ptr > 2'd2)
            $fatal(1, "m1_scheduler_top: slot_ptr out of range (%0d)", slot_ptr);
    // synthesis translate_on

endmodule
