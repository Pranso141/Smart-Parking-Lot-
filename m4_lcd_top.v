// ============================================================
//  FILE   : m4_lcd_top.v
//  MODULE : M4 - LCD Formatter + HD44780 16×2 Display Driver
//  PROJECT: Smart Parking Lot
// ------------------------------------------------------------
//  DESCRIPTION
//  -----------
//  Displays live parking slot status on a standard 16×2
//  character LCD (HD44780 / compatible, 8-bit parallel mode).
//
//  Display layout (16 chars per line):
//  ??????????????????
//  ?P1:FREE  P2:OCC ?   ? Line 1
//  ?P3:FREE         ?   ? Line 2
//  ??????????????????
//
//  Slot status strings: "FREE" (empty slot) / "OCC " (occupied)
//
//  The module:
//    1. Executes the HD44780 initialisation sequence after reset.
//    2. Writes the 32-character frame to DDRAM (lines 1 and 2).
//    3. Idles, watching slot_status for any bit change.
//    4. On any change: latches the new status and re-writes
//       the full display.
//
//  HD44780 8-bit write cycle (per byte):
//    ? Set RS, place data on bus
//    ? Assert E = 1 for ? 450 ns
//    ? Deassert E = 0
//    ? Wait for LCD to process: ? 37 µs (or 1.52 ms for Clear)
//
//  PARAMETERS (override for FPGA clock or simulation speedup)
//  ----------------------------------------------------------
//  CLK_FREQ  : system clock in Hz           (default 50 MHz)
//  T_PWRON   : power-on delay ticks         (default 2 500 000 = 50 ms)
//  T_EHIGH   : E-pulse high duration ticks  (default 25 = 500 ns)
//  T_CMD     : standard command settle tick (default 2 000 = 40 µs)
//  T_CLR     : clear-display settle ticks   (default 80 000 = 1.6 ms)
//
//  PORTS
//  -----
//  slot_status [2:0] : from M3 Threshold Comparator
//                      bit 0 = slot 1 (1=occupied, 0=free)
//                      bit 1 = slot 2
//                      bit 2 = slot 3
//  lcd_rs     : Register Select (0=command, 1=data)
//  lcd_rw     : Read/Write - tied LOW (write-only)
//  lcd_e      : Enable - pulse to latch byte
//  lcd_data   : 8-bit data bus (D0-D7)
// ============================================================
`timescale 1ns / 1ps

module m4_lcd_top #(
    parameter CLK_FREQ = 50_000_000,
    parameter T_PWRON  = 2_500_000,   // 50 ms power-on delay
    parameter T_EHIGH  = 25,           // 500 ns  E-pulse high
    parameter T_CMD    = 2_000,        // 40 µs   standard command settle
    parameter T_CLR    = 80_000        // 1.6 ms  clear display settle
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [2:0] slot_status,     // [0]=slot1, [1]=slot2, [2]=slot3

    // ---------- HD44780 LCD Interface (8-bit) ----------
    output reg        lcd_rs,
    output wire       lcd_rw,
    output reg        lcd_e,
    output reg  [7:0] lcd_data
);

    // RW always LOW - we never read back from the LCD
    assign lcd_rw = 1'b0;

    // ------------------------------------------------------------------
    // Initialisation Command ROM  (5 bytes, sent at startup)
    // ------------------------------------------------------------------
    // cmd_rom[i][8] = 1 ? use T_CLR wait after this command
    //               = 0 ? use T_CMD wait
    reg [8:0] init_rom [0:4];   // bit[8]=long_wait, bits[7:0]=command
    initial begin
        init_rom[0] = {1'b0, 8'h38}; // Function Set : 8-bit, 2-line, 5×8
        init_rom[1] = {1'b0, 8'h38}; // Function Set : repeat (HD44780 spec)
        init_rom[2] = {1'b0, 8'h0C}; // Display ON   : cursor off, blink off
        init_rom[3] = {1'b1, 8'h01}; // Clear Display: needs long wait (1.6 ms)
        init_rom[4] = {1'b0, 8'h06}; // Entry Mode   : auto-increment, no shift
    end

    // ------------------------------------------------------------------
    // Display Content Function
    // Returns the ASCII character to display at position idx (0..31)
    // for a given slot_status byte.
    //
    // idx  0..15 ? Line 1 : "P1:XXXX P2:XXXX "
    // idx 16..31 ? Line 2 : "P3:XXXX         "
    //
    // "FREE" = 8'h46, 8'h52, 8'h45, 8'h45
    // "OCC " = 8'h4F, 8'h43, 8'h43, 8'h20
    // ------------------------------------------------------------------
    function [7:0] disp_char;
        input [4:0] idx;
        input [2:0] status;
        reg [31:0] s1, s2, s3;
        begin
            // Select 4-char string per slot
            s1 = status[0] ? 32'h4F434320 : 32'h46524545;  // "OCC " : "FREE"
            s2 = status[1] ? 32'h4F434320 : 32'h46524545;
            s3 = status[2] ? 32'h4F434320 : 32'h46524545;

            // P1:FREE P2:OCC  = 16 chars (Line 1)
            case (idx)
                5'd0:  disp_char = "P";
                5'd1:  disp_char = "1";
                5'd2:  disp_char = ":";
                5'd3:  disp_char = s1[31:24];   // s1 char 0
                5'd4:  disp_char = s1[23:16];   // s1 char 1
                5'd5:  disp_char = s1[15: 8];   // s1 char 2
                5'd6:  disp_char = s1[ 7: 0];   // s1 char 3
                5'd7:  disp_char = " ";
                5'd8:  disp_char = "P";
                5'd9:  disp_char = "2";
                5'd10: disp_char = ":";
                5'd11: disp_char = s2[31:24];
                5'd12: disp_char = s2[23:16];
                5'd13: disp_char = s2[15: 8];
                5'd14: disp_char = s2[ 7: 0];
                5'd15: disp_char = " ";
                // P3:FREE         = 16 chars (Line 2)
                5'd16: disp_char = "P";
                5'd17: disp_char = "3";
                5'd18: disp_char = ":";
                5'd19: disp_char = s3[31:24];
                5'd20: disp_char = s3[23:16];
                5'd21: disp_char = s3[15: 8];
                5'd22: disp_char = s3[ 7: 0];
                // Remaining 9 chars of line 2: spaces
                default: disp_char = " ";
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // FSM State Encoding
    // ------------------------------------------------------------------
    localparam [3:0]
        S_PWRON      = 4'd0,   // power-on delay
        S_INIT_SETUP = 4'd1,   // put init command on bus
        S_INIT_WAIT  = 4'd2,   // drive E, then wait command settle
        S_LATCH      = 4'd3,   // latch slot_status, prepare write
        S_ADDR_SETUP = 4'd4,   // put DDRAM address command on bus
        S_ADDR_WAIT  = 4'd5,   // drive E, then wait settle
        S_DATA_SETUP = 4'd6,   // put character data on bus
        S_DATA_WAIT  = 4'd7,   // drive E, then wait settle
        S_IDLE       = 4'd8;   // monitor for slot_status change

    reg [3:0]  state;
    reg [21:0] cnt;            // general wait counter
    reg [2:0]  init_step;      // 0..4 index into init_rom
    reg [4:0]  char_idx;       // 0..31 current character to write
    reg        line2;          // 0=writing line 1, 1=writing line 2
    reg [2:0]  slot_lat;       // latched status at start of each write cycle
    reg [2:0]  slot_prev;      // last written status - detect changes

    // ------------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_PWRON;
            cnt       <= 22'd0;
            init_step <= 3'd0;
            char_idx  <= 5'd0;
            line2     <= 1'b0;
            slot_lat  <= 3'b000;
            slot_prev <= 3'b111;  // force an initial write after init
            lcd_rs    <= 1'b0;
            lcd_e     <= 1'b0;
            lcd_data  <= 8'h00;
        end else begin

            case (state)

                // -------------------------------------------------------
                // PWRON: wait T_PWRON ticks (50 ms) after power-up
                // -------------------------------------------------------
                S_PWRON: begin
                    lcd_e <= 1'b0;
                    if (cnt == T_PWRON - 1) begin
                        cnt       <= 22'd0;
                        init_step <= 3'd0;
                        state     <= S_INIT_SETUP;
                    end else
                        cnt <= cnt + 1'b1;
                end

                // -------------------------------------------------------
                // INIT_SETUP: place command byte on bus (RS=0)
                // -------------------------------------------------------
                S_INIT_SETUP: begin
                    lcd_rs   <= 1'b0;
                    lcd_data <= init_rom[init_step][7:0];
                    cnt      <= 22'd0;
                    state    <= S_INIT_WAIT;
                end

                // -------------------------------------------------------
                // INIT_WAIT: pulse E high for T_EHIGH, then wait settle
                // -------------------------------------------------------
                S_INIT_WAIT: begin
                    // E high for first T_EHIGH ticks, then low
                    lcd_e <= (cnt < T_EHIGH) ? 1'b1 : 1'b0;

                    // Wait the correct settle time (long for Clear Display)
                    if (cnt == (init_rom[init_step][8] ? T_CLR : T_CMD) - 1) begin
                        cnt <= 22'd0;
                        if (init_step == 3'd4) begin
                            // All init commands sent
                            state <= S_LATCH;
                        end else begin
                            init_step <= init_step + 1'b1;
                            state     <= S_INIT_SETUP;
                        end
                    end else
                        cnt <= cnt + 1'b1;
                end

                // -------------------------------------------------------
                // LATCH: capture current slot_status, start write
                // -------------------------------------------------------
                S_LATCH: begin
                    slot_lat  <= slot_status;
                    slot_prev <= slot_status;
                    line2     <= 1'b0;
                    char_idx  <= 5'd0;
                    cnt       <= 22'd0;
                    state     <= S_ADDR_SETUP;
                end

                // -------------------------------------------------------
                // ADDR_SETUP: send DDRAM address command
                //   Line 1 ? 0x80 (address 0x00)
                //   Line 2 ? 0xC0 (address 0x40)
                // -------------------------------------------------------
                S_ADDR_SETUP: begin
                    lcd_rs   <= 1'b0;   // command
                    lcd_data <= line2 ? 8'hC0 : 8'h80;
                    cnt      <= 22'd0;
                    state    <= S_ADDR_WAIT;
                end

                // -------------------------------------------------------
                // ADDR_WAIT: pulse E and wait settle
                // -------------------------------------------------------
                S_ADDR_WAIT: begin
                    lcd_e <= (cnt < T_EHIGH) ? 1'b1 : 1'b0;
                    if (cnt == T_CMD - 1) begin
                        cnt      <= 22'd0;
                        char_idx <= line2 ? 5'd16 : 5'd0;  // set start index
                        state    <= S_DATA_SETUP;
                    end else
                        cnt <= cnt + 1'b1;
                end

                // -------------------------------------------------------
                // DATA_SETUP: place character byte on bus (RS=1)
                // -------------------------------------------------------
                S_DATA_SETUP: begin
                    lcd_rs   <= 1'b1;   // data
                    lcd_data <= disp_char(char_idx, slot_lat);
                    cnt      <= 22'd0;
                    state    <= S_DATA_WAIT;
                end

                // -------------------------------------------------------
                // DATA_WAIT: pulse E and wait settle, then advance
                // -------------------------------------------------------
                S_DATA_WAIT: begin
                    lcd_e <= (cnt < T_EHIGH) ? 1'b1 : 1'b0;
                    if (cnt == T_CMD - 1) begin
                        cnt <= 22'd0;
                        if (char_idx == (line2 ? 5'd31 : 5'd15)) begin
                            // End of this line
                            if (!line2) begin
                                // Finished line 1 - switch to line 2
                                line2 <= 1'b1;
                                state <= S_ADDR_SETUP;
                            end else begin
                                // Finished line 2 - full frame written
                                state <= S_IDLE;
                            end
                        end else begin
                            char_idx <= char_idx + 1'b1;
                            state    <= S_DATA_SETUP;
                        end
                    end else
                        cnt <= cnt + 1'b1;
                end

                // -------------------------------------------------------
                // IDLE: watch for slot_status change; re-display if needed
                // -------------------------------------------------------
                S_IDLE: begin
                    lcd_e <= 1'b0;
                    if (slot_status !== slot_prev)
                        state <= S_LATCH;
                end

                default: state <= S_PWRON;

            endcase
        end
    end

endmodule
