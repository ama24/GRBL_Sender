#include <Arduino.h>

// ── Serial port assignment ─────────────────────────────────────────────────
// ESP32-S3-DevKitC-1 physical USB connectors:
//
//   Label  │ Chip      │ UART / GPIO              │ Arduino object
//   ───────┼───────────┼──────────────────────────┼───────────────
//   "COM"  │ CP2102N   │ UART0  GPIO43(TX)/44(RX) │ Serial0
//   "USB"  │ Native OTG│ GPIO19(D-) / GPIO20(D+)  │ Serial  ← CDC-on-boot
//
// With ARDUINO_USB_CDC_ON_BOOT=1 (platformio.ini):
//   Serial  → native USB CDC ("USB" connector)  — debug terminal
//   Serial0 → UART0 hardware  ("COM" connector) — CNC machine
//
// ⚠ Serial1 default pins are GPIO17(TX)/GPIO18(RX).
//   GPIO18 = USB_SEL on this board.  Do NOT use Serial1 at default pins.
#define CNC_SERIAL  Serial0   // UART0, GPIO43(TX)/GPIO44(RX), "COM" connector
#define DBG_SERIAL  Serial    // Native USB CDC, GPIO19/20,    "USB" connector

// ── Baud rates ────────────────────────────────────────────────────────────
#define CNC_BAUD    460800
#define DBG_BAUD    115200

// ── Pin ───────────────────────────────────────────────────────────────────
#define FIRE_PIN    4   // INPUT_PULLUP; pull LOW to fire laser

// ── Init-sequence delays (ms) ────────────────────────────────────────────
#define INIT_DELAY_SOFTRESET_MS 2500   // time for GRBL to reinit after 0x18
#define INIT_DELAY_STATUS_MS    500
#define INIT_DELAY_UNLOCK_MS    500
#define INIT_DELAY_SETTINGS_MS  1500
#define INIT_DELAY_SPINDLE_MS   500

// ── Debug CDC wait (ms) ───────────────────────────────────────────────────
// How long setup() will block waiting for the USB CDC terminal to be opened
// before proceeding. Lets init log messages appear in the terminal instead
// of being silently dropped because the port wasn't open yet.
#define CDC_CONNECT_TIMEOUT_MS  5000

// ── Debounce window (ms) ──────────────────────────────────────────────────
#define DEBOUNCE_MS  50

// ── Global state ──────────────────────────────────────────────────────────
static int           g_firePinLast   = HIGH;
static unsigned long g_debounceTs    = 0;
static bool          g_debouncing    = false;
static String        g_dbgLineBuffer = "";

// ─────────────────────────────────────────────────────────────────────────
// sendCommand — single exit point for all traffic toward the CNC machine.
//
// GRBL real-time commands (?, ~, !, 0x18) are sent as bare bytes with no
// line terminator.  Every other command gets \r\n appended.  Either way the
// transmission is echoed to the debug terminal with a "TX: " prefix.
// ─────────────────────────────────────────────────────────────────────────
void sendCommand(const String& cmd) {
    bool isRealtime = (cmd.length() == 1 &&
                       (cmd[0] == '?'  ||
                        cmd[0] == '~'  ||
                        cmd[0] == '!'  ||
                        cmd[0] == 0x18));

    if (isRealtime) {
        CNC_SERIAL.write((uint8_t)cmd[0]);
        DBG_SERIAL.print("TX: [");
        DBG_SERIAL.print(cmd);
        DBG_SERIAL.println("]");
    } else {
        CNC_SERIAL.print(cmd);
        CNC_SERIAL.print("\r\n");
        DBG_SERIAL.print("TX: ");
        DBG_SERIAL.println(cmd);
    }
}

// ─────────────────────────────────────────────────────────────────────────
// drainCNC — blocking read of the CNC response, used only during initGRBL.
//
// Exits early once CNC_SERIAL has been silent for 100 ms, but never waits
// longer than timeout_ms total.  Every received byte is echoed to DBG_SERIAL.
// ─────────────────────────────────────────────────────────────────────────
void drainCNC(unsigned long timeout_ms) {
    const unsigned long TAIL_SILENCE_MS = 100;
    unsigned long startTs    = millis();
    unsigned long lastByteTs = millis();

    while (true) {
        unsigned long now = millis();

        if ((now - startTs) >= timeout_ms) break;

        if (CNC_SERIAL.available()) {
            DBG_SERIAL.write((uint8_t)CNC_SERIAL.read());
            lastByteTs = now;
        } else if ((now - lastByteTs) >= TAIL_SILENCE_MS &&
                   (now - startTs)   >= TAIL_SILENCE_MS) {
            break;
        }
    }
    DBG_SERIAL.println();
}

// ─────────────────────────────────────────────────────────────────────────
// initGRBL — run once during setup to establish communication with GRBL.
//
// Sequence:  ? (status)  →  $$ (dump settings)  →  $32=0 (spindle mode).
// ─────────────────────────────────────────────────────────────────────────
void initGRBL() {
    // 1. Soft reset — clears any garbage the GRBL parser may have received
    //    from the ESP32 ROM bootloader (which sends noise on UART0 at 115200
    //    before user code starts). After 0x18, GRBL reinitialises fully.
    DBG_SERIAL.println("[INIT] Soft reset (0x18) — clearing boot garbage...");
    CNC_SERIAL.write((uint8_t)0x18);
    drainCNC(INIT_DELAY_SOFTRESET_MS);  // wait for GRBL to print its banner

    // Flush RX now that GRBL has settled
    while (CNC_SERIAL.available()) CNC_SERIAL.read();

    // 2. Query status — GRBL often boots into ALARM state
    DBG_SERIAL.println("[INIT] Querying status...");
    sendCommand("?");
    drainCNC(INIT_DELAY_STATUS_MS);

    // 3. Clear alarm lock — mandatory before any G-code will be accepted.
    //    In ALARM state GRBL silently drops M3/G-code; $X unlocks it to Idle.
    DBG_SERIAL.println("[INIT] Clearing alarm lock ($X)...");
    sendCommand("$X");
    drainCNC(INIT_DELAY_UNLOCK_MS);

    // 3. Dump all settings — informational, response can be ignored
    DBG_SERIAL.println("[INIT] Reading settings ($$)...");
    sendCommand("$$");
    drainCNC(INIT_DELAY_SETTINGS_MS);

    // 4. Spindle mode: $32=0 — laser fires at constant duty without motion.
    //    ($32=1 = laser mode, only fires while axis is moving — not what we want)
    DBG_SERIAL.println("[INIT] Setting spindle mode ($32=0)...");
    sendCommand("$32=0");
    drainCNC(INIT_DELAY_SPINDLE_MS);

    DBG_SERIAL.println("[INIT] Init complete — machine should be Idle and ready.");
}

// ─────────────────────────────────────────────────────────────────────────
// handleFirePin — polls GPIO 4 for a falling edge and fires M3 S1000.
//
// Uses a software debounce state machine: a transition is only accepted
// after the pin has been stable LOW for DEBOUNCE_MS milliseconds.
// ─────────────────────────────────────────────────────────────────────────
void handleFirePin() {
    int reading = digitalRead(FIRE_PIN);
    unsigned long now = millis();

    if (!g_debouncing) {
        if (reading == LOW && g_firePinLast == HIGH) {
            g_debouncing = true;
            g_debounceTs = now;
        }
    } else {
        if ((now - g_debounceTs) >= DEBOUNCE_MS) {
            g_debouncing = false;
            if (reading == LOW) {
                DBG_SERIAL.println("[FIRE] Trigger on pin 4 — firing laser!");
                sendCommand("M3 S1000");
            }
        }
    }

    g_firePinLast = reading;
}

// ─────────────────────────────────────────────────────────────────────────
// handleCNCOutput — non-blocking echo of CNC responses to the debug terminal.
// ─────────────────────────────────────────────────────────────────────────
void handleCNCOutput() {
    while (CNC_SERIAL.available()) {
        DBG_SERIAL.write((uint8_t)CNC_SERIAL.read());
    }
}

// ─────────────────────────────────────────────────────────────────────────
// handleDebugInput — accumulates lines from the debug terminal and forwards
// each complete line to the CNC machine via sendCommand.
// ─────────────────────────────────────────────────────────────────────────
void handleDebugInput() {
    while (DBG_SERIAL.available()) {
        char c = (char)DBG_SERIAL.read();

        if (c == '\n' || c == '\r') {
            if (g_dbgLineBuffer.length() > 0) {
                sendCommand(g_dbgLineBuffer);
                g_dbgLineBuffer = "";
            }
        } else {
            g_dbgLineBuffer += c;

            if (g_dbgLineBuffer.length() > 256) {
                DBG_SERIAL.println("[WARN] Input buffer overflow — discarding line.");
                g_dbgLineBuffer = "";
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
void setup() {
    DBG_SERIAL.begin(DBG_BAUD);

    // USB CDC (Serial) needs to enumerate and the host terminal needs to open
    // the COM port before any println() is visible. We block here up to
    // CDC_CONNECT_TIMEOUT_MS so the init sequence messages are not lost.
    // If no terminal connects in time the code proceeds anyway (headless).
    {
        unsigned long t0 = millis();
        while (!DBG_SERIAL && (millis() - t0) < CDC_CONNECT_TIMEOUT_MS) {
            delay(10);
        }
    }
    delay(100);

    DBG_SERIAL.println("\n[BOOT] GRBL Sender starting...");
    DBG_SERIAL.print("[BOOT] CNC baud: ");
    DBG_SERIAL.println(CNC_BAUD);

    // GRBL uses 8 data bits, no parity, 1 stop bit (8N1) — make it explicit
    CNC_SERIAL.begin(CNC_BAUD, SERIAL_8N1);
    DBG_SERIAL.println("[BOOT] CNC  → Serial0 (UART0) GPIO43(TX)/GPIO44(RX), 8N1 @ " + String(CNC_BAUD) + " baud  [\"COM\" connector]");
    DBG_SERIAL.println("[BOOT] DBG  → Serial  (USB CDC) GPIO19/GPIO20              [\"USB\" connector]  ← you are here");

    pinMode(FIRE_PIN, INPUT_PULLUP);
    g_firePinLast = digitalRead(FIRE_PIN);
    DBG_SERIAL.print("[BOOT] Fire pin ");
    DBG_SERIAL.print(FIRE_PIN);
    DBG_SERIAL.println(" configured (INPUT_PULLUP).");

    initGRBL();

    DBG_SERIAL.println("[BOOT] Ready. Type G-code to forward to CNC.");
}

// ─────────────────────────────────────────────────────────────────────────
void loop() {
    handleFirePin();
    handleCNCOutput();
    handleDebugInput();
}
