#include <Arduino.h>

// ── Serial port assignment ─────────────────────────────────────────────────
// ARDUINO_USB_CDC_ON_BOOT=1 (set in platformio.ini) remaps Serial to the
// native USB-OTG port ("USB" USB-C connector).  UART0 stays accessible as
// Serial0 and is wired through the onboard CP2102 to the "UART" USB-C port.
#define CNC_SERIAL  Serial0   // UART0 → CP2102 → "UART" USB-C → CNC machine
#define DBG_SERIAL  Serial    // Native USB CDC  → "USB"  USB-C → PC terminal

// ── Baud rates ────────────────────────────────────────────────────────────
#define CNC_BAUD    460800
#define DBG_BAUD    115200

// ── Pin ───────────────────────────────────────────────────────────────────
#define FIRE_PIN    4   // INPUT_PULLUP; pull LOW to fire laser

// ── Init-sequence delays (ms) ────────────────────────────────────────────
#define INIT_DELAY_STATUS_MS    500
#define INIT_DELAY_SETTINGS_MS  1000
#define INIT_DELAY_SPINDLE_MS   500

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
    // Flush any junk in the RX FIFO before starting
    while (CNC_SERIAL.available()) CNC_SERIAL.read();

    DBG_SERIAL.println("[INIT] Sending status query...");
    sendCommand("?");
    drainCNC(INIT_DELAY_STATUS_MS);

    DBG_SERIAL.println("[INIT] Sending settings query...");
    sendCommand("$$");
    drainCNC(INIT_DELAY_SETTINGS_MS);

    DBG_SERIAL.println("[INIT] Setting spindle mode ($32=0)...");
    sendCommand("$32=0");
    drainCNC(INIT_DELAY_SPINDLE_MS);

    DBG_SERIAL.println("[INIT] Init sequence complete.");
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
    delay(100);
    DBG_SERIAL.println("\n[BOOT] GRBL Sender starting...");
    DBG_SERIAL.print("[BOOT] CNC baud: ");
    DBG_SERIAL.println(CNC_BAUD);

    CNC_SERIAL.begin(CNC_BAUD);
    DBG_SERIAL.println("[BOOT] CNC serial initialized.");

    pinMode(FIRE_PIN, INPUT_PULLUP);
    g_firePinLast = digitalRead(FIRE_PIN);
    DBG_SERIAL.print("[BOOT] Fire pin ");
    DBG_SERIAL.print(FIRE_PIN);
    DBG_SERIAL.println(" configured (INPUT_PULLUP).");

    DBG_SERIAL.println("[BOOT] Waiting for CNC controller...");
    delay(2000);

    initGRBL();

    DBG_SERIAL.println("[BOOT] Ready. Type G-code to forward to CNC.");
}

// ─────────────────────────────────────────────────────────────────────────
void loop() {
    handleFirePin();
    handleCNCOutput();
    handleDebugInput();
}
