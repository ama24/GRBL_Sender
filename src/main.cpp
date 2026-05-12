/*
 * GRBL Sender — USB Host CDC-ACM edition
 *
 * Architecture:
 *   usbHostTask   — drives usb_host_lib_handle_events()
 *   usbClientTask — enumerates device, sets up CDC, drains TX queue, receives data
 *   loop()        — fire pin + debug UART, pushes TX items into s_txQueue
 *
 * Debug output goes to Serial0 (UART0, GPIO43 TX / GPIO44 RX) at 115200.
 * No blocking wait for a terminal — prints are fire-and-forget.
 *
 * Connect the single USB OTG port of the ESP32-S3 to the CNC board's USB port.
 */

#include <Arduino.h>
extern "C" {
#include "usb/usb_host.h"
#include "usb/usb_helpers.h"
}
#include "freertos/semphr.h"
#include "freertos/queue.h"

// ── Debug serial ──────────────────────────────────────────────────────────
// Without ARDUINO_USB_CDC_ON_BOOT, Serial = UART0 (GPIO43 TX / GPIO44 RX).
#define DBG_SERIAL  Serial
#define DBG_BAUD    115200

// ── Fire pin ──────────────────────────────────────────────────────────────
#define FIRE_PIN    4
#define DEBOUNCE_MS 50

// ── GRBL init delays (ms) ─────────────────────────────────────────────────
#define INIT_DELAY_SOFTRESET_MS 2500
#define INIT_DELAY_STATUS_MS     500
#define INIT_DELAY_UNLOCK_MS     500
#define INIT_DELAY_SETTINGS_MS  1500
#define INIT_DELAY_SPINDLE_MS    500

// ── USB / CDC constants ───────────────────────────────────────────────────
// USB_CLASS_COMM (0x02) and USB_CLASS_CDC_DATA (0x0a) come from usb_types_ch9.h
#define USB_EP_ATTR_BULK        0x02
#define USB_EP_DIR_IN           0x80
#define CDC_SET_LINE_CODING     0x20
#define CDC_SET_CTRL_LINE_STATE 0x22
#define CNC_BAUD                460800U

// ── Transfer buffer sizes ─────────────────────────────────────────────────
#define CTRL_BUF_SIZE    64
#define BULK_IN_BUF_SIZE 512
#define BULK_OUT_BUF_SIZE 512

// ── TX queue ──────────────────────────────────────────────────────────────
#define TX_QUEUE_LEN  16
#define TX_CMD_MAX    64   // max bytes per G-code command

typedef struct { uint8_t data[TX_CMD_MAX]; size_t len; } tx_item_t;

// ── RX ring buffer (producer: usbClientTask, consumer: loop()) ────────────
#define RX_RING_SIZE 1024
static uint8_t         s_rxRing[RX_RING_SIZE];
static volatile size_t s_rxHead = 0;
static volatile size_t s_rxTail = 0;
static portMUX_TYPE    s_rxMux  = portMUX_INITIALIZER_UNLOCKED;

// ── USB state ─────────────────────────────────────────────────────────────
static usb_host_client_handle_t s_client    = NULL;
static usb_device_handle_t      s_devHdl    = NULL;
static usb_transfer_t          *s_ctrlXfer  = NULL;
static usb_transfer_t          *s_bulkInXfer  = NULL;
static usb_transfer_t          *s_bulkOutXfer = NULL;
static uint8_t   s_bulkInEp  = 0;
static uint8_t   s_bulkOutEp = 0;
static uint16_t  s_bulkInMps = 64;
static uint8_t   s_dataIntf  = 0;
static uint8_t   s_commIntf  = 0xFF;  // 0xFF = no CDC comm interface

static volatile bool    s_cncReady  = false;
static volatile uint8_t s_newDevAddr = 0;
static volatile bool    s_devGone   = false;

// ── Synchronisation ───────────────────────────────────────────────────────
static SemaphoreHandle_t s_ctrlSem = NULL;
static SemaphoreHandle_t s_txSem   = NULL;
static QueueHandle_t     s_txQueue = NULL;

// ── Debug line buffer (loop() context) ───────────────────────────────────
static String g_dbgLine = "";

// ── Fire pin state ────────────────────────────────────────────────────────
static int           g_fireLast   = HIGH;
static bool          g_debouncing = false;
static int           g_pendEdge   = HIGH;
static unsigned long g_debounceTs = 0;

// ─────────────────────────────────────────────────────────────────────────
// rxPush — called from usbClientTask via bulkInCallback
// ─────────────────────────────────────────────────────────────────────────
static void rxPush(const uint8_t *data, size_t len) {
    portENTER_CRITICAL(&s_rxMux);
    for (size_t i = 0; i < len; i++) {
        size_t next = (s_rxHead + 1) % RX_RING_SIZE;
        if (next != s_rxTail) {
            s_rxRing[s_rxHead] = data[i];
            s_rxHead = next;
        }
    }
    portEXIT_CRITICAL(&s_rxMux);
}

// ─────────────────────────────────────────────────────────────────────────
// Transfer callbacks — called from usbClientTask via handle_events()
// ─────────────────────────────────────────────────────────────────────────
static void ctrlCallback(usb_transfer_t *xfer) {
    xSemaphoreGive(s_ctrlSem);
}

static void bulkInCallback(usb_transfer_t *xfer) {
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED && xfer->actual_num_bytes > 0) {
        rxPush(xfer->data_buffer, (size_t)xfer->actual_num_bytes);
    }
    if (s_cncReady && s_devHdl) {
        xfer->num_bytes = usb_round_up_to_mps(BULK_IN_BUF_SIZE, s_bulkInMps);
        usb_host_transfer_submit(xfer);
    }
}

static void bulkOutCallback(usb_transfer_t *xfer) {
    xSemaphoreGive(s_txSem);
}

// ─────────────────────────────────────────────────────────────────────────
// submitCtrl — submit a control transfer and wait for it to complete.
// Must only be called from usbClientTask.
// ─────────────────────────────────────────────────────────────────────────
static esp_err_t submitCtrl(void) {
    xSemaphoreTake(s_ctrlSem, 0);
    esp_err_t err = usb_host_transfer_submit_control(s_client, s_ctrlXfer);
    if (err != ESP_OK) return err;
    while (xSemaphoreTake(s_ctrlSem, pdMS_TO_TICKS(20)) != pdTRUE) {
        usb_host_client_handle_events(s_client, pdMS_TO_TICKS(20));
    }
    return (s_ctrlXfer->status == USB_TRANSFER_STATUS_COMPLETED) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

// ─────────────────────────────────────────────────────────────────────────
// findBulkEndpoints — walk raw config descriptor bytes.
// Prefers CDC Data class interfaces; falls back to any interface with
// one bulk IN + one bulk OUT endpoint.
// ─────────────────────────────────────────────────────────────────────────
static bool findBulkEndpoints(const usb_config_desc_t *cfg) {
    const uint8_t *buf = (const uint8_t *)cfg;
    const uint8_t *end = buf + cfg->wTotalLength;

    uint8_t curClass = 0, curIntfNum = 0;
    uint8_t bestIn = 0, bestOut = 0, bestIntf = 0;
    uint16_t bestInMps = 64;
    bool foundCDCData = false;
    uint8_t commIntf = 0xFF;

    while (buf < end) {
        uint8_t dlen  = buf[0];
        uint8_t dtype = buf[1];
        if (dlen < 2 || buf + dlen > end) break;

        if (dtype == 0x04 && dlen >= 9) {  // INTERFACE
            curIntfNum = buf[2];
            curClass   = buf[5];
            if (curClass == USB_CLASS_COMM) commIntf = curIntfNum;
        }

        if (dtype == 0x05 && dlen >= 7) {  // ENDPOINT
            uint8_t  addr  = buf[2];
            uint8_t  attrs = buf[3];
            uint16_t mps   = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

            if ((attrs & 0x03) == USB_EP_ATTR_BULK) {
                bool prefer = (curClass == USB_CLASS_CDC_DATA) || (!foundCDCData);
                if (prefer) {
                    if (addr & USB_EP_DIR_IN) {
                        bestIn    = addr;
                        bestInMps = mps;
                        bestIntf  = curIntfNum;
                        if (curClass == USB_CLASS_CDC_DATA) foundCDCData = true;
                    } else {
                        bestOut  = addr;
                        bestIntf = curIntfNum;
                        if (curClass == USB_CLASS_CDC_DATA) foundCDCData = true;
                    }
                }
            }
        }
        buf += dlen;
    }

    if (!bestIn || !bestOut) return false;

    s_bulkInEp  = bestIn;
    s_bulkOutEp = bestOut;
    s_bulkInMps = bestInMps;
    s_dataIntf  = bestIntf;
    s_commIntf  = commIntf;
    return true;
}

// ─────────────────────────────────────────────────────────────────────────
// setupCDC — send SET_LINE_CODING + SET_CONTROL_LINE_STATE.
// Must only be called from usbClientTask.
// ─────────────────────────────────────────────────────────────────────────
static void setupCDC(void) {
    uint8_t ctrlIntf = (s_commIntf != 0xFF) ? s_commIntf : s_dataIntf;

    // SET_LINE_CODING (7-byte payload: 460800 baud, 1 stop bit, no parity, 8 data bits)
    usb_setup_packet_t *sp = (usb_setup_packet_t *)s_ctrlXfer->data_buffer;
    sp->bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT |
                        USB_BM_REQUEST_TYPE_TYPE_CLASS |
                        USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
    sp->bRequest  = CDC_SET_LINE_CODING;
    sp->wValue    = 0;
    sp->wIndex    = ctrlIntf;
    sp->wLength   = 7;

    uint8_t *lc = s_ctrlXfer->data_buffer + USB_SETUP_PACKET_SIZE;
    uint32_t baud = CNC_BAUD;
    lc[0] = (uint8_t)(baud);
    lc[1] = (uint8_t)(baud >> 8);
    lc[2] = (uint8_t)(baud >> 16);
    lc[3] = (uint8_t)(baud >> 24);
    lc[4] = 0;   // 1 stop bit
    lc[5] = 0;   // no parity
    lc[6] = 8;   // 8 data bits

    s_ctrlXfer->num_bytes        = USB_SETUP_PACKET_SIZE + 7;
    s_ctrlXfer->device_handle    = s_devHdl;
    s_ctrlXfer->bEndpointAddress = 0;
    s_ctrlXfer->callback         = ctrlCallback;
    s_ctrlXfer->context          = NULL;
    s_ctrlXfer->timeout_ms       = 1000;

    esp_err_t err = submitCtrl();
    if (err != ESP_OK) DBG_SERIAL.println("[USB] SET_LINE_CODING failed (non-CDC device?)");

    // SET_CONTROL_LINE_STATE (DTR=1, RTS=1, no data stage)
    sp->bRequest = CDC_SET_CTRL_LINE_STATE;
    sp->wValue   = 0x0003;
    sp->wLength  = 0;
    s_ctrlXfer->num_bytes = USB_SETUP_PACKET_SIZE;
    err = submitCtrl();
    if (err != ESP_OK) DBG_SERIAL.println("[USB] SET_CTRL_LINE_STATE failed");
}

// ─────────────────────────────────────────────────────────────────────────
// initSendCommand — blocking send during GRBL init sequence.
// Must only be called from usbClientTask.
// ─────────────────────────────────────────────────────────────────────────
static void initSendCommand(const char *cmd, size_t len, bool realtime) {
    uint8_t *buf = s_bulkOutXfer->data_buffer;
    memcpy(buf, cmd, len);
    size_t total = len;
    if (!realtime) { buf[total++] = '\r'; buf[total++] = '\n'; }

    s_bulkOutXfer->num_bytes        = (int)total;
    s_bulkOutXfer->device_handle    = s_devHdl;
    s_bulkOutXfer->bEndpointAddress = s_bulkOutEp;
    s_bulkOutXfer->callback         = bulkOutCallback;
    s_bulkOutXfer->context          = NULL;
    s_bulkOutXfer->timeout_ms       = 1000;

    xSemaphoreTake(s_txSem, 0);
    esp_err_t err = usb_host_transfer_submit(s_bulkOutXfer);
    if (err != ESP_OK) { DBG_SERIAL.print("[ERR] TX: "); DBG_SERIAL.println(esp_err_to_name(err)); return; }
    while (xSemaphoreTake(s_txSem, pdMS_TO_TICKS(20)) != pdTRUE) {
        usb_host_client_handle_events(s_client, pdMS_TO_TICKS(20));
    }
    DBG_SERIAL.print("TX: "); DBG_SERIAL.write((const uint8_t*)cmd, len); DBG_SERIAL.println();
}

// ─────────────────────────────────────────────────────────────────────────
// drainCNC — wait for CNC responses during init.
// Must only be called from usbClientTask.
// ─────────────────────────────────────────────────────────────────────────
static void drainCNC(unsigned long timeout_ms) {
    const unsigned long SILENCE_MS = 100;
    unsigned long start    = millis();
    unsigned long lastByte = millis();

    while (millis() - start < timeout_ms) {
        usb_host_client_handle_events(s_client, pdMS_TO_TICKS(20));

        uint8_t tmp[64]; size_t n = 0;
        portENTER_CRITICAL(&s_rxMux);
        while (s_rxHead != s_rxTail && n < sizeof(tmp)) {
            tmp[n++] = s_rxRing[s_rxTail];
            s_rxTail = (s_rxTail + 1) % RX_RING_SIZE;
        }
        portEXIT_CRITICAL(&s_rxMux);

        if (n) { DBG_SERIAL.write(tmp, n); lastByte = millis(); }

        if (millis() - lastByte >= SILENCE_MS && millis() - start >= SILENCE_MS) break;
    }
    DBG_SERIAL.println();
}

// ─────────────────────────────────────────────────────────────────────────
// initGRBL — GRBL init sequence.
// Must only be called from usbClientTask after CDC is set up.
// ─────────────────────────────────────────────────────────────────────────
static void initGRBL(void) {
    DBG_SERIAL.println("[INIT] Soft reset (0x18)...");
    uint8_t rst = 0x18;
    initSendCommand((const char *)&rst, 1, true);
    drainCNC(INIT_DELAY_SOFTRESET_MS);

    // Flush RX ring
    portENTER_CRITICAL(&s_rxMux);
    s_rxTail = s_rxHead;
    portEXIT_CRITICAL(&s_rxMux);

    DBG_SERIAL.println("[INIT] Status query...");
    initSendCommand("?", 1, true);
    drainCNC(INIT_DELAY_STATUS_MS);

    DBG_SERIAL.println("[INIT] Clear alarm ($X)...");
    initSendCommand("$X", 2, false);
    drainCNC(INIT_DELAY_UNLOCK_MS);

    DBG_SERIAL.println("[INIT] Dump settings ($$)...");
    initSendCommand("$$", 2, false);
    drainCNC(INIT_DELAY_SETTINGS_MS);

    // $32=0: spindle fires at constant duty (not only while moving)
    DBG_SERIAL.println("[INIT] Spindle mode ($32=0)...");
    initSendCommand("$32=0", 5, false);
    drainCNC(INIT_DELAY_SPINDLE_MS);

    DBG_SERIAL.println("[INIT] Done. Machine should be Idle.");
}

// ─────────────────────────────────────────────────────────────────────────
// handleNewDev — open, parse, claim, set up CDC, start bulk IN, run init.
// Called from usbClientTask main loop.
// ─────────────────────────────────────────────────────────────────────────
static void handleNewDev(uint8_t addr) {
    DBG_SERIAL.printf("[USB] New device at addr %d\n", addr);

    if (usb_host_device_open(s_client, addr, &s_devHdl) != ESP_OK) {
        DBG_SERIAL.println("[USB] Failed to open device");
        s_devHdl = NULL;
        return;
    }

    const usb_config_desc_t *cfg;
    if (usb_host_get_active_config_descriptor(s_devHdl, &cfg) != ESP_OK || !cfg) {
        DBG_SERIAL.println("[USB] Failed to get config descriptor");
        usb_host_device_close(s_client, s_devHdl); s_devHdl = NULL;
        return;
    }

    if (!findBulkEndpoints(cfg)) {
        DBG_SERIAL.println("[USB] No usable bulk endpoints — not a CDC device");
        usb_host_device_close(s_client, s_devHdl); s_devHdl = NULL;
        return;
    }
    DBG_SERIAL.printf("[USB] Bulk IN=0x%02X OUT=0x%02X dataIntf=%d commIntf=%d MPS=%d\n",
                      s_bulkInEp, s_bulkOutEp, s_dataIntf, s_commIntf, s_bulkInMps);

    // Claim communication interface first (if present)
    if (s_commIntf != 0xFF) {
        usb_host_interface_claim(s_client, s_devHdl, s_commIntf, 0);
    }
    usb_host_interface_claim(s_client, s_devHdl, s_dataIntf, 0);

    // Allocate transfers if not already done
    if (!s_ctrlXfer)    usb_host_transfer_alloc(CTRL_BUF_SIZE, 0, &s_ctrlXfer);
    if (!s_bulkInXfer)  usb_host_transfer_alloc(BULK_IN_BUF_SIZE, 0, &s_bulkInXfer);
    if (!s_bulkOutXfer) usb_host_transfer_alloc(BULK_OUT_BUF_SIZE, 0, &s_bulkOutXfer);

    // Set line coding: 460800 baud, 8N1
    setupCDC();

    // Start async bulk IN
    s_bulkInXfer->num_bytes        = usb_round_up_to_mps(BULK_IN_BUF_SIZE, s_bulkInMps);
    s_bulkInXfer->device_handle    = s_devHdl;
    s_bulkInXfer->bEndpointAddress = s_bulkInEp;
    s_bulkInXfer->callback         = bulkInCallback;
    s_bulkInXfer->context          = NULL;
    s_bulkInXfer->timeout_ms       = 0;
    usb_host_transfer_submit(s_bulkInXfer);

    s_cncReady = true;
    DBG_SERIAL.println("[USB] Device ready. Running GRBL init...");

    initGRBL();
}

// ─────────────────────────────────────────────────────────────────────────
// handleDevGone
// ─────────────────────────────────────────────────────────────────────────
static void handleDevGone(void) {
    DBG_SERIAL.println("[USB] CNC disconnected.");
    s_cncReady = false;
    if (s_devHdl) {
        if (s_commIntf != 0xFF) usb_host_interface_release(s_client, s_devHdl, s_commIntf);
        usb_host_interface_release(s_client, s_devHdl, s_dataIntf);
        usb_host_device_close(s_client, s_devHdl);
        s_devHdl = NULL;
    }
}

// ─────────────────────────────────────────────────────────────────────────
// processTxQueue — drain TX queue and submit bulk OUT transfers.
// Must only be called from usbClientTask.
// ─────────────────────────────────────────────────────────────────────────
static void processTxQueue(void) {
    tx_item_t item;
    while (xQueueReceive(s_txQueue, &item, 0) == pdTRUE) {
        if (!s_devHdl || !s_cncReady) break;

        memcpy(s_bulkOutXfer->data_buffer, item.data, item.len);
        s_bulkOutXfer->num_bytes        = (int)item.len;
        s_bulkOutXfer->device_handle    = s_devHdl;
        s_bulkOutXfer->bEndpointAddress = s_bulkOutEp;
        s_bulkOutXfer->callback         = bulkOutCallback;
        s_bulkOutXfer->context          = NULL;
        s_bulkOutXfer->timeout_ms       = 500;

        xSemaphoreTake(s_txSem, 0);
        esp_err_t err = usb_host_transfer_submit(s_bulkOutXfer);
        if (err != ESP_OK) { DBG_SERIAL.print("[ERR] TX: "); DBG_SERIAL.println(esp_err_to_name(err)); continue; }
        while (xSemaphoreTake(s_txSem, pdMS_TO_TICKS(20)) != pdTRUE) {
            usb_host_client_handle_events(s_client, pdMS_TO_TICKS(20));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
// USB Host library event pump task
// ─────────────────────────────────────────────────────────────────────────
static void usbHostTask(void *arg) {
    while (true) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

// ─────────────────────────────────────────────────────────────────────────
// USB device event callback — called from inside handle_events()
// ─────────────────────────────────────────────────────────────────────────
static void usbClientEventCb(const usb_host_client_event_msg_t *msg, void *arg) {
    switch (msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:  s_newDevAddr = msg->new_dev.address; break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE: s_devGone    = true;                 break;
    }
}

// ─────────────────────────────────────────────────────────────────────────
// USB client task — owns all USB operations
// ─────────────────────────────────────────────────────────────────────────
static void usbClientTask(void *arg) {
    const usb_host_client_config_t cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = { .client_event_callback = usbClientEventCb, .callback_arg = NULL },
    };
    usb_host_client_register(&cfg, &s_client);
    DBG_SERIAL.println("[USB] Client registered. Waiting for CNC device...");

    while (true) {
        usb_host_client_handle_events(s_client, pdMS_TO_TICKS(100));

        if (s_devGone) {
            s_devGone = false;
            handleDevGone();
        }

        if (s_newDevAddr && !s_devHdl) {
            uint8_t addr = s_newDevAddr;
            s_newDevAddr = 0;
            handleNewDev(addr);
        }

        if (s_cncReady) processTxQueue();
    }
}

// ─────────────────────────────────────────────────────────────────────────
// sendCommand — queues a command from any task (loop() or usbClientTask).
// ─────────────────────────────────────────────────────────────────────────
void sendCommand(const String& cmd) {
    if (!s_cncReady) {
        DBG_SERIAL.println("[WARN] No CNC — command dropped.");
        return;
    }

    bool rt = (cmd.length() == 1 &&
               (cmd[0] == '?' || cmd[0] == '~' || cmd[0] == '!' || cmd[0] == 0x18));

    tx_item_t item;
    size_t len = cmd.length();
    if (len + 2 > TX_CMD_MAX) { DBG_SERIAL.println("[WARN] Command too long."); return; }
    memcpy(item.data, cmd.c_str(), len);
    if (!rt) { item.data[len++] = '\r'; item.data[len++] = '\n'; }
    item.len = len;

    if (xQueueSend(s_txQueue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
        DBG_SERIAL.println("[WARN] TX queue full — command dropped.");
        return;
    }
    if (rt) { DBG_SERIAL.print("TX: ["); DBG_SERIAL.print(cmd); DBG_SERIAL.println("]"); }
    else    { DBG_SERIAL.print("TX: ");  DBG_SERIAL.println(cmd); }
}

// ─────────────────────────────────────────────────────────────────────────
// handleFirePin — debounced edge: LOW→M3 S1000, HIGH→M5
// ─────────────────────────────────────────────────────────────────────────
static void handleFirePin(void) {
    int reading = digitalRead(FIRE_PIN);
    unsigned long now = millis();

    if (!g_debouncing) {
        if (reading != g_fireLast) { g_debouncing = true; g_debounceTs = now; g_pendEdge = reading; }
    } else if ((now - g_debounceTs) >= DEBOUNCE_MS) {
        g_debouncing = false;
        if      (g_pendEdge == LOW)  sendCommand("M3 S1000");
        else                         sendCommand("M5");
    }
    g_fireLast = reading;
}

// ─────────────────────────────────────────────────────────────────────────
// handleCNCOutput — drain RX ring to debug UART (non-blocking)
// ─────────────────────────────────────────────────────────────────────────
static void handleCNCOutput(void) {
    uint8_t tmp[64]; size_t n = 0;
    portENTER_CRITICAL(&s_rxMux);
    while (s_rxHead != s_rxTail && n < sizeof(tmp)) {
        tmp[n++] = s_rxRing[s_rxTail];
        s_rxTail = (s_rxTail + 1) % RX_RING_SIZE;
    }
    portEXIT_CRITICAL(&s_rxMux);
    if (n) DBG_SERIAL.write(tmp, n);
}

// ─────────────────────────────────────────────────────────────────────────
// handleDebugInput — forward lines from debug UART to CNC via queue
// ─────────────────────────────────────────────────────────────────────────
static void handleDebugInput(void) {
    while (DBG_SERIAL.available()) {
        char c = (char)DBG_SERIAL.read();
        if (c == '\n' || c == '\r') {
            if (g_dbgLine.length() > 0) { sendCommand(g_dbgLine); g_dbgLine = ""; }
        } else {
            g_dbgLine += c;
            if (g_dbgLine.length() > 256) { DBG_SERIAL.println("[WARN] Line overflow."); g_dbgLine = ""; }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
void setup() {
    DBG_SERIAL.begin(DBG_BAUD);
    delay(50);
    DBG_SERIAL.println("\n[BOOT] GRBL Sender — USB Host mode");

    s_ctrlSem = xSemaphoreCreateBinary();
    s_txSem   = xSemaphoreCreateBinary();
    s_txQueue = xQueueCreate(TX_QUEUE_LEN, sizeof(tx_item_t));

    const usb_host_config_t hostCfg = { .skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    ESP_ERROR_CHECK(usb_host_install(&hostCfg));

    xTaskCreatePinnedToCore(usbHostTask,   "usb_host",   3072, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(usbClientTask, "usb_client", 6144, NULL, 4, NULL, 0);

    pinMode(FIRE_PIN, INPUT_PULLUP);
    g_fireLast = digitalRead(FIRE_PIN);
    DBG_SERIAL.println("[BOOT] Fire pin 4 ready (INPUT_PULLUP).");
    DBG_SERIAL.println("[BOOT] Connect CNC USB — device will auto-init.");
}

// ─────────────────────────────────────────────────────────────────────────
void loop() {
    handleFirePin();
    handleCNCOutput();
    handleDebugInput();
    vTaskDelay(1);
}
