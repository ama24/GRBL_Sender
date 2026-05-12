# GRBL Sender

ESP32-S3 firmware that replaces a Raspberry Pi or PC as the host controller for a GRBL-based CNC machine. The ESP32-S3 connects to the CNC board's USB port as a **USB host**, sends a GRBL initialisation sequence on boot, and fires the spindle/laser on a trigger input.

---

## How it works

Most GRBL boards expose a USB port that presents itself as a virtual serial port (CDC-ACM). A host computer normally connects to this port to send G-code. This firmware makes the ESP32-S3 be that host — without any PC or Raspberry Pi in the chain.

```
ESP32-S3 OTG (USB HOST)  ──USB cable──▶  CNC board USB (CDC device)
ESP32-S3 UART0 (optional debug)  ──FTDI──▶  PC terminal
GPIO 4 (INPUT_PULLUP)  ◀──  trigger switch
```

> **Why not UART?** The previous approach wired UART0 through the CP2102N chip on the DevKitC-1 board and out the "COM" USB connector. That connector is a USB device, and so is the CNC board's USB port — two USB devices cannot talk to each other. The native OTG port must be used in host mode.

---

## Features

- **USB Host CDC-ACM** — enumerates the CNC board automatically; works with any board that presents as a CDC serial device (Arduino with ATmega16U2, built-in USB-CDC on STM32/RP2040-based controllers, etc.)
- **Auto GRBL init** on device connect: soft reset → status query → alarm clear (`$X`) → settings dump (`$$`) → spindle mode (`$32=0`)
- **Trigger input** on GPIO 4 (INPUT_PULLUP, debounced 50 ms):
  - Pull LOW → `M3 S1000` (spindle/laser on)
  - Release → `M5` (spindle/laser off)
- **Non-blocking debug serial** — UART0 logs at 115200 baud; nothing stalls if no terminal is connected
- **Manual G-code passthrough** — lines typed in the debug terminal are forwarded to the CNC

---

## Hardware

| ESP32-S3 pin / connector | Role |
|---|---|
| Native USB OTG ("USB" Type-C) | **Connect to CNC board USB** |
| UART0 TX GPIO 43 / RX GPIO 44 | Optional debug terminal (via FTDI or similar) |
| GPIO 4 | Trigger input (pull to GND to fire) |

### Single-USB boards

This firmware targets single-USB ESP32-S3 modules (no CP2102N, no second connector). The single OTG port is used as the host port. Debug output is available on the raw UART TX/RX pins; it works headlessly with nothing connected.

### VBUS

In USB host mode the ESP32-S3 must supply 5 V VBUS on the OTG connector. On most development boards this is provided automatically from the board's power rail. On bare modules you may need to drive a `VBUS_EN` GPIO or connect a 5 V supply externally. If the CNC device never enumerates, check VBUS first.

---

## Configuration

All user-adjustable settings are `#define` constants near the top of [src/main.cpp](src/main.cpp):

| Constant | Default | Description |
|---|---|---|
| `FIRE_PIN` | `4` | GPIO for the trigger input |
| `DEBOUNCE_MS` | `50` | Switch debounce window (ms) |
| `CNC_BAUD` | `460800` | GRBL baud rate — must match `$` setting on the CNC board |
| `INIT_DELAY_SOFTRESET_MS` | `2500` | Wait after soft reset (0x18) for GRBL to reboot |
| `INIT_DELAY_UNLOCK_MS` | `500` | Wait after `$X` alarm clear |
| `INIT_DELAY_SETTINGS_MS` | `1500` | Wait after `$$` settings dump |
| `INIT_DELAY_SPINDLE_MS` | `500` | Wait after `$32=0` spindle mode set |
| `DBG_BAUD` | `115200` | Debug UART baud rate |

---

## Build & flash

Requirements: [PlatformIO](https://platformio.org/) with the `espressif32` platform installed.

```bash
# Build
pio run

# Flash (connect via JTAG or the board's "COM" / programming port)
pio run --target upload

# Monitor debug serial (connect FTDI to GPIO43/44)
pio device monitor --baud 115200
```

The firmware uses only the built-in ESP-IDF USB Host library (`usb/usb_host.h`) — no external library dependencies.

---

## Compatibility notes

The USB Host driver scans the device's configuration descriptor for an interface with bulk IN + bulk OUT endpoints. It prefers CDC Data class interfaces (class `0x0A`) but falls back to any interface with suitable endpoints.

- **Standard CDC-ACM** (Arduino with ATmega16U2, STM32, RP2040): full support including `SET_LINE_CODING`
- **CP2102 / CP2104**: typically advertises CDC; `SET_LINE_CODING` is sent and usually accepted
- **CH340 / CH341**: exposes bulk endpoints but uses vendor-specific baud-rate commands; the standard `SET_LINE_CODING` request will be silently ignored and the CH340 will use its default/hardware baud rate — if your board uses a CH340 you will need to add the CH340 vendor init sequence

---

## License

MIT — see [LICENSE](LICENSE).
