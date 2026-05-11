# ESP32-C6 Zigbee Router + Presence Sensor Design

**Date:** 2026-05-11  
**Board:** Seeed Studio XIAO ESP32-C6  
**Toolchain:** ESP-IDF v5.3 via Docker (`espressif/idf:v5.3`)  
**HA Integration:** ZHA (Zigbee Home Automation)

---

## Overview

Firmware for the XIAO ESP32-C6 that serves two roles simultaneously:

1. **Zigbee router** — extends Zigbee mesh range for other ZHA devices
2. **Room-level presence sensor** — passively detects occupancy via WiFi probe request sniffing, reported to ZHA as an occupancy sensor endpoint

The onboard user LED (GPIO15) is exposed as a ZHA-controllable dimmable light.

---

## Architecture

Three FreeRTOS tasks communicating through a shared state struct:

```
┌─────────────────────────────────────────┐
│            XIAO ESP32-C6                │
│                                         │
│  ┌──────────────┐   ┌────────────────┐  │
│  │ WiFi Sniffer │   │ Zigbee Stack   │  │
│  │ (802.11 2.4) │   │ (802.15.4)     │  │
│  │              │   │                │  │
│  │ Probe detect │──▶│ Router +       │  │
│  │ → occupied   │   │ Occupancy EP   │  │
│  │   state      │   │ + Light EP     │  │
│  └──────────────┘   └────────────────┘  │
│          │                  │           │
│          └────────┬─────────┘           │
│                   ▼                     │
│           ┌──────────────┐              │
│           │  LED (GPIO15) │              │
│           └──────────────┘              │
└─────────────────────────────────────────┘
```

Both radios coexist via Espressif's hardware coexistence mechanism, enabled in `sdkconfig.defaults`.

---

## Zigbee Device Profile

The device type at the network layer is **ZB_DEVICE_TYPE_ROUTER** — transparent to ZHA's device model, just extends the mesh.

### Endpoint 1 — Occupancy Sensor

| Cluster | Direction |
|---|---|
| Basic | Server |
| Occupancy Sensing | Server |

- Reports `occupied` / `unoccupied` via ZHA attribute report on every state change (push, no polling).
- Occupancy sensor type: `0x02` (ultrasonic — closest standard type for a non-PIR sensor).

### Endpoint 2 — Dimmable Light (LED)

| Cluster | Direction |
|---|---|
| Basic | Server |
| On/Off | Server |
| Level Control | Server |

- Controls onboard LED (GPIO15) via PWM (LEDC peripheral).
- Responds to `on/off` and `move_to_level` commands from ZHA.
- Last brightness value persisted in NVS, restored on reboot.
- Default brightness on first join: 50%.

---

## WiFi Sniffer + Occupancy Logic

- WiFi radio initialized in station mode (no credentials) with promiscuous mode enabled.
- Hops through **all 2.4GHz channels 1–13** at 200ms per channel (~2.6s full sweep).
- Channel range defined as `#define WIFI_CHANNEL_MIN 1` / `#define WIFI_CHANNEL_MAX 13` in `wifi_sniffer.h`.
- Filters for management frames, subtype `0x04` (probe requests).
- On each probe request: updates `last_activity` timestamp. No MACs stored.
- A task running every 30 seconds evaluates: if `now - last_activity > OCCUPANCY_TIMEOUT_SEC` → set `occupied = false`.
- On any occupancy state change → Zigbee attribute report sent to ZHA immediately.
- `OCCUPANCY_TIMEOUT_SEC` defined in `wifi_sniffer.h`, default `300` (5 minutes).

**Note on MAC randomization:** Modern devices (iOS, Android 10+) randomize probe request MACs. Since this firmware tracks only timestamp (not MACs), randomization has no negative effect — randomized devices still emit probe requests and are detected normally.

---

## LED Behavior (GPIO15, active LOW, PWM)

### Pre-join (firmware controlled)

| State | Pattern |
|---|---|
| Scanning / not joined | Fast blink, 200ms on/200ms off |
| Joining in progress | Slow blink, 1s on/1s off |

### Post-join (ZHA controlled)

- LED responds exclusively to ZHA `on/off` and `move_to_level` commands.
- Occupancy state is **not** reflected in LED post-join — handled via ZHA automations.
- Firmware hands over LED control to the Zigbee Level Control cluster on successful join.

---

## Project Structure

```
esp32c6-zigbee-router/
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                # Init, FreeRTOS task startup
│   ├── zigbee.c / zigbee.h   # Router init, endpoints, attribute callbacks
│   ├── wifi_sniffer.c / .h   # Promiscuous mode, channel hop, occupancy logic
│   └── led.c / led.h         # LEDC PWM driver, pre/post-join state machine
├── CMakeLists.txt
├── sdkconfig.defaults        # Zigbee router + coex + WiFi enabled
├── Makefile                  # Docker build/flash/monitor targets
├── .github/
│   └── workflows/
│       └── build.yml         # CI: build on push, release binary on tag
├── docs/
│   ├── index.html            # ESP Web Tools web flasher (GitHub Pages)
│   └── superpowers/specs/
│       └── 2026-05-11-zigbee-router-design.md
└── README.md
```

---

## Build & Flash

### Development (Docker)

```sh
make build                        # compile in espressif/idf:v5.3 container
make flash PORT=/dev/ttyACM0      # flash via Docker with USB passthrough
make monitor PORT=/dev/ttyACM0    # serial monitor
make clean                        # full clean
```

`PORT` defaults to `/dev/ttyACM0` (XIAO appears as ACM device on Linux).

### Release (CI + Web Flasher)

- **GitHub Actions** (`build.yml`): builds on every push; on a semver tag (e.g., `v1.0.0`) merges firmware into a single `.bin` and attaches it to the GitHub Release.
- **ESP Web Tools** (`docs/index.html`): static page hosted on GitHub Pages. Chrome/Edge + USB cable → visit URL → click Flash. No local tools required.
- **CLI fallback**: release includes `flash.sh` using only `esptool.py` (`pip install esptool`).

### sdkconfig.defaults key settings

```
CONFIG_ZB_ENABLED=y
CONFIG_ZB_ZCZR=y
CONFIG_ESP_COEX_SW_COEXIST_ENABLE=y
CONFIG_ESP_WIFI_ENABLED=y
CONFIG_IEEE802154_ENABLED=y
```

---

## Key Constants (all in source headers, easy to adjust)

| Constant | File | Default | Purpose |
|---|---|---|---|
| `OCCUPANCY_TIMEOUT_SEC` | `wifi_sniffer.h` | `300` | Seconds of silence before unoccupied |
| `WIFI_CHANNEL_MIN` | `wifi_sniffer.h` | `1` | First 2.4GHz channel to scan |
| `WIFI_CHANNEL_MAX` | `wifi_sniffer.h` | `13` | Last 2.4GHz channel to scan |
| `WIFI_CHANNEL_DWELL_MS` | `wifi_sniffer.h` | `200` | Milliseconds per channel |
| `LED_GPIO` | `led.h` | `15` | Onboard user LED pin |
| `LED_DEFAULT_BRIGHTNESS` | `led.h` | `128` | Default brightness (0–255) on first join |
