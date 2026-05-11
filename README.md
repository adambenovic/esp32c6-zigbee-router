# ESP32-C6 Zigbee Router + Room Presence Sensor

Firmware for the [Seeed XIAO ESP32-C6](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/) that:

- **Extends your Zigbee mesh** as a ZHA router
- **Detects room occupancy** passively via WiFi probe request sniffing (no sensors needed)
- **Exposes the onboard LED** as a ZHA dimmable light (controllable from HA)

## Flash from browser (no tools required)

Visit the [web flasher](https://YOUR_GITHUB_USERNAME.github.io/esp32c6-zigbee-router/) in Chrome or Edge with your XIAO connected via USB-C.

## Flash with esptool (CLI)

```bash
pip install esptool
esptool.py --chip esp32c6 write_flash 0x0 firmware-merged.bin
```

Download `firmware-merged.bin` from [Releases](https://github.com/YOUR_GITHUB_USERNAME/esp32c6-zigbee-router/releases).

## Build from source

Requires Docker.

```bash
git clone https://github.com/YOUR_GITHUB_USERNAME/esp32c6-zigbee-router
cd esp32c6-zigbee-router
make build
make flash PORT=/dev/ttyACM0
make monitor PORT=/dev/ttyACM0
```

## Pairing with ZHA

1. Home Assistant → Settings → Devices & Services → ZHA → Add Device
2. Power the XIAO — LED fast-blinks (scanning) then slow-blinks (joining)
3. Device joins and appears with two entities: occupancy sensor + dimmable light

## Tuning constants

Edit and rebuild:

| Constant | File | Default | Meaning |
|---|---|---|---|
| `OCCUPANCY_TIMEOUT_SEC` | `main/wifi_sniffer.h` | `300` | Seconds of silence → unoccupied |
| `WIFI_CHANNEL_MIN` | `main/wifi_sniffer.h` | `1` | First channel to scan |
| `WIFI_CHANNEL_MAX` | `main/wifi_sniffer.h` | `13` | Last channel to scan |
| `WIFI_CHANNEL_DWELL_MS` | `main/wifi_sniffer.h` | `200` | ms per channel |
| `LED_DEFAULT_BRIGHTNESS` | `main/led.h` | `128` | Brightness on first join (0–254) |
