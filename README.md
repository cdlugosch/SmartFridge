# SmartFridge

Replacement fridge thermostat on a Seeed XIAO ESP32-C3 with the XIAO Expansion Board v1.1.
A DS18B20 inside the fridge measures the temperature, a 3.3 V relay module switches the
compressor. Status is shown on the expansion board OLED and published via MQTT.

## Features

- Hysteresis control (default 3.0 / 6.0 °C), adjustable via MQTT and stored in flash
- Compressor protection: min. 5 min off / 2 min on, 60 s start delay after boot
- Manual override with the user button (short press toggles, long press returns to auto, ends after 30 min)
- Alarm when the temperature stays 15 % outside the limits for 10 min, or the sensor fails
- Non-blocking WiFi/MQTT reconnect

## Wiring

| Function | Pin |
|---|---|
| DS18B20 data | D0 (GPIO2), 4.7k pull-up to 3V3 recommended |
| Relay | D7 (GPIO20) |
| User button | D1 (GPIO3), on the expansion board |
| OLED | SDA/SCL (GPIO6/7), on the expansion board |

## Setup

```bash
cp include/config.example.h include/config.h   # then edit WiFi + MQTT settings
pio run -t upload
```

## MQTT

All topics below `topic_root` (default `esp/fridge/`).

| Topic | Direction | Payload | When |
|---|---|---|---|
| `temperature` | out | `4.56` | every 5 min |
| `relay` | out | `1` / `0` | on change, on connect, every 5 min |
| `mode` | out | `auto` / `manual` | with `relay` |
| `alarm` | out | `{"alarm":false,"reason":"none","temperature":4.56,"limit":6.90}` | on change, on connect, every 5 min |
| `hysteresis` | out (retained) | `{"low":3.0,"high":6.0}` | on change, on connect |
| `device_status` | out | `{"device":"ds18b20","status":"..."}` | every 5 min while the sensor fails |
| `log` | out | text / JSON | only when debug is enabled |
| `hysteresis/low/set` | in | e.g. `3.0` | |
| `hysteresis/high/set` | in | e.g. `6.0` | |
| `debug` | in | `1` / `0` | |
