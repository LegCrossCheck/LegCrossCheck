# Firmware

Arduino sketches for the two Feathers. Needs the **esp32 by Espressif** core 3.x and the **Adafruit LSM6DS** and **Adafruit LIS3MDL** libraries (see the main README).

## Week-1 test sketches (`tests/`)

| Sketch | Flash to | What it does | Test |
|---|---|---|---|
| `pin_check` | Each board, bare | Checks every header pin for shorts and solder bridges, blinks D13, then confirms each pin when you touch it with a GND jumper | Soldering |
| `espnow_ping` | Both | Each board broadcasts a counter and prints what it hears, with RSSI and lost packets | Radio link |
| `imu_log` | Both | Streams accel, gyro, magnetometer and hall switch as CSV at 104 Hz; send `b` to take a magnet baseline | 2, 4 |
| `ir_tx` | Node A | Sends 38 kHz bursts at off / low / mid / high drive, announcing each block over ESP-NOW | 1 |
| `ir_rx` | Node B | Counts hits on both IR receivers per block and prints the hit rate as CSV | 1 |

Serial Monitor: 115200 baud. Lines starting with `#` are comments; everything else is CSV.

## Pins (week-1 tests)

| Feather pin | Node A | Node B |
|---|---|---|
| SDA / SCL | 9-DoF IMU (STEMMA QT) | 9-DoF IMU (STEMMA QT) |
| D6 | – | DRV5032 hall switch |
| D9 | IR drive low (220 Ω) | TSSP77038 output |
| D10 | IR drive mid (100 Ω) | TSOP38238 output |
| D11 | IR drive high (3 × 100 Ω) | – |

## Current firmware (`node_a/`, `node_b/`)

Both nodes run the same symmetric sketch; the two files differ only in `#define NODE 'A'` / `'B'`, so change both together. Flash `node_a` to Case A (left ankle) and `node_b` to Case B (right ankle).

- Board: **Adafruit Feather ESP32-S3 2MB PSRAM** (product 5477). Choosing No PSRAM flashes but will not boot. Tools → USB CDC On Boot: **Enabled**.
- Extra library: **Adafruit NeoPixel**.
- Every 50–70 ms each node fires a 38 kHz IR burst (A: 20 cycles, B: 40 cycles) and counts the other node's bursts by pulse width, ignoring its own.
- IMU at 100 Hz, sent in batches of 10 as JSON lines over USB serial and BLE (Nordic UART, `CrossCheck-A` / `CrossCheck-B`). View both nodes with `monitor/crosscheck_monitor.html`.
- Serial / BLE commands: `v` buzz 0.4 s, `v1000` buzz 1 s (max 3 s), `x` stop. The motor code on D11 has not been tested on hardware yet.

| Feather pin | Use |
|---|---|
| D9 | IR LED drive (1k → PN2222 base) |
| D5 | TSSP77038 output |
| D11 | Vibration motor drive (1k → PN2222 base, 10k to GND) |
| SDA / SCL | 9-DoF IMU (STEMMA QT), 0x6A + 0x1C |
| EN | Power switch to GND (not fitted yet) |

Full details: `docs/HANDOFF.md`.

## Early experiments (`experiments/`)

Kept for reference, not maintained: `crosscheck_A/B` (symmetric IR only), `nodeA` / `nodeB` and `nodeA_tx` / `nodeB_rx` (ESP-NOW command protocol, dropped in the current firmware), and single-purpose tests (`pin_test`, `ir_loopback_test`, `d9_dc_test`, `nodeA_ir_check`, `nodeA_imu_test`). Their pin maps differ from the current wiring; check each sketch's header.
