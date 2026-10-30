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

## Pins

| Feather pin | Node A | Node B |
|---|---|---|
| SDA / SCL | 9-DoF IMU (STEMMA QT) | 9-DoF IMU (STEMMA QT) |
| D6 | – | DRV5032 hall switch |
| D9 | IR drive low (220 Ω) | TSSP77038 output |
| D10 | IR drive mid (100 Ω) | TSOP38238 output |
| D11 | IR drive high (3 × 100 Ω) | – |

`node_a/` and `node_b/` hold the final firmware once week-1 results and the first model are in.
