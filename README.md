# CrossCheck

CS 437 final project: two ankle-worn nodes that detect leg crossing and leg bouncing and buzz after the habit lasts 20–60 s.

- **Node A (emitter):** ESP32-S3 Feather, 9-DoF IMU, IR LED (3 drive levels), magnet, vibration motor
- **Node B (receiver):** ESP32-S3 Feather, 9-DoF IMU (magnetometer reads Node A's magnet), TSSP77038 + TSOP38238 IR receivers, DRV5032 hall switch, vibration motor
- Nodes talk over ESP-NOW; a decision tree on Node A classifies normal / crossed / bouncing

Team: Yuhao Cheng, Tong Li, Kaijia Chen.

## Layout

```
firmware/
  node_a/        current firmware, Case A (left ankle)
  node_b/        current firmware, Case B (right ankle)
  tests/         one sketch per week-1 test
  experiments/   early sketches, kept for reference
monitor/         browser dashboard for both nodes (USB serial or BLE)
hardware/
  schematics/    KiCad schematics (breadboard stage)
  enclosure/     case STEP files (v9 current)
  cad_parts/     Adafruit STEP models of the parts
  figures/       circuit and assembly diagrams
analysis/        Python: serial logger, plots, training, tree export to C
data/            raw logs from each session
docs/            proposal, design update, parts list, handoff notes (HANDOFF.md)
```

Start with **[docs/HANDOFF.md](docs/HANDOFF.md)** for the current status, wiring, and next steps.

## Toolchain

**Firmware:** Arduino (C++) with the Espressif `arduino-esp32` core, built with arduino-cli or Arduino IDE 2.

- Board manager URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`, install **esp32 by Espressif**
- Board: **Adafruit Feather ESP32-S3** (2 MB PSRAM or No PSRAM, per the board label), or **Adafruit Feather ESP32-S3 TFT** for the TFT board
- Libraries: **Adafruit LSM6DS**, **Adafruit LIS3MDL**
- Built into the core: ESP-NOW, RMT (38 kHz IR), deep sleep
- Upload over a USB-C data cable; if it hangs, hold BOOT and tap RESET

**Analysis:** Python 3.12 via uv, with `pyserial`, `numpy`, `pandas`, `matplotlib`, `scikit-learn`, and `micromlgen` or `emlearn` for exporting the tree to C.

## Week-1 tests

Postures for every test: neutral sit, ankle cross, knee-over-knee, figure-4, feet tucked together, with all three of us.

| # | Test | Pass if |
|---|---|---|
| 1 | IR line of sight | At some drive level: ≥ 90% hits when crossed, ≤ 5% when not |
| 2 | Magnet distance | Crossed and uncrossed separate in ≥ 90% of samples after baseline subtraction |
| 4 | IMU data | Leave-one-person-out tree: ≥ 90% for bouncing and figure-4 |
| 5 | Power | Under 1 mA average seated with no events |

Test 3 (ToF) is dropped; the distance sensor is not part of the design.
