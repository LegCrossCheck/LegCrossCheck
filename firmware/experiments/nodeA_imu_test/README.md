# Node A IMU test

Reads the 9-DoF IMU on the ESP32-S3 TFT Feather, shows tilt / motion on the screen,
and streams CSV over USB serial.

## One-time setup (Arduino IDE 2.x)

1. **Board package**: Preferences → Additional boards manager URLs, add
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`,
   then Boards Manager → install **esp32 by Espressif Systems**.
2. **Libraries** (Library Manager), say "Install all" when asked for dependencies:
   - Adafruit LSM6DS
   - Adafruit LIS3MDL
   - Adafruit ST7735 and ST7789 Library
   - (Adafruit GFX, Adafruit BusIO, Adafruit Unified Sensor come in as dependencies)
3. **Board**: Tools → Board → esp32 → **Adafruit Feather ESP32-S3 TFT**.
   Leave "USB CDC On Boot" = Enabled.

## Flash

1. Plug the Feather in with a data-capable USB-C cable, pick its port, click Upload.
2. If upload fails or no port shows up: hold **BOOT**, tap **RESET**, release BOOT,
   upload again, then tap RESET once when it finishes.

## What you should see

- Screen, at boot: `I2C: 1C 36 6A` (or similar; 0x36 is the Feather's battery gauge),
  then `IMU + mag OK`.
- Then live values:
  - `pitch` / `roll`: tilt from gravity. Lay the board flat → about 0 / 0;
    stand it on its edge → one of them goes to about ±90.
  - `gyro`: rotation speed, near 0 when still.
  - `|B|`: magnetic field. Without a magnet nearby, about 25–65 uT (Earth's field);
    bring the magnet close and it jumps.
  - `bounce`: shakiness over the last second. Tap or shake the sensor and it rises;
    above 0.05 g it turns yellow.
- Serial Monitor at **115200** baud: one CSV line per sample (50 Hz):
  `t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT`.
  Close the Serial Monitor and use Serial Plotter to see the curves.

## If something is wrong

| Screen says | Likely cause |
| --- | --- |
| `I2C: nothing found` | QT cable not fully seated, or cable in the wrong port |
| `LSM6DSOX not found` | Same as above; check the cable at both ends |
| `IMU OK, no mag` | Magnetometer address changed (solder jumper on the back) |
| Screen stays black | Wrong board selected in Tools → Board |
