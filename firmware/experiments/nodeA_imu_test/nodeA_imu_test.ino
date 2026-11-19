// CrossCheck - Node A IMU test
// Board : Adafruit Feather ESP32-S3 TFT (PID 5483)
// Sensor: Adafruit LSM6DSOX + LIS3MDL 9-DoF (PID 4517) on the STEMMA QT port
//
// What it does
//   1. Powers the TFT and the STEMMA QT port, scans the I2C bus.
//   2. Reads accel + gyro (LSM6DSOX, 0x6A) and magnetometer (LIS3MDL, 0x1C) at 50 Hz.
//   3. Prints CSV over USB serial (115200) for logging:
//        t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT
//   4. Shows pitch / roll, gyro speed, magnetic field and a simple
//      "bounce" number (std dev of |accel| over the last second) on the TFT.

#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

// ---------- settings ----------
const uint32_t SAMPLE_MS  = 20;   // 50 Hz sampling
const uint32_t SCREEN_MS  = 200;  // 5 Hz screen refresh
const int      BOUNCE_WIN = 50;   // 1 s window at 50 Hz

const float GRAV = 9.80665f;
const float RAD2DEG = 57.29578f;

Adafruit_ST7789   tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL  mag;

bool magOK = false;

// ring buffer of |accel| (in g) for the bounce estimate
float accBuf[BOUNCE_WIN];
int   accIdx = 0;
int   accCount = 0;

// latest values for the screen
float ax, ay, az, gx, gy, gz, mx, my, mz;

// ---------- screen helpers ----------
void screenLine(int row, const char *text, uint16_t color = ST77XX_WHITE) {
  char buf[21];
  snprintf(buf, sizeof(buf), "%-20s", text);   // pad so old text is overwritten
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(0, row * 22);
  tft.print(buf);
}

void halt(const char *msg) {
  Serial.println(msg);
  screenLine(5, msg, ST77XX_RED);
  while (true) delay(1000);
}

// ---------- I2C scan ----------
void scanI2C() {
  char line[21] = "I2C:";
  int n = 0;
  Serial.print("I2C scan:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      if (n < 3) {
        char a[6];
        snprintf(a, sizeof(a), " %02X", addr);
        strncat(line, a, sizeof(line) - strlen(line) - 1);
      }
      n++;
    }
  }
  Serial.println(n ? "" : " nothing found");
  screenLine(1, n ? line : "I2C: nothing found", n ? ST77XX_WHITE : ST77XX_RED);
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  delay(1500);  // give the USB serial port time to come up

  // Power the TFT backlight and the I2C / STEMMA QT port
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  tft.init(135, 240);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextSize(2);           // 12 x 16 px characters, 20 per line
  screenLine(0, "CrossCheck Node A", ST77XX_CYAN);

  Wire.begin();
  scanI2C();

  if (!sox.begin_I2C()) halt("LSM6DSOX not found");
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  magOK = mag.begin_I2C();
  if (magOK) {
    mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag.setDataRate(LIS3MDL_DATARATE_80_HZ);
    mag.setRange(LIS3MDL_RANGE_4_GAUSS);   // best resolution; saturates very close to a magnet
  } else {
    Serial.println("LIS3MDL not found (continuing without magnetometer)");
  }

  screenLine(2, magOK ? "IMU + mag OK" : "IMU OK, no mag", magOK ? ST77XX_GREEN : ST77XX_YELLOW);
  delay(1200);
  tft.fillScreen(ST77XX_BLACK);

  Serial.println("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT");
}

// ---------- loop ----------
void loop() {
  static uint32_t nextSample = 0;
  static uint32_t nextScreen = 0;
  uint32_t now = millis();

  if (now >= nextSample) {
    nextSample = now + SAMPLE_MS;

    sensors_event_t a, g, t;
    sox.getEvent(&a, &g, &t);
    ax = a.acceleration.x / GRAV;  ay = a.acceleration.y / GRAV;  az = a.acceleration.z / GRAV;
    gx = g.gyro.x * RAD2DEG;    gy = g.gyro.y * RAD2DEG;    gz = g.gyro.z * RAD2DEG;

    if (magOK) {
      sensors_event_t m;
      mag.getEvent(&m);
      mx = m.magnetic.x;  my = m.magnetic.y;  mz = m.magnetic.z;   // microtesla
    } else {
      mx = my = mz = NAN;
    }

    accBuf[accIdx] = sqrtf(ax * ax + ay * ay + az * az);
    accIdx = (accIdx + 1) % BOUNCE_WIN;
    if (accCount < BOUNCE_WIN) accCount++;

    Serial.printf("%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                  (unsigned long)now, ax, ay, az, gx, gy, gz, mx, my, mz);
  }

  if (now >= nextScreen) {
    nextScreen = now + SCREEN_MS;

    // shin tilt from gravity (only meaningful when the leg is fairly still)
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
    float roll  = atan2f(ay, az) * RAD2DEG;
    float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
    float bField  = sqrtf(mx * mx + my * my + mz * mz);

    // bounce = std dev of |accel| over the last second
    float mean = 0, var = 0;
    for (int i = 0; i < accCount; i++) mean += accBuf[i];
    mean /= max(accCount, 1);
    for (int i = 0; i < accCount; i++) var += (accBuf[i] - mean) * (accBuf[i] - mean);
    float bounce = sqrtf(var / max(accCount, 1));

    char s[32];
    snprintf(s, sizeof(s), "Node A   t=%lus", (unsigned long)(now / 1000));
    screenLine(0, s, ST77XX_CYAN);
    snprintf(s, sizeof(s), "pitch %7.1f deg", pitch);   screenLine(1, s);
    snprintf(s, sizeof(s), "roll  %7.1f deg", roll);    screenLine(2, s);
    snprintf(s, sizeof(s), "gyro  %7.0f dps", gyroMag); screenLine(3, s);
    if (magOK) snprintf(s, sizeof(s), "|B|   %7.1f uT", bField);
    else       snprintf(s, sizeof(s), "|B|   (no mag)");
    screenLine(4, s);
    snprintf(s, sizeof(s), "bounce %6.3f g", bounce);
    screenLine(5, s, bounce > 0.05f ? ST77XX_YELLOW : ST77XX_WHITE);
  }
}
