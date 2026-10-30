// Week-1 tests 2 and 4: stream the 9-DoF IMU over USB serial as CSV.
// Flash to both Feathers. On Node B the magnetometer column is the magnet test.
//
// Columns: t_ms, accel (m/s^2), gyro (rad/s), mag (uT), dm, hall
//   dm   = size of (mag - baseline) in uT; -1 until a baseline is taken
//   hall = DRV5032 output on D6, 0 when the magnet is close (Node B only)
//
// Send "b" in the Serial Monitor while sitting normally to take a new
// magnetometer baseline (averages 1 s of readings).

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

const int HALL_PIN = 6;               // D6
const uint32_t SAMPLE_HZ = 104;       // matches the LSM6DSOX data rate
const uint32_t PERIOD_US = 1000000 / SAMPLE_HZ;
const int BASELINE_SAMPLES = 100;

Adafruit_LSM6DSOX imu;
Adafruit_LIS3MDL mag;

float baseX = 0, baseY = 0, baseZ = 0;
bool hasBaseline = false;
uint32_t nextSample = 0;

void takeBaseline() {
  float sx = 0, sy = 0, sz = 0;
  sensors_event_t m;
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    mag.getEvent(&m);
    sx += m.magnetic.x;
    sy += m.magnetic.y;
    sz += m.magnetic.z;
    delay(10);
  }
  baseX = sx / BASELINE_SAMPLES;
  baseY = sy / BASELINE_SAMPLES;
  baseZ = sz / BASELINE_SAMPLES;
  hasBaseline = true;
  Serial.printf("# baseline uT: %.2f, %.2f, %.2f\n", baseX, baseY, baseZ);
  nextSample = micros();  // don't burst out the samples missed while averaging
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);

  // Power the STEMMA QT port (pin name differs between Feather versions)
#if defined(PIN_I2C_POWER)
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, HIGH);
#elif defined(TFT_I2C_POWER)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif
  delay(10);

  if (!imu.begin_I2C()) {
    Serial.println("# LSM6DSOX not found, check the STEMMA QT cable");
    while (true) delay(1000);
  }
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

  if (!mag.begin_I2C()) {
    Serial.println("# LIS3MDL not found, check the STEMMA QT cable");
    while (true) delay(1000);
  }
  // 16 gauss = +/-1600 uT, so the magnet doesn't saturate it at close range
  mag.setRange(LIS3MDL_RANGE_16_GAUSS);
  mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
  mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  pinMode(HALL_PIN, INPUT_PULLUP);

  Serial.println("t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,dm,hall");
  nextSample = micros();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'b' || c == 'B') takeBaseline();
  }

  if ((int32_t)(micros() - nextSample) < 0) return;
  nextSample += PERIOD_US;

  sensors_event_t a, g, t, m;
  imu.getEvent(&a, &g, &t);
  mag.getEvent(&m);

  float dm = -1;
  if (hasBaseline) {
    float dx = m.magnetic.x - baseX;
    float dy = m.magnetic.y - baseY;
    float dz = m.magnetic.z - baseZ;
    dm = sqrtf(dx * dx + dy * dy + dz * dz);
  }

  Serial.printf("%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%d\n",
                (unsigned long)millis(),
                a.acceleration.x, a.acceleration.y, a.acceleration.z,
                g.gyro.x, g.gyro.y, g.gyro.z,
                m.magnetic.x, m.magnetic.y, m.magnetic.z,
                dm, digitalRead(HALL_PIN));
}
