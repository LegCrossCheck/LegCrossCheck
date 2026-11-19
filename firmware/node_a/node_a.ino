// node_a.ino  --  Node A (Feather ESP32-S3, no screen)
//
// node_a.ino and node_b.ino are identical except for the NODE line.
// View both at once with crosscheck_monitor.html (in monitor/).
//
// Same IR behaviour as crosscheck_A / crosscheck_B (A sends 20-cycle bursts,
// B sends 40-cycle bursts, each accepts only the other's length), plus: reads every attached sensor and streams one JSON line
// over USB serial 20 times per second for nodeB_monitor.html.
// The same lines also go out over Bluetooth LE (Nordic UART service, device
// name "CrossCheck-B") 5 times per second, so the page can connect wirelessly.
//
// The accelerometer and gyro are sampled 100 times per second. Samples go out
// in batches of 10 (one line every 100 ms, on USB and Bluetooth):
//   {"t":<ms of first sample>,"s":[[ax,ay,az,gx,gy,gz], ...]}
//   ax..az in mg (1000 = 1 g), gx..gz in 0.1 deg/s
//
// Libraries: Adafruit NeoPixel, Adafruit LSM6DS, Adafruit LIS3MDL
// Board:     must match the Feather you have, or it will not boot:
//              product 5477 (4MB flash, 2MB PSRAM) -> "Adafruit Feather ESP32-S3 2MB PSRAM"
//              product 5323 (8MB flash, no PSRAM)  -> "Adafruit Feather ESP32-S3 No PSRAM"
//            Tools -> USB CDC On Boot: Enabled
//
// Pins
//   D9   IR LED driver (1k -> MPS2222A base)
//   D5   TSSP77038 Sig
//   D6   DRV5032 hall switch (reads "no magnet" if nothing is connected)
//   D11  vibration motor driver (1k -> PN2222 base, 10k base -> GND, 1N4001 across the motor)
//   QT   LSM6DSOX + LIS3MDL 9-DoF (I2C)
//   on-board: NeoPixel 33 (power 21), red LED 13, battery gauge on I2C

#define NODE 'A'          // <-- 'A' in node_a, 'B' in node_b

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

// ---------------- Bluetooth LE (Nordic UART service) ----------------
#if NODE == 'A'
#define BLE_NAME    "CrossCheck-A"
#else
#define BLE_NAME    "CrossCheck-B"
#endif
#define NUS_SERVICE "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX_CHAR "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   // board -> computer (notify)
#define NUS_RX_CHAR "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   // computer -> board (write)

BLEServer*         bleServer = nullptr;
BLECharacteristic* bleTx = nullptr;
volatile bool      bleConnected = false;

// ---------------- vibration motor ----------------
const uint8_t  MOTOR_PIN      = 11;
const uint32_t VIB_DEFAULT_MS = 400;
const uint32_t VIB_MAX_MS     = 3000;
const uint32_t VIB_STOP       = 0xFFFFFFFF;
volatile uint32_t vibRequestMs = 0;   // set by a command, picked up in loop()
uint32_t vibOffAtMs = 0;
bool     vibOn = false;

// Commands, over USB serial (end with Enter) or Bluetooth:
//   v        buzz for 400 ms
//   v1000    buzz for 1000 ms (3000 max)
//   x        stop
void handleCommand(const char* s, size_t n) {
  if (n == 0) return;
  if (s[0] == 'v' || s[0] == 'V') {
    uint32_t ms = 0;
    for (size_t i = 1; i < n && s[i] >= '0' && s[i] <= '9'; i++) ms = ms * 10 + (s[i] - '0');
    if (ms == 0) ms = VIB_DEFAULT_MS;
    if (ms > VIB_MAX_MS) ms = VIB_MAX_MS;
    vibRequestMs = ms;
  } else if (s[0] == 'x' || s[0] == 'X') {
    vibRequestMs = VIB_STOP;
  }
}

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto v = c->getValue();
    handleCommand(v.c_str(), v.length());
  }
};

class BleCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override    { bleConnected = true; }
  void onDisconnect(BLEServer*) override { bleConnected = false; BLEDevice::startAdvertising(); }
};

// ---------------- pins ----------------
const uint8_t IR_TX_PIN = 9;
const uint8_t IR_RX_PIN = 5;
const uint8_t HALL_PIN  = 6;
const uint8_t LED_PIN   = 13;
const uint8_t NEO_PIN   = 33;
const uint8_t NEO_PWR   = 21;
const uint8_t I2C_PWR   = 7;      // powers the QT port on this Feather

// ---------------- IR ----------------
const uint32_t CARRIER_HZ  = 38000;
const uint8_t  DUTY        = 128;   // 128 = 50%, 64 = 25%, 16 = 6%, 8 = 3%
#if NODE == 'A'
const uint16_t MY_CYCLES   = 20;    // A sends 20 cycles (~0.53 ms)
const uint32_t PEER_MIN_US = 840;   // B sends 40 cycles (~1.05 ms)
const uint32_t PEER_MAX_US = 1320;
#else
const uint16_t MY_CYCLES   = 40;    // B sends 40 cycles (~1.05 ms)
const uint32_t PEER_MIN_US = 300;   // A sends 20 cycles (~0.53 ms)
const uint32_t PEER_MAX_US = 800;
#endif
const uint32_t BLANK_US    = 1500;  // ignore the receiver this long after our own burst
const uint32_t TX_PERIOD_MS = 50;   // plus 0-20 ms random

// ---------------- power ----------------
const uint32_t CPU_MHZ        = 80;   // 80 is the lowest speed Bluetooth allows (default is 240)
const uint8_t  PIXEL_BRIGHT   = 8;    // colour LED brightness, 0-255 (was 40)

const uint32_t REPORT_MS = 50;      // 20 status lines per second on USB
const uint32_t SAMPLE_MS = 10;      // IMU sample period: 100 Hz
const int      BATCH     = 10;      // IMU samples per line

#if ESP_ARDUINO_VERSION_MAJOR < 3
const uint8_t LEDC_CH = 0;
#endif

Adafruit_NeoPixel pixel(1, NEO_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL  lis;

bool imuOk = false, magOk = false, gaugeOk = false;

// shared with the interrupt
volatile bool     txBusy = false;
volatile uint32_t txEndUs = 0;
volatile bool     lowValid = false;
volatile uint32_t lowStartUs = 0;
volatile uint32_t peerCount = 0;    // valid bursts from the other node
volatile uint32_t badCount = 0;     // wrong-length pulses
volatile uint32_t lastWidthUs = 0;
volatile uint32_t lastPeerMs = 0;
volatile bool     everSeen = false;

uint32_t sentCount = 0;

// latest IMU sample, shared by the batch stream and the status line
float sAx = 0, sAy = 0, sAz = 0;    // g
float sGx = 0, sGy = 0, sGz = 0;    // deg/s
float sTc = 0;

void IRAM_ATTR onRxEdge() {
  uint32_t now = micros();
  if (digitalRead(IR_RX_PIN) == LOW) {            // burst begins
    lowStartUs = now;
    lowValid = !txBusy && (uint32_t)(now - txEndUs) > BLANK_US;
  } else {                                        // burst ends
    if (!lowValid || txBusy) return;
    lowValid = false;
    uint32_t w = now - lowStartUs;
    lastWidthUs = w;
    if (w >= PEER_MIN_US && w <= PEER_MAX_US) {
      peerCount++;
      lastPeerMs = millis();
      everSeen = true;
    } else {
      badCount++;
    }
  }
}

void carrier(bool on) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(IR_TX_PIN, on ? DUTY : 0);
#else
  ledcWrite(LEDC_CH, on ? DUTY : 0);
#endif
}

void sendBurst() {
  txBusy = true;
  lowValid = false;
  carrier(true);
  delayMicroseconds((uint32_t)MY_CYCLES * 1000000UL / CARRIER_HZ);
  carrier(false);
  txEndUs = micros();
  txBusy = false;
  sentCount++;
}

void bleBegin() {
  BLEDevice::init(BLE_NAME);
  BLEDevice::setMTU(247);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BleCallbacks());
  BLEService* svc = bleServer->createService(NUS_SERVICE);
  bleTx = svc->createCharacteristic(NUS_TX_CHAR, BLECharacteristic::PROPERTY_NOTIFY);
  BLECharacteristic* rx = svc->createCharacteristic(
      NUS_RX_CHAR, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  bleTx->addDescriptor(new BLE2902());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);          // the name travels in the scan response
  BLEDevice::startAdvertising();
}

// Send one line, split into pieces that fit the negotiated packet size.
void bleSend(const char* data, size_t len) {
  if (!bleConnected || !bleTx) return;
  int mtu = bleServer->getPeerMTU(bleServer->getConnId());
  size_t chunk = (mtu > 23) ? (size_t)(mtu - 3) : 20;
  if (chunk > 244) chunk = 244;
  while (len > 0 && bleConnected) {
    size_t n = len < chunk ? len : chunk;
    bleTx->setValue((uint8_t*)data, n);
    bleTx->notify();
    data += n;  len -= n;
    if (len > 0) delay(3);
  }
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void tryInitSensors() {
  if (!imuOk && i2cPresent(0x6A) && sox.begin_I2C(0x6A)) {
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
    sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
    imuOk = true;
  }
  if (!magOk && i2cPresent(0x1C) && lis.begin_I2C(0x1C)) {
    lis.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    lis.setDataRate(LIS3MDL_DATARATE_80_HZ);
    lis.setRange(LIS3MDL_RANGE_16_GAUSS);   // widest range, the magnet gets close
    magOk = true;
  }
  if (!gaugeOk) gaugeOk = i2cPresent(0x36);  // MAX17048 battery gauge
}

// MAX17048 read without a library. Returns false if the chip did not answer.
// rate = charge (+) or discharge (-) speed in percent per hour, as estimated by the chip.
bool readGauge(float &volts, float &percent, float &rate) {
  uint8_t regs[3] = {0x02, 0x04, 0x16};
  uint16_t val[3];
  for (int i = 0; i < 3; i++) {
    Wire.beginTransmission(0x36);
    Wire.write(regs[i]);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)0x36, (uint8_t)2) != 2) return false;
    val[i] = ((uint16_t)Wire.read() << 8) | Wire.read();
  }
  volts = val[0] * 78.125e-6f;
  percent = val[1] / 256.0f;
  rate = (int16_t)val[2] * 0.208f;
  return true;
}

// Lists every I2C address that answers, on USB and Bluetooth (shows in the page log).
void printI2cScan() {
  char buf[200];
  int n = snprintf(buf, sizeof(buf), "i2c SDA=%d SCL=%d found:", SDA, SCL);
  int count = 0;
  for (uint8_t a = 8; a < 120 && n < (int)sizeof(buf) - 8; a++) {
    if (i2cPresent(a)) { n += snprintf(buf + n, sizeof(buf) - n, " 0x%02X", a); count++; }
  }
  if (count == 0) n += snprintf(buf + n, sizeof(buf) - n, " nothing");
  n += snprintf(buf + n, sizeof(buf) - n, "\n");
  if (Serial) Serial.write((const uint8_t*)buf, n);
  bleSend(buf, n);
}

// Read the IMU on a fixed 100 Hz schedule and send every BATCH samples as one line.
void sampleImu(uint32_t now) {
  static uint32_t nextMs = 0;
  static char buf[560];
  static int len = 0, count = 0;

  if (!imuOk) return;
  if ((int32_t)(now - nextMs) < 0) return;
  nextMs += SAMPLE_MS;
  if ((int32_t)(now - nextMs) > 50) nextMs = now + SAMPLE_MS;   // fell far behind: restart the schedule

  sensors_event_t a, g, t;
  sox.getEvent(&a, &g, &t);
  sAx = a.acceleration.x / 9.80665f;
  sAy = a.acceleration.y / 9.80665f;
  sAz = a.acceleration.z / 9.80665f;
  sGx = g.gyro.x * 57.29578f;
  sGy = g.gyro.y * 57.29578f;
  sGz = g.gyro.z * 57.29578f;
  sTc = t.temperature;

  if (count == 0) len = snprintf(buf, sizeof(buf), "{\"t\":%lu,\"s\":[", (unsigned long)now);
  len += snprintf(buf + len, sizeof(buf) - len, "%s[%d,%d,%d,%d,%d,%d]", count ? "," : "",
                  (int)lroundf(sAx * 1000), (int)lroundf(sAy * 1000), (int)lroundf(sAz * 1000),
                  (int)lroundf(sGx * 10), (int)lroundf(sGy * 10), (int)lroundf(sGz * 10));
  if (++count >= BATCH) {
    len += snprintf(buf + len, sizeof(buf) - len, "]}\n");
    if (Serial) Serial.write((const uint8_t*)buf, len);
    bleSend(buf, len);
    count = 0;
  }
}

void setPixel(uint8_t r, uint8_t g, uint8_t b) {
  static uint32_t last = 0xFFFFFFFF;
  uint32_t c = pixel.Color(r, g, b);
  if (c == last) return;
  last = c;
  pixel.setPixelColor(0, c);
  pixel.show();
}

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);          // motor off before anything else
  digitalWrite(MOTOR_PIN, LOW);

  setCpuFrequencyMhz(CPU_MHZ);
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(IR_RX_PIN, INPUT_PULLUP);

  pinMode(NEO_PWR, OUTPUT);
  digitalWrite(NEO_PWR, HIGH);
  pinMode(I2C_PWR, OUTPUT);
  digitalWrite(I2C_PWR, HIGH);
  delay(10);

  pixel.begin();
  pixel.setBrightness(PIXEL_BRIGHT);
  setPixel(0, 0, 255);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(IR_TX_PIN, CARRIER_HZ, 8);
#else
  ledcSetup(LEDC_CH, CARRIER_HZ, 8);
  ledcAttachPin(IR_TX_PIN, LEDC_CH);
#endif
  carrier(false);

  Wire.begin();
  Wire.setClock(400000);
  delay(50);
  tryInitSensors();

  bleBegin();

  attachInterrupt(digitalPinToInterrupt(IR_RX_PIN), onRxEdge, CHANGE);

  delay(300);
  Serial.printf("Node %c monitor ready (IR tx D9, rx D5, motor D11)\n", NODE);
  printI2cScan();
}

void loop() {
  static uint32_t nextTxMs = 0, nextReportMs = 0, nextRetryMs = 2000;
  static uint32_t rateT0 = 0, peer0 = 0, bad0 = 0, sent0 = 0;
  static uint32_t rxRate = 0, badRate = 0, txRate = 0;
  static uint32_t lastPeerSeen = 0;
  static float batV = 0, batPct = 0, batRate = 0;
  static bool batOk = false;

  uint32_t now = millis();

  sampleImu(now);

  // ---- commands typed over USB ----
  static char cmd[12];
  static uint8_t cmdLen = 0;
  while (Serial && Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleCommand(cmd, cmdLen); cmdLen = 0; }
    else if (cmdLen < sizeof(cmd) - 1) cmd[cmdLen++] = ch;
  }

  // ---- vibration motor ----
  uint32_t req = vibRequestMs;
  if (req) {
    vibRequestMs = 0;
    if (req == VIB_STOP) vibOn = false;
    else { vibOn = true; vibOffAtMs = now + req; }
    digitalWrite(MOTOR_PIN, vibOn ? HIGH : LOW);
  }
  if (vibOn && (int32_t)(now - vibOffAtMs) >= 0) {
    vibOn = false;
    digitalWrite(MOTOR_PIN, LOW);
  }

  // ---- send our own burst ----
  if ((int32_t)(now - nextTxMs) >= 0) {
    sendBurst();
    nextTxMs = now + TX_PERIOD_MS + random(0, 21);
  }

  // ---- lights ----
  uint32_t pc = peerCount;
  if (pc != lastPeerSeen) {
    lastPeerSeen = pc;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  uint32_t age = now - lastPeerMs;
  bool seen = everSeen && age < 300;
  if (everSeen && age < 60)  setPixel(255, 0, 0);   // pulse just arrived
  else if (seen)             setPixel(60, 0, 0);    // seen within 300 ms
  else                       setPixel(0, 0, 255);   // nothing from the other node

  // ---- once a second: rates, battery, retry missing sensors ----
  if (now - rateT0 >= 1000) {
    uint32_t bc = badCount;
    rxRate = pc - peer0;         peer0 = pc;
    badRate = bc - bad0;         bad0 = bc;
    txRate = sentCount - sent0;  sent0 = sentCount;
    rateT0 = now;
    batOk = gaugeOk && readGauge(batV, batPct, batRate);
  }
  if ((int32_t)(now - nextRetryMs) >= 0) {
    nextRetryMs = now + 2000;
    if (!imuOk || !magOk || !gaugeOk) tryInitSensors();
    if (!imuOk || !magOk) printI2cScan();      // keep reporting until both are found
  }

  // ---- report ----
  if ((int32_t)(now - nextReportMs) >= 0) {
    nextReportMs = now + REPORT_MS;

    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, tc = 0;
    float mx = 0, my = 0, mz = 0, pitch = 0, roll = 0;

    if (imuOk) {                          // latest sample from sampleImu()
      ax = sAx;  ay = sAy;  az = sAz;
      gx = sGx;  gy = sGy;  gz = sGz;
      tc = sTc;
      roll  = atan2f(ay, az) * 57.29578f;
      pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
    }
    if (magOk) {
      sensors_event_t m;
      lis.getEvent(&m);
      mx = m.magnetic.x;  my = m.magnetic.y;  mz = m.magnetic.z;   // uT
    }
    float bAbs = sqrtf(mx * mx + my * my + mz * mz);
    int hall = (digitalRead(HALL_PIN) == LOW) ? 1 : 0;   // DRV5032 pulls low near a magnet

    static char line[448];
    static uint32_t reportNo = 0;
    int len = snprintf(line, sizeof(line),
        "{\"node\":\"%c\",\"t\":%lu,\"seen\":%d,\"rx\":%lu,\"bad\":%lu,\"w\":%lu,\"tx\":%lu,\"n\":%lu,"
        "\"imu\":%d,\"a\":[%.3f,%.3f,%.3f],\"g\":[%.1f,%.1f,%.1f],\"pitch\":%.1f,\"roll\":%.1f,\"tc\":%.1f,"
        "\"mag\":%d,\"m\":[%.1f,%.1f,%.1f],\"B\":%.1f,"
        "\"hall\":%d,\"vib\":%d,\"bat\":%d,\"v\":%.3f,\"pct\":%.1f,\"cr\":%.1f}\n",
        NODE, (unsigned long)now, seen ? 1 : 0, (unsigned long)rxRate, (unsigned long)badRate,
        (unsigned long)lastWidthUs, (unsigned long)txRate, (unsigned long)pc,
        imuOk ? 1 : 0, ax, ay, az, gx, gy, gz, pitch, roll, tc,
        magOk ? 1 : 0, mx, my, mz, bAbs,
        hall, vibOn ? 1 : 0, batOk ? 1 : 0, batV, batPct, batRate);
    if (len >= (int)sizeof(line)) len = sizeof(line) - 1;
    if (len > 0) {
      if (Serial) Serial.write((const uint8_t*)line, len);
      if ((reportNo++ & 3) == 0) bleSend(line, len);    // every 4th line = 5 Hz
    }
  }

  delay(1);   // let the CPU idle; everything above runs on a 10 ms or slower schedule
}
