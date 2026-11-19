// CrossCheck - Node B (RECEIVER)   *** flash this onto the Feather WITH the screen ***
// Board: Adafruit Feather ESP32-S3 TFT
//
// Wiring
//   TSSP77038 board : Sig (white) -> D9, V+ (red) -> 3V, GND (black) -> GND
//   DRV5032 hall    : OUT -> D6, VIN -> 3V, GND -> GND      (optional)
//   9-DoF IMU       : STEMMA QT cable (only the LIS3MDL magnetometer is used here, optional)
//
// Two modes - press BOOT to switch:
//
// 1) TEST (default): IR + link test at 38 kHz
//    Node B repeatedly tells Node A over ESP-NOW:
//      FIRE   -> A sends one IR burst; B checks whether its receiver (D9) saw it   -> IR hit rate
//      SILENT -> A does nothing;      B checks whether the receiver triggered anyway -> false rate
//    Every 50 FIRE + 50 SILENT rounds the screen shows IR hit, false, link % + RSSI,
//    magnetic field |B| and the hall switch.
//
// 2) SWEEP: carrier frequency sweep
//    For each frequency (30 ... 56 kHz) A fires 40 bursts at reduced power and B records the
//    hit rate, drawn as a bar chart. The receiver is tuned to 38 kHz, so bars should peak
//    around 38 and fall off on both sides.
//    If every bar is 100 %: the signal is too strong - move the boards apart (or lower SWEEP_DUTY).
//    If every bar is 0 %: too weak - move them closer (or raise SWEEP_DUTY).
//
// Serial (115200):
//   TEST : t_ms,fire_ok,ir_hits,silent_ok,false_hits,rounds,rssi,b_uT,hall
//   SWEEP: sweep,freq_hz,duty,fire_ok,ir_hits
//
// Protocol v2 - Node A must run the matching nodeA_tx (same MAGIC).

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_LIS3MDL.h>

// ---- must match Node A ----
const uint8_t WIFI_CHANNEL = 1;
const uint8_t MAGIC = 0xC6;   // protocol v2
enum : uint8_t { MSG_FIRE = 1, MSG_SILENT = 2, MSG_ACK = 3 };
struct __attribute__((packed)) Msg {
  uint8_t  magic;
  uint8_t  type;
  uint16_t seq;
  uint8_t  fired;
  uint8_t  duty;   // 0-255, 128 = 50 %
  uint32_t freq;   // carrier frequency in Hz
};
const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---- pins / timing ----
const uint8_t  IR_RX_PIN  = 9;
const uint8_t  HALL_PIN   = 6;
const uint8_t  BOOT_BTN   = 0;
const uint32_t TIMEOUT_US = 20000;   // give up waiting for A's reply after 20 ms
const uint32_t TAIL_US    = 1000;    // keep watching the receiver 1 ms after A's reply
const uint32_t GAP_MS     = 8;       // pause between commands

// ---- test mode ----
const int      TEST_ROUNDS = 50;     // FIRE + SILENT pairs per report
const uint32_t TEST_FREQ   = 38000;
const uint8_t  TEST_DUTY   = 128;    // 50 %

// ---- sweep mode ----
const uint32_t SWEEP_FREQS[] = {30000, 33000, 35000, 36000, 37000, 38000,
                                39000, 40000, 42000, 45000, 50000, 56000};
const int      N_SWEEP      = sizeof(SWEEP_FREQS) / sizeof(SWEEP_FREQS[0]);
const int      SWEEP_ROUNDS = 40;    // bursts per frequency
const uint8_t  SWEEP_DUTY   = 128;    // ~6 % duty = weaker signal, so the receiver's tuning shows

Adafruit_ST7789  tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_LIS3MDL mag;
bool magOK = false;

enum Mode { MODE_TEST, MODE_SWEEP };
Mode mode = MODE_TEST;

volatile bool     ackFlag = false;
volatile uint16_t ackSeq = 0;
volatile int      lastRssi = 0;
uint16_t seqCounter = 0;

// ---------- ESP-NOW receive ----------
bool handleMsg(const uint8_t *data, int len) {   // true if it was an ACK from Node A
  if (len != sizeof(Msg)) return false;
  Msg m;
  memcpy(&m, data, sizeof(m));
  if (m.magic != MAGIC || m.type != MSG_ACK) return false;
  ackSeq = m.seq;
  ackFlag = true;
  return true;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (handleMsg(data, len) && info && info->rx_ctrl) lastRssi = info->rx_ctrl->rssi;
}
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) { handleMsg(data, len); }
#endif

// One command + listening window.
// Returns 0 = A did not answer, 1 = answered and receiver went LOW, 2 = answered, receiver stayed HIGH
int doRound(uint8_t type, uint32_t freq, uint8_t duty) {
  uint16_t seq = seqCounter++;
  Msg m = {MAGIC, type, seq, 0, duty, freq};
  ackFlag = false;
  esp_now_send(BCAST, (const uint8_t *)&m, sizeof(m));

  uint32_t t0 = micros(), tAck = 0;
  bool low = false, gotAck = false;
  while (true) {
    if (digitalRead(IR_RX_PIN) == LOW) low = true;
    if (!gotAck && ackFlag && ackSeq == seq) { gotAck = true; tAck = micros(); }
    if (gotAck && micros() - tAck > TAIL_US) break;
    if (micros() - t0 > TIMEOUT_US) break;
  }
  if (!gotAck) return 0;
  return low ? 1 : 2;
}

// BOOT button: true once per press
bool bootPressed() {
  static bool last = HIGH;
  bool now = digitalRead(BOOT_BTN);
  bool pressed = (last == HIGH && now == LOW);
  last = now;
  return pressed;
}

void switchMode() {
  mode = (mode == MODE_TEST) ? MODE_SWEEP : MODE_TEST;
  while (digitalRead(BOOT_BTN) == LOW) delay(10);   // wait for release
  tft.fillScreen(ST77XX_BLACK);
}

// ---------- screen ----------
void line(int row, const char *text, uint16_t color = ST77XX_WHITE) {
  char buf[21];
  snprintf(buf, sizeof(buf), "%-20s", text);
  tft.setTextSize(2);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(0, row * 22);
  tft.print(buf);
}

void halt(const char *msg) {
  Serial.println(msg);
  line(5, msg, ST77XX_RED);
  while (true) delay(1000);
}

// ---------- TEST mode ----------
void runTest() {
  int fireOk = 0, hits = 0, silentOk = 0, falses = 0;

  for (int i = 0; i < TEST_ROUNDS; i++) {
    if (bootPressed()) { switchMode(); return; }

    int r = doRound(MSG_FIRE, TEST_FREQ, TEST_DUTY);
    if (r) { fireOk++; if (r == 1) hits++; }
    delay(GAP_MS);

    r = doRound(MSG_SILENT, TEST_FREQ, TEST_DUTY);
    if (r) { silentOk++; if (r == 1) falses++; }
    delay(GAP_MS);
  }

  float bField = NAN;
  if (magOK) {
    sensors_event_t e;
    mag.getEvent(&e);
    bField = sqrtf(e.magnetic.x * e.magnetic.x + e.magnetic.y * e.magnetic.y + e.magnetic.z * e.magnetic.z);
  }
  bool hallClose = digitalRead(HALL_PIN) == LOW;

  int linkPct = (fireOk + silentOk) * 100 / (2 * TEST_ROUNDS);
  char s[32];
  line(0, "TEST 38k  BOOT=sweep", ST77XX_CYAN);

  if (fireOk + silentOk == 0) {
    line(1, "No reply from A", ST77XX_RED);
    line(2, "check A is on and", ST77XX_RED);
    line(3, "running nodeA_tx", ST77XX_RED);
    line(4, "");
    line(5, "");
  } else {
    int hitPct = fireOk ? hits * 100 / fireOk : 0;
    int falsePct = silentOk ? falses * 100 / silentOk : 0;

    snprintf(s, sizeof(s), "IR hit %3d%% %2d/%d", hitPct, hits, fireOk);
    line(1, s, hitPct >= 90 ? ST77XX_GREEN : (hitPct > 0 ? ST77XX_YELLOW : ST77XX_RED));
    snprintf(s, sizeof(s), "false  %3d%% %2d/%d", falsePct, falses, silentOk);
    line(2, s, falsePct <= 5 ? ST77XX_WHITE : ST77XX_RED);
    snprintf(s, sizeof(s), "link %3d%% %4ddBm", linkPct, lastRssi);
    line(3, s, linkPct >= 95 ? ST77XX_WHITE : ST77XX_YELLOW);
    if (magOK) snprintf(s, sizeof(s), "|B|   %7.1f uT", bField);
    else       snprintf(s, sizeof(s), "|B|   (no mag)");
    line(4, s);
    line(5, hallClose ? "hall: MAGNET" : "hall: -", hallClose ? ST77XX_YELLOW : ST77XX_WHITE);
  }

  Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%.1f,%d\n", (unsigned long)millis(), fireOk, hits, silentOk,
                falses, TEST_ROUNDS, lastRssi, bField, hallClose ? 1 : 0);
}

// ---------- SWEEP mode ----------
// bar chart: 12 bars, 20 px per slot, bars from y=24 (100 %) to y=114 (0 %), labels below
const int BAR_TOP = 24, BAR_BOTTOM = 114, SLOT_W = 20, BAR_W = 14;

void drawSweepFrame() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(0, 0);
  char s[32];
  snprintf(s, sizeof(s), "SWEEP duty %d%%", SWEEP_DUTY * 100 / 256);
  tft.print(s);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  for (int i = 0; i < N_SWEEP; i++) {
    tft.setCursor(i * SLOT_W + 4, BAR_BOTTOM + 6);
    tft.print(SWEEP_FREQS[i] / 1000);   // kHz
  }
  tft.drawFastHLine(0, BAR_BOTTOM, 240, ST77XX_WHITE);
}

void drawBar(int i, int pct, bool linkOk) {
  int x = i * SLOT_W + 3;
  tft.fillRect(x, BAR_TOP, BAR_W, BAR_BOTTOM - BAR_TOP, ST77XX_BLACK);
  if (!linkOk) {   // A did not answer at this frequency
    tft.drawLine(x, BAR_BOTTOM - 10, x + BAR_W, BAR_BOTTOM - 1, ST77XX_RED);
    return;
  }
  int h = (BAR_BOTTOM - BAR_TOP) * pct / 100;
  uint16_t c = SWEEP_FREQS[i] == 38000 ? ST77XX_CYAN : (pct >= 90 ? ST77XX_GREEN : ST77XX_YELLOW);
  tft.fillRect(x, BAR_BOTTOM - h, BAR_W, h, c);
}

void markCurrent(int i) {   // small white dot above the bar being measured
  tft.fillRect(0, BAR_TOP - 4, 240, 3, ST77XX_BLACK);
  tft.fillRect(i * SLOT_W + 8, BAR_TOP - 4, 4, 3, ST77XX_WHITE);
}

void runSweep() {
  static bool framed = false;
  static Mode lastMode = MODE_TEST;
  if (!framed || lastMode != MODE_SWEEP) { drawSweepFrame(); framed = true; }
  lastMode = MODE_SWEEP;

  for (int i = 0; i < N_SWEEP; i++) {
    markCurrent(i);
    int ok = 0, hits = 0;
    for (int k = 0; k < SWEEP_ROUNDS; k++) {
      if (bootPressed()) { switchMode(); framed = false; lastMode = MODE_TEST; return; }
      int r = doRound(MSG_FIRE, SWEEP_FREQS[i], SWEEP_DUTY);
      if (r) { ok++; if (r == 1) hits++; }
      delay(GAP_MS);
    }
    int pct = ok ? hits * 100 / ok : 0;
    drawBar(i, pct, ok > 0);
    Serial.printf("sweep,%lu,%d,%d,%d\n", (unsigned long)SWEEP_FREQS[i], SWEEP_DUTY, ok, hits);
  }
  tft.fillRect(0, BAR_TOP - 4, 240, 3, ST77XX_BLACK);
  delay(1000);   // hold the finished chart, then sweep again
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);
  tft.init(135, 240);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  line(0, "Node B (RX)", ST77XX_CYAN);

  pinMode(IR_RX_PIN, INPUT_PULLUP);   // receiver output is active LOW
  pinMode(HALL_PIN, INPUT_PULLUP);    // hall output is LOW when a magnet is close
  pinMode(BOOT_BTN, INPUT_PULLUP);

  Wire.begin();
  magOK = mag.begin_I2C();
  if (magOK) {
    mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag.setDataRate(LIS3MDL_DATARATE_80_HZ);
    mag.setRange(LIS3MDL_RANGE_4_GAUSS);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) halt("ESP-NOW init FAILED");
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.print("Node B (RX, protocol v2) ready. MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("t_ms,fire_ok,ir_hits,silent_ok,false_hits,rounds,rssi,b_uT,hall");
  line(1, "talking to Node A..");
}

void loop() {
  if (mode == MODE_TEST) runTest();
  else runSweep();
}
