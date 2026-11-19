// CrossCheck - Node B   *** flash onto the Feather WITH the screen ***
// Board: Adafruit Feather ESP32-S3 TFT
// Both nodes can now SEND and RECEIVE IR, so either leg can be the one on top.
//
// Wiring (Node B)
//   IR receiver   : TSSP77038 board  Sig (white) -> D9, V+ (red) -> 3V, GND (black) -> GND
//   IR LED driver : D10 -> 1k -> PN2222 base; 3V -> 100R -> IR LED (+) -> LED (-) -> PN2222 C; E -> GND
//   DRV5032 hall  : OUT -> D6, VIN -> 3V, GND -> GND              (optional)
//   9-DoF IMU     : STEMMA QT cable (LIS3MDL magnetometer used)    (optional)
//   LED and receiver side by side, both facing outward (same direction).
//
// Node B is the boss; Node A follows its ESP-NOW commands. Press BOOT to switch modes.
//
// 1) TEST (default) - both directions at 38 kHz, 50 % power. Each report = 25 rounds of:
//      A->B fire   : A fires, B's receiver should see it        -> "A>B hit"
//      A->B silent : A does not fire, B's receiver should not   -> "f" (false)
//      B->A fire   : B fires, A's receiver should see it        -> "B>A hit"
//      B->A silent : B does not fire, A's receiver should not   -> "f"
//    plus link % / RSSI, magnetic field |B| and the hall switch.
//    Crossing either leg over the other should light up at least one direction.
//
// 2) SWEEP - carrier frequency sweep, direction A->B only (bar chart, 30 ... 56 kHz).
//    Power starts at 50 %; type 1-6 in Serial Monitor to change it (1 = 50 % ... 6 = 1 %).
//
// Serial (115200):
//   TEST : t_ms,ab_ok,ab_hits,ab_sok,ab_false,ba_ok,ba_hits,ba_sok,ba_false,rssi,b_uT,hall
//   SWEEP: sweep,freq_hz,duty,fire_ok,ir_hits
//
// Protocol v3 - Node A must run the matching nodeA sketch (same MAGIC).

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
const uint8_t MAGIC = 0xC7;   // protocol v3
enum : uint8_t { MSG_FIRE = 1, MSG_SILENT = 2, MSG_ACK = 3, MSG_LISTEN = 4, MSG_READY = 5, MSG_RESULT = 6 };
struct __attribute__((packed)) Msg {
  uint8_t  magic;
  uint8_t  type;
  uint16_t seq;
  uint8_t  flag;   // ACK: fired, RESULT: receiver saw IR
  uint8_t  duty;   // 0-255, 128 = 50 %
  uint32_t freq;   // carrier frequency in Hz
};
const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---- pins / timing ----
const uint8_t  IR_RX_PIN  = 9;
const uint8_t  IR_TX_PIN  = 10;
const uint8_t  HALL_PIN   = 6;
const uint8_t  BOOT_BTN   = 0;
const uint32_t TIMEOUT_US = 20000;   // give up waiting for a reply from A after 20 ms
const uint32_t TAIL_US    = 1000;    // keep watching B's receiver 1 ms after A's ACK
const uint32_t GAP_MS     = 8;       // pause between commands
const int      BURST_CYC  = 20;

// ---- test mode ----
const int      TEST_ROUNDS = 25;     // each report: 25 x (A>B fire, A>B silent, B>A fire, B>A silent)
const uint32_t TEST_FREQ   = 38000;
const uint8_t  TEST_DUTY   = 128;    // 50 %

// ---- sweep mode ----
const uint32_t SWEEP_FREQS[] = {30000, 33000, 35000, 36000, 37000, 38000,
                                39000, 40000, 42000, 45000, 50000, 56000};
const int      N_SWEEP      = sizeof(SWEEP_FREQS) / sizeof(SWEEP_FREQS[0]);
const int      SWEEP_ROUNDS = 40;
// sweep power: type a digit in Serial Monitor - 1 = 50 %  2 = 25 %  3 = 12 %  4 = 6 %  5 = 3 %  6 = 1 %
const uint8_t  DUTY_LEVELS[] = {128, 64, 32, 16, 8, 3};
uint8_t        sweepDuty     = 128;

Adafruit_ST7789  tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_LIS3MDL mag;
bool magOK = false;

enum Mode { MODE_TEST, MODE_SWEEP };
Mode mode = MODE_TEST;

// replies from A (written in the Wi-Fi callback)
volatile bool     ackFlag = false,    readyFlag = false,  resultFlag = false;
volatile uint16_t ackSeq = 0,         readySeq = 0,       resultSeq = 0;
volatile uint8_t  resultSeen = 0;
volatile int      lastRssi = 0;
uint16_t seqCounter = 0;

// ---------- B's own IR transmitter ----------
void carrierSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(IR_TX_PIN, TEST_FREQ, 8);
  ledcWrite(IR_TX_PIN, 0);
#else
  ledcSetup(0, TEST_FREQ, 8);
  ledcAttachPin(IR_TX_PIN, 0);
  ledcWrite(0, 0);
#endif
}

void carrierDuty(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(IR_TX_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}

void burstB(uint8_t duty) {
  carrierDuty(duty);
  delayMicroseconds(BURST_CYC * 1000000UL / TEST_FREQ);
  carrierDuty(0);
}

// ---------- ESP-NOW receive ----------
bool handleMsg(const uint8_t *data, int len) {   // true if it was a reply from Node A
  if (len != sizeof(Msg)) return false;
  Msg m;
  memcpy(&m, data, sizeof(m));
  if (m.magic != MAGIC) return false;
  switch (m.type) {
    case MSG_ACK:    ackSeq = m.seq;    ackFlag = true;    return true;
    case MSG_READY:  readySeq = m.seq;  readyFlag = true;  return true;
    case MSG_RESULT: resultSeq = m.seq; resultSeen = m.flag; resultFlag = true; return true;
  }
  return false;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (handleMsg(data, len) && info && info->rx_ctrl) lastRssi = info->rx_ctrl->rssi;
}
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) { handleMsg(data, len); }
#endif

void sendCmd(uint8_t type, uint16_t seq, uint32_t freq, uint8_t duty) {
  Msg m = {MAGIC, type, seq, 0, duty, freq};
  esp_now_send(BCAST, (const uint8_t *)&m, sizeof(m));
}

// Direction A -> B. type = FIRE or SILENT.
// Returns 0 = A did not answer, 1 = answered and B's receiver went LOW, 2 = answered, stayed HIGH
int roundAB(uint8_t type, uint32_t freq, uint8_t duty) {
  uint16_t seq = seqCounter++;
  ackFlag = false;
  sendCmd(type, seq, freq, duty);

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

// Direction B -> A. fire = whether B actually sends a burst.
// Returns 0 = A did not answer, 1 = A's receiver saw IR, 2 = A saw nothing
int roundBA(bool fire, uint8_t duty) {
  uint16_t seq = seqCounter++;
  readyFlag = false;
  resultFlag = false;
  sendCmd(MSG_LISTEN, seq, TEST_FREQ, duty);

  uint32_t t0 = micros();
  while (!(readyFlag && readySeq == seq)) {          // wait until A is listening
    if (micros() - t0 > TIMEOUT_US) return 0;
  }
  if (fire) burstB(duty);

  t0 = micros();
  while (!(resultFlag && resultSeq == seq)) {        // wait for A's verdict
    if (micros() - t0 > TIMEOUT_US) return 0;
  }
  return resultSeen ? 1 : 2;
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
  while (digitalRead(BOOT_BTN) == LOW) delay(10);
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

uint16_t hitColor(int pct) {
  return pct >= 90 ? ST77XX_GREEN : (pct > 0 ? ST77XX_YELLOW : ST77XX_RED);
}

// ---------- TEST mode ----------
void runTest() {
  int abOk = 0, abHits = 0, abSok = 0, abFalse = 0;
  int baOk = 0, baHits = 0, baSok = 0, baFalse = 0;

  for (int i = 0; i < TEST_ROUNDS; i++) {
    if (bootPressed()) { switchMode(); return; }
    int r;
    r = roundAB(MSG_FIRE, TEST_FREQ, TEST_DUTY);   if (r) { abOk++;  if (r == 1) abHits++; }  delay(GAP_MS);
    r = roundAB(MSG_SILENT, TEST_FREQ, TEST_DUTY); if (r) { abSok++; if (r == 1) abFalse++; } delay(GAP_MS);
    r = roundBA(true, TEST_DUTY);                  if (r) { baOk++;  if (r == 1) baHits++; }  delay(GAP_MS);
    r = roundBA(false, TEST_DUTY);                 if (r) { baSok++; if (r == 1) baFalse++; } delay(GAP_MS);
  }

  float bField = NAN;
  if (magOK) {
    sensors_event_t e;
    mag.getEvent(&e);
    bField = sqrtf(e.magnetic.x * e.magnetic.x + e.magnetic.y * e.magnetic.y + e.magnetic.z * e.magnetic.z);
  }
  bool hallClose = digitalRead(HALL_PIN) == LOW;

  int total = abOk + abSok + baOk + baSok;
  int linkPct = total * 100 / (4 * TEST_ROUNDS);
  char s[32];
  line(0, "TEST  BOOT=sweep", ST77XX_CYAN);

  if (total == 0) {
    line(1, "No reply from A", ST77XX_RED);
    line(2, "check A is on and", ST77XX_RED);
    line(3, "running nodeA", ST77XX_RED);
    line(4, "");
    line(5, "");
  } else {
    int abPct = abOk ? abHits * 100 / abOk : 0,  abF = abSok ? abFalse * 100 / abSok : 0;
    int baPct = baOk ? baHits * 100 / baOk : 0,  baF = baSok ? baFalse * 100 / baSok : 0;

    snprintf(s, sizeof(s), "A>B hit%3d%% f%3d%%", abPct, abF);
    line(1, s, abF > 5 ? ST77XX_RED : hitColor(abPct));
    snprintf(s, sizeof(s), "B>A hit%3d%% f%3d%%", baPct, baF);
    line(2, s, baF > 5 ? ST77XX_RED : hitColor(baPct));
    snprintf(s, sizeof(s), "link %3d%% %4ddBm", linkPct, lastRssi);
    line(3, s, linkPct >= 95 ? ST77XX_WHITE : ST77XX_YELLOW);
    if (magOK) snprintf(s, sizeof(s), "|B|   %7.1f uT", bField);
    else       snprintf(s, sizeof(s), "|B|   (no mag)");
    line(4, s);
    line(5, hallClose ? "hall: MAGNET" : "hall: -", hallClose ? ST77XX_YELLOW : ST77XX_WHITE);
  }

  Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%d\n", (unsigned long)millis(),
                abOk, abHits, abSok, abFalse, baOk, baHits, baSok, baFalse,
                lastRssi, bField, hallClose ? 1 : 0);
}

// ---------- SWEEP mode (A -> B) ----------
const int BAR_TOP = 24, BAR_BOTTOM = 114, SLOT_W = 20, BAR_W = 14;

void drawSweepFrame() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(0, 0);
  char s[32];
  snprintf(s, sizeof(s), "SWEEP duty %d%%", (sweepDuty * 100 + 128) / 256);
  tft.print(s);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  for (int i = 0; i < N_SWEEP; i++) {
    tft.setCursor(i * SLOT_W + 4, BAR_BOTTOM + 6);
    tft.print(SWEEP_FREQS[i] / 1000);
  }
  tft.drawFastHLine(0, BAR_BOTTOM, 240, ST77XX_WHITE);
}

void drawBar(int i, int pct, bool linkOk) {
  int x = i * SLOT_W + 3;
  tft.fillRect(x, BAR_TOP, BAR_W, BAR_BOTTOM - BAR_TOP, ST77XX_BLACK);
  if (!linkOk) {
    tft.drawLine(x, BAR_BOTTOM - 10, x + BAR_W, BAR_BOTTOM - 1, ST77XX_RED);
    return;
  }
  int h = (BAR_BOTTOM - BAR_TOP) * pct / 100;
  uint16_t c = SWEEP_FREQS[i] == 38000 ? ST77XX_CYAN : (pct >= 90 ? ST77XX_GREEN : ST77XX_YELLOW);
  tft.fillRect(x, BAR_BOTTOM - h, BAR_W, h, c);
}

void markCurrent(int i) {
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
      if (Serial.available()) {
        char c = Serial.read();
        if (c >= '1' && c <= '6') {
          sweepDuty = DUTY_LEVELS[c - '1'];
          Serial.printf("sweep duty -> %d/256\n", sweepDuty);
          framed = false;
          return;
        }
      }
      int r = roundAB(MSG_FIRE, SWEEP_FREQS[i], sweepDuty);
      if (r) { ok++; if (r == 1) hits++; }
      delay(GAP_MS);
    }
    int pct = ok ? hits * 100 / ok : 0;
    drawBar(i, pct, ok > 0);
    Serial.printf("sweep,%lu,%d,%d,%d\n", (unsigned long)SWEEP_FREQS[i], sweepDuty, ok, hits);
  }
  tft.fillRect(0, BAR_TOP - 4, 240, 3, ST77XX_BLACK);
  delay(1000);
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
  line(0, "Node B", ST77XX_CYAN);

  pinMode(IR_RX_PIN, INPUT_PULLUP);
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(BOOT_BTN, INPUT_PULLUP);
  carrierSetup();

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

  Serial.print("Node B (TX+RX, protocol v3) ready. MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("t_ms,ab_ok,ab_hits,ab_sok,ab_false,ba_ok,ba_hits,ba_sok,ba_false,rssi,b_uT,hall");
  line(1, "talking to Node A..");
}

void loop() {
  if (mode == MODE_TEST) runTest();
  else runSweep();
}
