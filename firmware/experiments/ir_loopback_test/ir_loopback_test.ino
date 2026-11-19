// CrossCheck - IR loopback test (one board)
// Board: Adafruit Feather ESP32-S3 TFT
//
// Wiring
//   IR LED driver : D9 -> 1k -> PN2222 base; 3V -> 100R -> IR LED (+, long leg) -> LED (-) -> PN2222 C; E -> GND
//   Receiver      : TSSP77038 board  Sig (white) -> D6, V+ (red) -> 3V, GND (black) -> GND
//   Optional      : more drive channels on D10 / D11, same circuit with other resistors
//
// What it does
//   Sends short 38 kHz bursts (20 carrier cycles) on the selected IR pin and checks whether
//   the receiver output goes LOW. Between bursts it also listens with the LED OFF, to count
//   false triggers (sunlight, lamps, noise).
//   Every 50 bursts it shows:
//     hit  = % of bursts the receiver saw        (want ~100 % when LED faces the receiver)
//     false= % of silent windows that triggered  (want ~0 %)
//   BOOT button cycles the transmit pin D9 -> D10 -> D11.
//   Serial (115200) prints CSV: t_ms,tx_pin,hits,false_hits,bursts

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

const uint8_t  TX_PINS[]   = {9, 10, 11};
const char    *TX_NAMES[]  = {"D9", "D10", "D11"};
const int      N_TX        = 3;
const uint8_t  RX_PIN      = 6;
const uint8_t  BOOT_BTN    = 0;

const uint32_t CARRIER_HZ  = 38000;
const int      BURST_CYC   = 20;                                   // >= 10 cycles needed by TSSP77038
const uint32_t BURST_US    = BURST_CYC * 1000000UL / CARRIER_HZ;   // ~526 us
const uint32_t LISTEN_US   = 800;   // receiver output lags the burst by up to ~13 cycles (~340 us)
const uint32_t GAP_US      = 2000;  // quiet time between windows
const int      BURSTS      = 50;    // bursts per report

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
int txIdx = 0;

// ---------- 38 kHz carrier on/off (works on ESP32 Arduino core 2.x and 3.x) ----------
void carrierSetup() {
  for (int i = 0; i < N_TX; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(TX_PINS[i], CARRIER_HZ, 8);
    ledcWrite(TX_PINS[i], 0);
#else
    ledcSetup(i, CARRIER_HZ, 8);
    ledcAttachPin(TX_PINS[i], i);
    ledcWrite(i, 0);
#endif
  }
}

void carrier(int idx, bool on) {
  uint32_t duty = on ? 128 : 0;   // 50 % duty while on, fully off (transistor off) otherwise
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(TX_PINS[idx], duty);
#else
  ledcWrite(idx, duty);
#endif
}

// Watch the receiver for `us` microseconds; true if it went LOW at any point
bool sawLow(uint32_t us) {
  bool low = false;
  uint32_t t0 = micros();
  while (micros() - t0 < us) {
    if (digitalRead(RX_PIN) == LOW) low = true;
  }
  return low;
}

// ---------- screen ----------
void line(int row, const char *text, uint16_t color = ST77XX_WHITE, uint8_t size = 2) {
  char buf[21];
  snprintf(buf, sizeof(buf), "%-20s", text);
  tft.setTextSize(size);
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(0, row * 22);
  tft.print(buf);
}

void showResult(int hits, int falseHits) {
  int hitPct = hits * 100 / BURSTS;
  int falsePct = falseHits * 100 / BURSTS;
  char s[32];

  snprintf(s, sizeof(s), "IR test  TX %s", TX_NAMES[txIdx]);
  line(0, s, ST77XX_CYAN);
  snprintf(s, sizeof(s), "hit   %3d%% %2d/%d", hitPct, hits, BURSTS);
  line(1, s, hitPct >= 90 ? ST77XX_GREEN : (hitPct > 0 ? ST77XX_YELLOW : ST77XX_RED));
  snprintf(s, sizeof(s), "false %3d%% %2d/%d", falsePct, falseHits, BURSTS);
  line(2, s, falsePct <= 5 ? ST77XX_WHITE : ST77XX_RED);

  // hit-rate bar
  int w = map(hitPct, 0, 100, 0, 236);
  tft.fillRect(0, 70, 240, 18, ST77XX_BLACK);
  tft.drawRect(0, 70, 238, 18, ST77XX_WHITE);
  tft.fillRect(1, 71, w, 16, hitPct >= 90 ? ST77XX_GREEN : ST77XX_YELLOW);

  line(5, "BOOT = next TX pin", ST77XX_WHITE);
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

  pinMode(RX_PIN, INPUT_PULLUP);   // receiver output is active LOW
  pinMode(BOOT_BTN, INPUT_PULLUP);
  carrierSetup();

  line(0, "IR loopback test", ST77XX_CYAN);
  Serial.printf("burst %d cycles = %lu us\n", BURST_CYC, (unsigned long)BURST_US);
  Serial.println("t_ms,tx_pin,hits,false_hits,bursts");
}

void loop() {
  int hits = 0, falseHits = 0;

  for (int i = 0; i < BURSTS; i++) {
    // 1) burst ON: did the receiver see it?
    carrier(txIdx, true);
    bool seen = sawLow(BURST_US);
    carrier(txIdx, false);
    seen |= sawLow(LISTEN_US);
    if (seen) hits++;
    delayMicroseconds(GAP_US);

    // 2) LED OFF for the same time: anything here is a false trigger
    if (sawLow(BURST_US + LISTEN_US)) falseHits++;
    delayMicroseconds(GAP_US);

    // BOOT button: switch transmit pin
    if (digitalRead(BOOT_BTN) == LOW) {
      txIdx = (txIdx + 1) % N_TX;
      while (digitalRead(BOOT_BTN) == LOW) delay(10);   // wait for release
      tft.fillScreen(ST77XX_BLACK);
      return;   // start a fresh measurement on the new pin
    }
  }

  showResult(hits, falseHits);
  Serial.printf("%lu,%s,%d,%d,%d\n", (unsigned long)millis(), TX_NAMES[txIdx], hits, falseHits, BURSTS);
}
