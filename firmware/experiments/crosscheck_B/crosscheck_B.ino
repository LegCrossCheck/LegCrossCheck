// CrossCheck - symmetric IR node, NODE B
// Board: Adafruit Feather ESP32-S3 (no screen). Library: Adafruit NeoPixel.
//
// The A and B sketches are identical except for the NODE line below.
//
// Wiring (same on both nodes)
//   IR LED driver : D9 -> 1k -> 2222 base; 3V -> 100R -> IR LED (+) -> LED (-) -> 2222 collector; emitter -> GND
//   IR receiver   : TSSP77038 board  Sig -> D5 (IR_RX_PIN), V+ -> 3V, GND -> GND
//
// How it works (no radio needed)
//   - Every ~50 ms (plus a random jitter so A and B rarely collide) this node fires one 38 kHz
//     IR burst. A's bursts are 20 carrier cycles long, B's are 40, so each receiver can tell
//     who sent a pulse from how long the receiver output stays LOW.
//   - While this node is firing (and 1.5 ms after) it ignores its own receiver, so its own
//     light bouncing back is never counted.
//   - Every LOW pulse on the receiver is timed. Only pulses with the OTHER node's length count.
//
// Onboard RGB LED (NeoPixel)
//   BLUE        : nothing received from the other node
//   RED flash   : each pulse received from the other node (keeps flashing while pulses keep coming)
//   dim RED     : a pulse arrived in the last 300 ms
// The small red LED (D13) also toggles on every received pulse.
// Serial 115200: once a second, pulses received / rejected / sent.

#define NODE 'B'          // <-- 'A' in crosscheck_A, 'B' in crosscheck_B

#include <Adafruit_NeoPixel.h>

// ---------------- pins ----------------
const uint8_t IR_TX_PIN = 9;     // IR LED driver
const uint8_t IR_RX_PIN = 5;     // TSSP77038 Sig   <-- change if your receiver is on another pin
const uint8_t RED_LED   = 13;
// Onboard RGB LED of the Feather ESP32-S3 (no screen). Hard-coded so it still works when a
// different Feather variant (e.g. the TFT one, which uses another power pin) is selected in the IDE.
const uint8_t NEO_PIN   = 33;
const uint8_t NEO_PWR   = 21;    // must be HIGH to power the RGB LED

// ---------------- IR timing ----------------
const uint32_t CARRIER_HZ   = 38000;
const uint8_t  DUTY         = 128;       // 50 % = full power (0..255)
const int      CYC_A        = 20;        // A's burst length in carrier cycles (~526 us)
const int      CYC_B        = 40;        // B's burst length in carrier cycles (~1053 us)
const uint32_t PERIOD_MS    = 50;        // fire every 50 ms ...
const uint32_t JITTER_MS    = 20;        // ... plus 0-20 ms random
const uint32_t BLANK_US     = 1500;      // ignore own receiver this long after own burst

// accepted LOW-pulse width at the receiver, in carrier cycles (receiver output is
// roughly the burst length, give or take a few cycles)
const int A_MIN = 12, A_MAX = 28;        // 316 - 737 us
const int B_MIN = 32, B_MAX = 50;        // 842 - 1316 us

#if NODE == 'A'
const int MY_CYC = CYC_A;
const int OTHER_MIN = B_MIN, OTHER_MAX = B_MAX;
const char OTHER = 'B';
#else
const int MY_CYC = CYC_B;
const int OTHER_MIN = A_MIN, OTHER_MAX = A_MAX;
const char OTHER = 'A';
#endif

const uint32_t US_PER_CYC  = 1000000UL / CARRIER_HZ;          // ~26 us
const uint32_t OTHER_MIN_US = OTHER_MIN * US_PER_CYC;
const uint32_t OTHER_MAX_US = OTHER_MAX * US_PER_CYC;

// ---------------- LED display ----------------
const uint32_t FLASH_MS = 40;    // bright red after each pulse
const uint32_t HOLD_MS  = 300;   // dim red until this long without pulses

Adafruit_NeoPixel pixel(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- shared with the interrupt ----------------
volatile bool     txActive  = false;
volatile uint32_t txEndUs   = 0;
volatile uint32_t fallUs    = 0;
volatile bool     inPulse   = false;
volatile uint32_t nGood     = 0;     // pulses from the other node
volatile uint32_t nBadWidth = 0;     // LOW pulses with the wrong length (noise, own echo...)
volatile uint32_t lastWidth = 0;

uint32_t nSent = 0;

// Receiver output is active LOW: falling edge = pulse start, rising edge = pulse end
void IRAM_ATTR onRxEdge() {
  uint32_t now = micros();
  bool blanked = txActive || (now - txEndUs < BLANK_US);
  if (digitalRead(IR_RX_PIN) == LOW) {
    if (blanked) { inPulse = false; return; }
    fallUs = now;
    inPulse = true;
  } else {
    if (!inPulse) return;
    inPulse = false;
    uint32_t w = now - fallUs;
    lastWidth = w;
    if (w >= OTHER_MIN_US && w <= OTHER_MAX_US) nGood++;
    else nBadWidth++;
  }
}

// ---------------- 38 kHz carrier (ESP32 Arduino core 2.x and 3.x) ----------------
void carrierSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(IR_TX_PIN, CARRIER_HZ, 8);
  ledcWrite(IR_TX_PIN, 0);
#else
  ledcSetup(0, CARRIER_HZ, 8);
  ledcAttachPin(IR_TX_PIN, 0);
  ledcWrite(0, 0);
#endif
}

void carrier(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(IR_TX_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}

void fireBurst() {
  txActive = true;
  carrier(DUTY);
  delayMicroseconds(MY_CYC * US_PER_CYC);
  carrier(0);
  txEndUs = micros();
  txActive = false;
  nSent++;
}

// ---------------- setup / loop ----------------
void setColor(uint8_t r, uint8_t g, uint8_t b) {
  static uint32_t last = 0xFFFFFFFF;
  uint32_t c = pixel.Color(r, g, b);
  if (c == last) return;      // only talk to the LED when the colour changes
  last = c;
  pixel.setPixelColor(0, c);
  pixel.show();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(NEO_PWR, OUTPUT);
  digitalWrite(NEO_PWR, HIGH);
  delay(10);                 // let the LED power up before the first write
  pixel.begin();
  pixel.setBrightness(40);
  setColor(0, 0, 255);

  pinMode(RED_LED, OUTPUT);
  pinMode(IR_RX_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IR_RX_PIN), onRxEdge, CHANGE);
  carrierSetup();
  randomSeed(esp_random());

  Serial.printf("CrossCheck node %c: sends %d-cycle bursts on D%d, listens on D%d for node %c (%lu-%lu us)\n",
                NODE, MY_CYC, IR_TX_PIN, IR_RX_PIN, OTHER,
                (unsigned long)OTHER_MIN_US, (unsigned long)OTHER_MAX_US);
}

void loop() {
  static uint32_t nextFire = 0;
  static uint32_t lastGood = 0, seenGood = 0;
  static uint32_t lastPulseMs = 0, flashUntil = 0;
  uint32_t now = millis();

  // 1) fire our own burst
  if (now >= nextFire) {
    fireBurst();
    nextFire = now + PERIOD_MS + random(JITTER_MS + 1);
  }

  // 2) new pulses from the other node?
  uint32_t g = nGood;
  if (g != seenGood) {
    seenGood = g;
    lastPulseMs = now;
    flashUntil = now + FLASH_MS;
    digitalWrite(RED_LED, !digitalRead(RED_LED));
  }

  // 3) LED colour
  if (now < flashUntil)                       setColor(255, 0, 0);   // bright red: pulse just arrived
  else if (lastPulseMs && now - lastPulseMs < HOLD_MS) setColor(60, 0, 0);    // dim red: still receiving
  else { setColor(0, 0, 255); digitalWrite(RED_LED, LOW); }          // blue: nothing

  // 4) once a second: statistics
  static uint32_t nextPrint = 0;
  if (now >= nextPrint) {
    nextPrint = now + 1000;
    static uint32_t prevGood = 0, prevBad = 0, prevSent = 0;
    uint32_t good = nGood, bad = nBadWidth;
    Serial.printf("node %c | from %c: %lu/s | wrong length: %lu/s (last %lu us) | sent: %lu/s\n",
                  NODE, OTHER, (unsigned long)(good - prevGood), (unsigned long)(bad - prevBad),
                  (unsigned long)lastWidth, (unsigned long)(nSent - prevSent));
    prevGood = good; prevBad = bad; prevSent = nSent;
  }
}
