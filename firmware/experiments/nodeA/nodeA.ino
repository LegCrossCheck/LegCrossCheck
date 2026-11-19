// CrossCheck - Node A   *** flash onto the Feather WITHOUT a screen ***
// Board: Adafruit Feather ESP32-S3 (no TFT)
// Both nodes can now SEND and RECEIVE IR, so either leg can be the one on top.
//
// Wiring (Node A)
//   IR LED driver : D9 -> 1k -> PN2222 base; 3V -> 100R -> IR LED (+) -> LED (-) -> PN2222 C; E -> GND
//   IR receiver   : TSSP77038 board  Sig (white) -> D6, V+ (red) -> 3V, GND (black) -> GND
//   LED and receiver side by side, both facing outward (same direction).
//
// Node A is a follower. Node B (screen) sends ESP-NOW commands:
//   FIRE   -> A sends one 38 kHz IR burst on D9, replies ACK            (direction A -> B)
//   SILENT -> A does not fire, replies ACK                             (A -> B false check)
//   LISTEN -> A replies READY, watches its receiver (D6) for a short
//             window while B fires (or not), then replies RESULT(seen) (direction B -> A)
// Red LED (D13) flickers while commands arrive. Serial 115200 prints status every 2 s.
//
// Protocol v3 - Node B must run the matching nodeB sketch (same MAGIC).

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---- must match Node B ----
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
const uint8_t  IR_TX_PIN  = 9;
const uint8_t  IR_RX_PIN  = 6;
const uint8_t  LED_PIN    = 13;
const int      BURST_CYC  = 20;      // carrier cycles per burst (TSSP77038 needs >= 10)
const uint32_t LISTEN_US  = 8000;    // how long A watches its receiver after READY
uint32_t       curFreq    = 38000;

volatile bool     pending = false;
volatile uint8_t  pendType = 0;
volatile uint16_t pendSeq = 0;
volatile uint8_t  pendDuty = 128;
volatile uint32_t pendFreq = 38000;

uint32_t nFire = 0, nSilent = 0, nListen = 0, nSeen = 0;
uint32_t lastCmdMs = 0;

// ---------- carrier (ESP32 Arduino core 2.x and 3.x) ----------
void carrierSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(IR_TX_PIN, curFreq, 8);
  ledcWrite(IR_TX_PIN, 0);
#else
  ledcSetup(0, curFreq, 8);
  ledcAttachPin(IR_TX_PIN, 0);
  ledcWrite(0, 0);
#endif
}

void setFreq(uint32_t f) {
  if (f == curFreq || f < 10000 || f > 100000) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcChangeFrequency(IR_TX_PIN, f, 8);
#else
  ledcChangeFrequency(0, f, 8);
#endif
  curFreq = f;
}

void carrierDuty(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(IR_TX_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}

void send(uint8_t type, uint16_t seq, uint8_t flag, uint8_t duty) {
  Msg m = {MAGIC, type, seq, flag, duty, curFreq};
  esp_now_send(BCAST, (const uint8_t *)&m, sizeof(m));
}

// ---------- ESP-NOW receive ----------
void handleMsg(const uint8_t *data, int len) {
  if (len != sizeof(Msg)) return;
  Msg m;
  memcpy(&m, data, sizeof(m));
  if (m.magic != MAGIC) return;
  if (m.type != MSG_FIRE && m.type != MSG_SILENT && m.type != MSG_LISTEN) return;
  pendType = m.type;
  pendSeq = m.seq;
  pendDuty = m.duty;
  pendFreq = m.freq;
  pending = true;   // handled in loop(), not inside the Wi-Fi callback
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { handleMsg(data, len); }
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) { handleMsg(data, len); }
#endif

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(LED_PIN, OUTPUT);
  pinMode(IR_RX_PIN, INPUT_PULLUP);   // receiver output is active LOW
  carrierSetup();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED");
    while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }   // fast blink = error
  }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.print("Node A (TX+RX, protocol v3) ready. MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  if (pending) {
    uint8_t  type = pendType;
    uint16_t seq  = pendSeq;
    uint8_t  duty = pendDuty;
    uint32_t freq = pendFreq;
    pending = false;

    if (type == MSG_FIRE) {
      setFreq(freq);
      uint32_t burstUs = BURST_CYC * 1000000UL / curFreq;
      carrierDuty(duty);
      delayMicroseconds(burstUs);
      carrierDuty(0);
      send(MSG_ACK, seq, 1, duty);
      nFire++;
    } else if (type == MSG_SILENT) {
      send(MSG_ACK, seq, 0, duty);
      nSilent++;
    } else if (type == MSG_LISTEN) {
      send(MSG_READY, seq, 0, duty);       // tell B to fire now
      bool seen = false;
      uint32_t t0 = micros();
      while (micros() - t0 < LISTEN_US) {
        if (digitalRead(IR_RX_PIN) == LOW) seen = true;
      }
      send(MSG_RESULT, seq, seen ? 1 : 0, duty);
      nListen++;
      if (seen) nSeen++;
    }

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastCmdMs = millis();
  }

  if (millis() - lastCmdMs > 1000) digitalWrite(LED_PIN, LOW);

  static uint32_t nextPrint = 0;
  if (millis() >= nextPrint) {
    nextPrint = millis() + 2000;
    Serial.printf("fired %lu, silent %lu, listened %lu (saw IR %lu); %s\n",
                  (unsigned long)nFire, (unsigned long)nSilent, (unsigned long)nListen,
                  (unsigned long)nSeen,
                  millis() - lastCmdMs < 1000 ? "Node B connected" : "waiting for Node B...");
  }
}
