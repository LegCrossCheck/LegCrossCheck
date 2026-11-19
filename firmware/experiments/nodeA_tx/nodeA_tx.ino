// CrossCheck - Node A (TRANSMITTER)   *** flash this onto the Feather WITHOUT a screen ***
// Board: Adafruit Feather ESP32-S3 (no TFT)
//
// Wiring
//   IR LED driver: D9 -> 1k -> PN2222 base; 3V -> 100R -> IR LED (+, long leg) -> LED (-) -> PN2222 C; E -> GND
//
// What it does
//   Waits for ESP-NOW commands from Node B:
//     FIRE   -> sends one IR burst (20 carrier cycles) on D9 at the carrier frequency and
//               duty cycle Node B asks for (normally 38 kHz, 50 %), then replies ACK
//     SILENT -> does NOT fire, just replies ACK (lets Node B count false triggers)
//   Node B changes the frequency during its sweep mode; this firmware follows automatically.
//   The red LED (D13) flickers while commands are arriving.
//   Serial (115200) prints a status line every 2 s.
//
// Protocol v2 - Node B must run the matching nodeB_rx (same MAGIC). With mismatched
// versions the boards ignore each other and Node B shows "No reply from A".

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---- must match Node B ----
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

// ---- IR ----
const uint8_t  IR_TX_PIN  = 9;
const uint8_t  LED_PIN    = 13;
const int      BURST_CYC  = 20;      // carrier cycles per burst (TSSP77038 needs >= 10)
uint32_t       curFreq    = 38000;

volatile bool     pending = false;
volatile uint8_t  pendType = 0;
volatile uint16_t pendSeq = 0;
volatile uint8_t  pendDuty = 128;
volatile uint32_t pendFreq = 38000;

uint32_t nFire = 0, nSilent = 0;
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

uint32_t actualFreq() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcReadFreq(IR_TX_PIN);
#else
  return (uint32_t)ledcReadFreq(0);
#endif
}

void carrierDuty(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(IR_TX_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}

// ---------- ESP-NOW receive ----------
void handleMsg(const uint8_t *data, int len) {
  if (len != sizeof(Msg)) return;
  Msg m;
  memcpy(&m, data, sizeof(m));
  if (m.magic != MAGIC) return;
  if (m.type != MSG_FIRE && m.type != MSG_SILENT) return;
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

  Serial.print("Node A (TX, protocol v2) ready. MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("carrier %lu Hz (actual %lu Hz) on D%d\n",
                (unsigned long)curFreq, (unsigned long)actualFreq(), IR_TX_PIN);
}

void loop() {
  if (pending) {
    uint8_t  type = pendType;
    uint16_t seq  = pendSeq;
    uint8_t  duty = pendDuty;
    uint32_t freq = pendFreq;
    pending = false;

    bool fired = (type == MSG_FIRE);
    if (fired) {
      setFreq(freq);
      uint32_t burstUs = BURST_CYC * 1000000UL / curFreq;
      carrierDuty(duty);
      delayMicroseconds(burstUs);
      carrierDuty(0);
      nFire++;
    } else {
      nSilent++;
    }

    Msg ack = {MAGIC, MSG_ACK, seq, (uint8_t)fired, duty, curFreq};
    esp_now_send(BCAST, (const uint8_t *)&ack, sizeof(ack));

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastCmdMs = millis();
  }

  // LED off when Node B is not talking to us
  if (millis() - lastCmdMs > 1000) digitalWrite(LED_PIN, LOW);

  static uint32_t nextPrint = 0;
  if (millis() >= nextPrint) {
    nextPrint = millis() + 2000;
    Serial.printf("fired %lu, silent %lu, carrier %lu Hz (actual %lu Hz); %s\n",
                  (unsigned long)nFire, (unsigned long)nSilent, (unsigned long)curFreq,
                  (unsigned long)actualFreq(),
                  millis() - lastCmdMs < 1000 ? "Node B connected" : "waiting for Node B...");
  }
}
