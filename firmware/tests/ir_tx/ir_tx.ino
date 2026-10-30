// Week-1 test 1, Node A: IR burst sender. Flash to Node A; run ir_rx on Node B.
//
// Cycles through 4 blocks: off, low, mid, high drive. Before each block it
// tells Node B over ESP-NOW which level is coming, sends BURSTS_PER_LEVEL
// bursts of 38 kHz, then tells Node B the block is over. Node B counts what
// its receivers saw and prints the hit rate. The "off" block sends nothing,
// so it measures false hits from room light.

#include <WiFi.h>
#include <esp_now.h>

// D9 low (220 ohm, ~8 mA), D10 mid (100 ohm, ~18 mA), D11 high (3 x 100 ohm, ~55 mA)
const int IR_PINS[3] = {9, 10, 11};
const char *LEVEL_NAMES[4] = {"off", "low", "mid", "high"};

const uint32_t CARRIER_HZ = 38000;
const uint32_t BURST_US = 600;   // ~23 carrier cycles, inside the receivers' 10-70 cycle limit
const uint32_t GAP_US = 1400;    // receivers need a gap to recover between bursts
const uint16_t BURSTS_PER_LEVEL = 50;

const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Keep in sync with ir_rx.ino
const uint32_t MAGIC = 0xC0FFEE02;
enum : uint8_t { IR_START = 1, IR_END = 2 };
struct IrMsg {
  uint32_t magic;
  uint32_t seq;
  uint8_t type;
  uint8_t level;  // 0 off, 1 low, 2 mid, 3 high
  uint16_t bursts;
};

uint32_t seq = 0;

void sendMsg(uint8_t type, uint8_t level) {
  IrMsg msg = {MAGIC, seq, type, level, BURSTS_PER_LEVEL};
  esp_now_send(BROADCAST, (const uint8_t *)&msg, sizeof(msg));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);

  for (int i = 0; i < 3; i++) {
    ledcAttach(IR_PINS[i], CARRIER_HZ, 8);
    ledcWrite(IR_PINS[i], 0);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("# ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("# ir_tx running: watch Node B's Serial Monitor for hit rates");
}

void loop() {
  for (uint8_t level = 0; level < 4; level++) {
    sendMsg(IR_START, level);
    delay(20);  // let Node B reset its counters

    for (uint16_t i = 0; i < BURSTS_PER_LEVEL; i++) {
      if (level > 0) ledcWrite(IR_PINS[level - 1], 128);  // 50% duty = carrier on
      delayMicroseconds(BURST_US);
      if (level > 0) ledcWrite(IR_PINS[level - 1], 0);
      delayMicroseconds(GAP_US);
    }

    delay(20);
    sendMsg(IR_END, level);
    Serial.printf("# seq %lu sent %s\n", (unsigned long)seq, LEVEL_NAMES[level]);
    delay(50);
  }
  seq++;
}
