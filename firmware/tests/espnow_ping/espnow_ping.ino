// ESP-NOW link check. Flash to both Feathers.
// Each board broadcasts a counter every 500 ms and prints what it hears from
// the other board, with signal strength (RSSI) and how many packets were lost.

#include <WiFi.h>
#include <esp_now.h>

const uint32_t MAGIC = 0xC0FFEE01;  // ignore packets from anything else
const uint32_t SEND_EVERY_MS = 500;
const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct PingMsg {
  uint32_t magic;
  uint32_t count;
};

uint32_t sendCount = 0;

// Filled by the receive callback, printed in loop()
volatile bool gotPing = false;
volatile uint32_t rxCount = 0;
volatile int rxRssi = 0;
uint8_t rxMac[6];

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(PingMsg)) return;
  PingMsg msg;
  memcpy(&msg, data, sizeof(msg));
  if (msg.magic != MAGIC) return;
  memcpy(rxMac, info->src_addr, 6);
  rxCount = msg.count;
  rxRssi = info->rx_ctrl->rssi;
  gotPing = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.print("# this board: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("# ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0;  // current channel
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void loop() {
  static uint32_t lastSend = 0;
  static uint32_t lastRx = 0;
  static uint32_t lost = 0;

  if (millis() - lastSend >= SEND_EVERY_MS) {
    lastSend = millis();
    PingMsg msg = {MAGIC, sendCount++};
    esp_now_send(BROADCAST, (const uint8_t *)&msg, sizeof(msg));
  }

  if (gotPing) {
    gotPing = false;
    uint32_t count = rxCount;
    if (lastRx != 0 && count > lastRx + 1) lost += count - lastRx - 1;
    lastRx = count;
    Serial.printf("from %02X:%02X:%02X:%02X:%02X:%02X  count=%lu  rssi=%d dBm  lost=%lu\n",
                  rxMac[0], rxMac[1], rxMac[2], rxMac[3], rxMac[4], rxMac[5],
                  (unsigned long)count, (int)rxRssi, (unsigned long)lost);
  }
}
