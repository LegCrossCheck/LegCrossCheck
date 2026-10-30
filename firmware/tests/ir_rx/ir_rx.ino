// Week-1 test 1, Node B: IR hit counter. Flash to Node B; run ir_tx on Node A.
//
// Counts falling edges from both IR receivers during each block Node A
// announces over ESP-NOW, then prints one CSV row per block.
//
// Columns: t_ms, seq, level, bursts, tssp_edges, tsop_edges, tssp_hit, tsop_hit
//   level: 0 off, 1 low, 2 mid, 3 high
//   *_hit: fraction of bursts seen (edges capped at bursts)
// Pass (design doc): at some level, hit >= 0.90 when crossed, <= 0.05 when not.

#include <WiFi.h>
#include <esp_now.h>

const int TSSP_PIN = 9;   // D9, TSSP77038 output, low while it sees 38 kHz
const int TSOP_PIN = 10;  // D10, TSOP38238 output, same

// Keep in sync with ir_tx.ino
const uint32_t MAGIC = 0xC0FFEE02;
enum : uint8_t { IR_START = 1, IR_END = 2 };
struct IrMsg {
  uint32_t magic;
  uint32_t seq;
  uint8_t type;
  uint8_t level;
  uint16_t bursts;
};

volatile uint32_t tsspEdges = 0;
volatile uint32_t tsopEdges = 0;

void IRAM_ATTR onTssp() { tsspEdges++; }
void IRAM_ATTR onTsop() { tsopEdges++; }

// Block state, written by the ESP-NOW callback
volatile bool blockOpen = false;
volatile uint32_t blockSeq = 0;
volatile uint8_t blockLevel = 0;

// Finished block, printed in loop()
volatile bool reportReady = false;
uint32_t repSeq, repTssp, repTsop;
uint8_t repLevel;
uint16_t repBursts;

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(IrMsg)) return;
  IrMsg msg;
  memcpy(&msg, data, sizeof(msg));
  if (msg.magic != MAGIC) return;

  if (msg.type == IR_START) {
    tsspEdges = 0;
    tsopEdges = 0;
    blockSeq = msg.seq;
    blockLevel = msg.level;
    blockOpen = true;
  } else if (msg.type == IR_END) {
    // Drop the block if its START was lost
    if (!blockOpen || msg.seq != blockSeq || msg.level != blockLevel) return;
    repSeq = msg.seq;
    repLevel = msg.level;
    repBursts = msg.bursts;
    repTssp = tsspEdges;
    repTsop = tsopEdges;
    blockOpen = false;
    reportReady = true;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);

  pinMode(TSSP_PIN, INPUT_PULLUP);
  pinMode(TSOP_PIN, INPUT_PULLUP);
  attachInterrupt(TSSP_PIN, onTssp, FALLING);
  attachInterrupt(TSOP_PIN, onTsop, FALLING);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("# ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);

  Serial.println("t_ms,seq,level,bursts,tssp_edges,tsop_edges,tssp_hit,tsop_hit");
}

void loop() {
  if (!reportReady) return;
  reportReady = false;

  float tsspHit = (float)min(repTssp, (uint32_t)repBursts) / repBursts;
  float tsopHit = (float)min(repTsop, (uint32_t)repBursts) / repBursts;
  Serial.printf("%lu,%lu,%u,%u,%lu,%lu,%.2f,%.2f\n",
                (unsigned long)millis(), (unsigned long)repSeq, repLevel, repBursts,
                (unsigned long)repTssp, (unsigned long)repTsop, tsspHit, tsopHit);
}
