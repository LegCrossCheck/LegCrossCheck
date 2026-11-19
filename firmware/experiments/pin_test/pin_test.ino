// Feather ESP32-S3 header solder test
//
// Mode 1 (default): ALL header GPIOs driven HIGH.
//   Multimeter black probe on GND, red probe on each pin -> every pin should read ~3.3 V.
//   0 V or a reading that flickers when you wiggle the board = bad joint.
//
// Mode 2 (walk): ONE pin HIGH at a time, all others LOW, moving every 3 s.
//   Serial Monitor tells you which pin is HIGH. That pin should read ~3.3 V and its
//   neighbours ~0 V. A neighbour that also reads 3.3 V = solder bridge between them.
//
// Switch modes: press the BOOT button, or type 'a' (all) / 'w' (walk) in Serial Monitor.
// Serial Monitor: 115200 baud.
//
// Unplug anything wired to the header pins first (breadboard circuits).
// The STEMMA QT IMU can stay plugged in.
// Not tested here: 3V, GND, BAT, EN, USB, RST (power / control pins, not GPIOs).
// Measure 3V (~3.3 V) and BAT (battery voltage, or ~4.2 V on USB) directly.

struct HeaderPin {
  const char *name;
  uint8_t gpio;
};

HeaderPin pins[] = {
  {"D5", 5},  {"D6", 6},  {"D9", 9},  {"D10", 10}, {"D11", 11}, {"D12", 12}, {"D13", 13},
  {"A0", A0}, {"A1", A1}, {"A2", A2}, {"A3", A3},  {"A4", A4},  {"A5", A5},
  {"SCK", SCK}, {"MO", MOSI}, {"MI", MISO}, {"RX", RX}, {"TX", TX},
  {"SDA", SDA}, {"SCL", SCL},
};
const int N = sizeof(pins) / sizeof(pins[0]);

const uint8_t BOOT_BTN = 0;
const uint32_t WALK_MS = 3000;

bool walkMode = false;
int walkIdx = 0;
uint32_t nextStep = 0;

void allHigh() {
  for (int i = 0; i < N; i++) digitalWrite(pins[i].gpio, HIGH);
  Serial.println();
  Serial.println("MODE: ALL HIGH - every pin below should read ~3.3 V:");
  for (int i = 0; i < N; i++) {
    Serial.printf("  %-4s (GPIO%d)\n", pins[i].name, pins[i].gpio);
  }
  Serial.println("Press BOOT or type 'w' for walk mode.");
}

void walkShow() {
  for (int i = 0; i < N; i++) digitalWrite(pins[i].gpio, i == walkIdx ? HIGH : LOW);
  Serial.printf("WALK: %-4s (GPIO%d) is HIGH, all others should read 0 V\n",
                pins[walkIdx].name, pins[walkIdx].gpio);
}

void setMode(bool walk) {
  walkMode = walk;
  if (walkMode) {
    Serial.println();
    Serial.println("MODE: WALK - one pin HIGH at a time. Press BOOT or type 'a' for all-high.");
    walkIdx = 0;
    walkShow();
    nextStep = millis() + WALK_MS;
  } else {
    allHigh();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  for (int i = 0; i < N; i++) pinMode(pins[i].gpio, OUTPUT);
  pinMode(BOOT_BTN, INPUT_PULLUP);
  setMode(false);
}

void loop() {
  // BOOT button toggles the mode
  static bool lastBtn = HIGH;
  bool btn = digitalRead(BOOT_BTN);
  if (lastBtn == HIGH && btn == LOW) {
    setMode(!walkMode);
    delay(50);  // debounce
  }
  lastBtn = btn;

  // Serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'a') setMode(false);
    if (c == 'w') setMode(true);
  }

  // advance the walk
  if (walkMode && millis() >= nextStep) {
    walkIdx = (walkIdx + 1) % N;
    walkShow();
    nextStep = millis() + WALK_MS;
  }
}
