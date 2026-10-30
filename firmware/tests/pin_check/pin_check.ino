// Header pin check for a Feather ESP32-S3. Flash to one Feather at a time,
// with nothing else wired to it (unplug the STEMMA QT cable too).
//
// 1. Self-check on start, no wiring needed:
//      stuck LOW  - reads LOW with its pull-up on      (short to GND)
//      stuck HIGH - reads HIGH with its pull-down on   (short to 3V)
//      bridged    - pulling one pin low drags another  (solder bridge)
//    Then the red LED on D13 blinks 3 times.
// 2. Touch test: touch a jumper wire from GND to each pin's breadboard row.
//    Each pin prints once when it feels the touch. A pin that never prints
//    has a bad solder joint or breadboard contact.
//
// Send "r" in the Serial Monitor to start over.

#include <driver/gpio.h>

struct Pin {
  const char *name;
  uint8_t gpio;
};

// Every signal pin on the Feather header except D13 (checked by blinking,
// because its LED pulls the pin low) and RST/EN (touching it resets the board)
const Pin PINS[] = {
  {"A0", A0}, {"A1", A1}, {"A2", A2}, {"A3", A3}, {"A4", A4}, {"A5", A5},
  {"SCK", SCK}, {"MO", MOSI}, {"MI", MISO}, {"RX", RX}, {"TX", TX},
  {"SDA", SDA}, {"SCL", SCL},
  {"D5", 5}, {"D6", 6}, {"D9", 9}, {"D10", 10}, {"D11", 11}, {"D12", 12},
};
const int N = sizeof(PINS) / sizeof(PINS[0]);
const uint8_t LED_PIN = 13;

bool stuck[N];
bool seen[N];
int seenCount = 0;
int okCount = 0;  // pins that passed the self-check and can be touch-tested

void allPullUp() {
  for (int i = 0; i < N; i++) pinMode(PINS[i].gpio, INPUT_PULLUP);
  delay(5);
}

void selfCheck() {
  int problems = 0;
  Serial.println("\n# Self-check");

  allPullUp();
  for (int i = 0; i < N; i++) {
    stuck[i] = false;
    seen[i] = false;
    if (digitalRead(PINS[i].gpio) == LOW) {
      Serial.printf("  %-4s stuck LOW  (short to GND?)\n", PINS[i].name);
      stuck[i] = true;
      problems++;
    }
  }

  for (int i = 0; i < N; i++) pinMode(PINS[i].gpio, INPUT_PULLDOWN);
  delay(5);
  for (int i = 0; i < N; i++) {
    if (!stuck[i] && digitalRead(PINS[i].gpio) == HIGH) {
      Serial.printf("  %-4s stuck HIGH (short to 3V?)\n", PINS[i].name);
      stuck[i] = true;
      problems++;
    }
  }

  // Pull each pin low in turn, weakly, and see if any other pin follows
  allPullUp();
  for (int i = 0; i < N; i++) {
    if (stuck[i]) continue;
    pinMode(PINS[i].gpio, OUTPUT);
    gpio_set_drive_capability((gpio_num_t)PINS[i].gpio, GPIO_DRIVE_CAP_0);
    digitalWrite(PINS[i].gpio, LOW);
    delayMicroseconds(200);
    for (int j = i + 1; j < N; j++) {
      if (!stuck[j] && digitalRead(PINS[j].gpio) == LOW) {
        Serial.printf("  %s and %s bridged (solder bridge?)\n", PINS[i].name, PINS[j].name);
        problems++;
      }
    }
    pinMode(PINS[i].gpio, INPUT_PULLUP);
  }
  delay(5);

  okCount = 0;
  for (int i = 0; i < N; i++) if (!stuck[i]) okCount++;
  if (problems == 0) Serial.printf("  no shorts found on %d pins\n", N);

  Serial.println("\n# D13: the red LED should blink 3 times now");
  pinMode(LED_PIN, OUTPUT);
  for (int k = 0; k < 3; k++) {
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(300);
  }

  seenCount = 0;
  Serial.println("\n# Touch test: touch GND to each pin's row, one at a time");
  printRemaining();
}

void printRemaining() {
  Serial.print("  remaining:");
  for (int i = 0; i < N; i++) {
    if (!stuck[i] && !seen[i]) Serial.printf(" %s", PINS[i].name);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) delay(10);

  // Power the I2C pull-ups so SDA/SCL don't read as stuck LOW
#if defined(PIN_I2C_POWER)
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, HIGH);
#elif defined(TFT_I2C_POWER)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif

  selfCheck();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') selfCheck();
  }

  for (int i = 0; i < N; i++) {
    if (stuck[i] || seen[i]) continue;
    if (digitalRead(PINS[i].gpio) == LOW) {
      seen[i] = true;
      seenCount++;
      Serial.printf("  %-4s OK  (%d/%d)\n", PINS[i].name, seenCount, okCount);
      if (seenCount == okCount) {
        Serial.println("\n# All pins confirmed. Send r to run again.");
      } else {
        printRemaining();
      }
    }
  }
  delay(10);
}
