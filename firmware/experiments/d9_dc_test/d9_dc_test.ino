// D9 DC test - no PWM, no 38 kHz. Tells you whether a problem is code or hardware.
// D9 is held HIGH for 3 s, then LOW for 3 s, forever. The small red LED (D13) follows D9.
//
// While the red LED is ON, measure with a multimeter (black probe on GND):
//   D9                         -> ~3.3 V
//   transistor base (middle)   -> ~0.7 V
//   transistor collector       -> < 0.3 V   (LED - side)
//   across the 100 ohm         -> ~1.5 V    (about 15 mA through the IR LED)
// The IR LED is steadily on: a phone front camera should show a clear purple glow.
//
// If D9 itself is far below 3.3 V, something on the board is pulling it down
// (solder bridge, missing 1k base resistor, or another part wired to D9).

const uint8_t IR_TX_PIN = 9;
const uint8_t RED_LED   = 13;

void setup() {
  Serial.begin(115200);
  pinMode(IR_TX_PIN, OUTPUT);
  pinMode(RED_LED, OUTPUT);
}

void loop() {
  digitalWrite(IR_TX_PIN, HIGH);
  digitalWrite(RED_LED, HIGH);
  Serial.println("D9 HIGH (3 s) - measure now");
  delay(3000);

  digitalWrite(IR_TX_PIN, LOW);
  digitalWrite(RED_LED, LOW);
  Serial.println("D9 LOW  (3 s)");
  delay(3000);
}
