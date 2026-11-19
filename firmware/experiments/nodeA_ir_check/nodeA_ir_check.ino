// Node A IR LED check   *** flash onto the Feather WITHOUT a screen ***
// Turns the 38 kHz carrier on D9 ON for 2 s, OFF for 1 s, forever.
// While ON:
//   - phone front camera pointed at the IR LED shows a steady purple glow
//   - the red "Sig" LED on the TSSP77038 board (Node B) should light / flicker
//   - multimeter DC volts, D9 to GND: ~1.6 V (50 % duty of 3.3 V)
//   - multimeter, PN2222 collector (LED - side) to GND: well below 3.3 V
// The red LED on the Feather (D13) is on while the carrier is on.
// Serial (115200) prints the carrier frequency the chip actually configured.

const uint8_t  IR_TX_PIN  = 9;
const uint8_t  LED_PIN    = 13;
const uint32_t CARRIER_HZ = 38000;

void carrier(bool on) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(IR_TX_PIN, on ? 128 : 0);
#else
  ledcWrite(0, on ? 128 : 0);
#endif
  digitalWrite(LED_PIN, on);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(LED_PIN, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(IR_TX_PIN, CARRIER_HZ, 8);
  Serial.printf("carrier configured: %lu Hz on D%d\n", (unsigned long)ledcReadFreq(IR_TX_PIN), IR_TX_PIN);
#else
  ledcSetup(0, CARRIER_HZ, 8);
  ledcAttachPin(IR_TX_PIN, 0);
  Serial.printf("carrier configured: %.0f Hz on D%d\n", ledcReadFreq(0), IR_TX_PIN);
#endif
}

void loop() {
  Serial.println("IR ON  (2 s)");
  carrier(true);
  delay(2000);
  Serial.println("IR OFF (1 s)");
  carrier(false);
  delay(1000);
}
