// Repeating I2C scanner (ESP32). Expect 0x68 (AD0 to GND) or 0x69 (AD0 to VCC).
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21, 22);   // SDA, SCL
}

void loop() {
  int found = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found 0x"); Serial.println(a, HEX);
      found++;
    }
  }
  if (!found) Serial.println("none");
  delay(500);
}
