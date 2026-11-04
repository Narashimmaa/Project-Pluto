// Raw MPU6050 read (ESP32). Prints ax, ay, az, gy and the tilt angle.
// Flat and still: one accel axis near 8192 (1 g at +/-4 g). Tilt and watch the angle.
#include <Wire.h>

uint8_t MPU_ADDR = 0x68;

bool detectMpu() {
  for (uint8_t a = 0x68; a <= 0x69; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { MPU_ADDR = a; return true; }
  }
  return false;
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

bool mpuReadAll(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) < 14) return false;
  uint8_t b[14];
  for (uint8_t i = 0; i < 14; i++) b[i] = Wire.read();
  ax = (int16_t)((b[0] << 8) | b[1]);   ay = (int16_t)((b[2] << 8) | b[3]);
  az = (int16_t)((b[4] << 8) | b[5]);   gx = (int16_t)((b[8] << 8) | b[9]);
  gy = (int16_t)((b[10] << 8) | b[11]); gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21, 22);
  Wire.setClock(100000);
  if (!detectMpu()) { Serial.println("MPU not found"); while (1) delay(100); }
  Serial.print("MPU at 0x"); Serial.println(MPU_ADDR, HEX);
  mpuWrite(0x6B, 0x00); delay(100);
  mpuWrite(0x1B, 0x08);
  mpuWrite(0x1C, 0x08);
  delay(100);
  Serial.println("ax\tay\taz\tgy\tangle");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  if (!mpuReadAll(ax, ay, az, gx, gy, gz)) { Serial.println("read fail"); delay(100); return; }
  float angle = atan2((float)ax, (float)az) * 180.0 / PI;
  Serial.print(ax); Serial.print('\t'); Serial.print(ay); Serial.print('\t');
  Serial.print(az); Serial.print('\t'); Serial.print(gy); Serial.print('\t');
  Serial.println(angle);
  delay(100);
}
