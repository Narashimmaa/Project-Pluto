// Self-balancing robot - ESP32 (NodeMCU-32S) + MPU6050 + 2x BTS7960
// Requires ESP32 Arduino core 3.x (ledcAttach API)

#include <Wire.h>

// ---------- Pins ----------
#define A_REN   16
#define A_LEN   17
#define A_RPWM  25
#define A_LPWM  26

#define B_REN   18
#define B_LEN   19
#define B_RPWM  27
#define B_LPWM  32

#define SDA_PIN 21
#define SCL_PIN 22

#define PWM_FREQ 20000
#define PWM_RES  8          // 0-255

// Per-motor direction. Flip a sign if that wheel spins the wrong way.
#define DIR_A  1
#define DIR_B -1
#define GYRO_SIGN 1         // set to -1 if the angle lags/overshoots on fast rotation

uint8_t MPU_ADDR = 0x68;

// ---------- Filter ----------
float angle = 0;
float gyroOffset = 0;
unsigned long lastMicros;

// ---------- PID ----------
float Kp = 25.0;
float Ki = 0.0;
float Kd = 1.0;

float targetAngle = 0;      // angle shown when held at the true balance pose
float integral = 0;

// ---------- Safety ----------
const float FALL_ANGLE     = 35.0;
const float ARM_WINDOW     = 3.0;
const int   DEADZONE       = 25;    // measure yours
const float ERROR_DEADBAND = 0.5;
bool armed = false;
byte failCount = 0;

unsigned long lastLoop = 0;
unsigned long lastPrint = 0;
const unsigned long LOOP_INTERVAL_US = 5000;   // 200 Hz

// ---------- I2C ----------
bool detectMpu() {
  for (uint8_t a = 0x68; a <= 0x69; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      MPU_ADDR = a;
      Serial.print("MPU at 0x"); Serial.println(a, HEX);
      return true;
    }
  }
  return false;
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t mpuRead(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

bool mpuReadAll(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) < 14) return false;

  uint8_t b[14];
  for (uint8_t i = 0; i < 14; i++) b[i] = Wire.read();

  ax = (int16_t)((b[0]  << 8) | b[1]);
  ay = (int16_t)((b[2]  << 8) | b[3]);
  az = (int16_t)((b[4]  << 8) | b[5]);
  gx = (int16_t)((b[8]  << 8) | b[9]);
  gy = (int16_t)((b[10] << 8) | b[11]);
  gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

void mpuInit() {
  mpuWrite(0x6B, 0x00);   // wake
  delay(100);
  mpuWrite(0x19, 0x04);   // 200 Hz sample rate
  mpuWrite(0x1A, 0x03);   // DLPF ~44 Hz
  mpuWrite(0x1B, 0x08);   // gyro +/-500 deg/s (65.5 LSB per deg/s)
  mpuWrite(0x1C, 0x08);   // accel +/-4g
  delay(100);
}

// ---------- Motors ----------
void driveMotor(int rpwm, int lpwm, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) {
    ledcWrite(rpwm, 0);
    ledcWrite(lpwm, 0);
    return;
  }
  if (speed > 0 && speed < DEADZONE) speed = DEADZONE;
  if (speed < 0 && speed > -DEADZONE) speed = -DEADZONE;

  if (speed > 0) {
    ledcWrite(rpwm, speed);
    ledcWrite(lpwm, 0);
  } else {
    ledcWrite(rpwm, 0);
    ledcWrite(lpwm, -speed);
  }
}

void stopMotors() {
  ledcWrite(A_RPWM, 0); ledcWrite(A_LPWM, 0);
  ledcWrite(B_RPWM, 0); ledcWrite(B_LPWM, 0);
}

void enableDrivers(bool on) {
  digitalWrite(A_REN, on); digitalWrite(A_LEN, on);
  digitalWrite(B_REN, on); digitalWrite(B_LEN, on);
}

void haltForever(const char *msg) {
  stopMotors();
  enableDrivers(false);
  Serial.println(msg);
  while (1) delay(100);
}

void calibrateGyro() {
  Serial.println("Calibrating gyro, keep still...");
  long sum = 0;
  int good = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < 500; i++) {
    if (mpuReadAll(ax, ay, az, gx, gy, gz)) { sum += gy; good++; }
    delay(2);
  }
  if (good < 400) haltForever("Calibration failed (bad reads). Motors OFF.");
  gyroOffset = (float)sum / good;
  Serial.print("Gyro offset: "); Serial.println(gyroOffset);
}

void setup() {
  // drivers off before anything else
  pinMode(A_REN, OUTPUT); pinMode(A_LEN, OUTPUT);
  pinMode(B_REN, OUTPUT); pinMode(B_LEN, OUTPUT);
  enableDrivers(false);

  ledcAttach(A_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(A_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(B_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(B_LPWM, PWM_FREQ, PWM_RES);
  stopMotors();

  Serial.begin(115200);
  delay(1000);
  Serial.println("BOOT OK");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(20);          // ms, so a stuck bus can't hang the loop
  delay(100);

  if (!detectMpu()) haltForever("MPU not found, motors OFF");

  Serial.print("WHO_AM_I: 0x");
  Serial.println(mpuRead(0x75), HEX);   // clones often report 0x70

  mpuInit();
  calibrateGyro();

  int16_t ax, ay, az, gx, gy, gz;
  if (!mpuReadAll(ax, ay, az, gx, gy, gz)) haltForever("First read failed, motors OFF");
  angle = atan2((float)ax, (float)az) * 180.0 / PI;

  Serial.print("Start angle: "); Serial.println(angle);
  Serial.println("Ready. Motors arm when angle is near targetAngle.");

  lastMicros = micros();
  lastLoop = micros();
}

void loop() {
  unsigned long now = micros();
  if (now - lastLoop < LOOP_INTERVAL_US) return;
  lastLoop = now;

  int16_t ax, ay, az, gx, gy, gz;
  if (!mpuReadAll(ax, ay, az, gx, gy, gz)) {
    if (++failCount >= 5) {            // 5 bad reads in a row
      stopMotors();
      enableDrivers(false);
      armed = false;
      integral = 0;
      if (millis() - lastPrint > 200) {
        lastPrint = millis();
        Serial.println("SENSOR FAIL, motors cut");
      }
    }
    lastMicros = micros();
    return;
  }
  failCount = 0;

  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;

  float accAngle = atan2((float)ax, (float)az) * 180.0 / PI;
  float gyroRate = GYRO_SIGN * (gy - gyroOffset) / 65.5;   // deg/s

  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accAngle;

  float error = targetAngle - angle;

  // Fallen: cut and disarm
  if (fabs(error) > FALL_ANGLE) {
    stopMotors();
    enableDrivers(false);
    armed = false;
    integral = 0;
    if (millis() - lastPrint > 200) {
      lastPrint = millis();
      Serial.print(angle); Serial.println("\tFALLEN");
    }
    return;
  }

  // Wait until the robot is held near the balance point
  if (!armed) {
    if (fabs(error) < ARM_WINDOW) {
      armed = true;
      integral = 0;
      Serial.println("ARMED");
    } else {
      stopMotors();
      enableDrivers(false);
      if (millis() - lastPrint > 200) {
        lastPrint = millis();
        Serial.print("waiting, angle="); Serial.println(angle);
      }
      return;
    }
  }

  enableDrivers(true);

  if (fabs(error) < ERROR_DEADBAND) {
    driveMotor(A_RPWM, A_LPWM, 0);
    driveMotor(B_RPWM, B_LPWM, 0);
    return;
  }

  integral += error * dt;
  integral = constrain(integral, -50, 50);
  float derivative = -gyroRate;        // d(error)/dt from the gyro

  float output = Kp * error + Ki * integral + Kd * derivative;
  output = constrain(output, -255, 255);

  driveMotor(A_RPWM, A_LPWM, (int)(output * DIR_A));
  driveMotor(B_RPWM, B_LPWM, (int)(output * DIR_B));

  if (millis() - lastPrint > 100) {    // 10 Hz print so it doesn't slow the loop
    lastPrint = millis();
    Serial.print(angle); Serial.print('\t'); Serial.println(output);
  }
}
