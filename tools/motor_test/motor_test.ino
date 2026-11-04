// Motor test for 2x BTS7960 on ESP32 (no sensor, no PID).
// Wheels OFF the ground. Each motor runs forward, stops, reverses, stops.
// ESP32 Arduino core 3.x
#define A_REN   16
#define A_LEN   17
#define A_RPWM  25
#define A_LPWM  26
#define B_REN   18
#define B_LEN   19
#define B_RPWM  27
#define B_LPWM  32

#define PWM_FREQ 20000
#define PWM_RES  8
#define TEST_SPEED 120   // 0-255

void run(int rpwm, int lpwm, int speed) {
  if (speed >= 0) { ledcWrite(rpwm, speed);  ledcWrite(lpwm, 0); }
  else            { ledcWrite(rpwm, 0);      ledcWrite(lpwm, -speed); }
}

void allStop() {
  ledcWrite(A_RPWM, 0); ledcWrite(A_LPWM, 0);
  ledcWrite(B_RPWM, 0); ledcWrite(B_LPWM, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(A_REN, OUTPUT); pinMode(A_LEN, OUTPUT);
  pinMode(B_REN, OUTPUT); pinMode(B_LEN, OUTPUT);
  digitalWrite(A_REN, LOW); digitalWrite(A_LEN, LOW);
  digitalWrite(B_REN, LOW); digitalWrite(B_LEN, LOW);

  ledcAttach(A_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(A_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(B_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(B_LPWM, PWM_FREQ, PWM_RES);
  allStop();

  delay(1000);
  digitalWrite(A_REN, HIGH); digitalWrite(A_LEN, HIGH);
  digitalWrite(B_REN, HIGH); digitalWrite(B_LEN, HIGH);
}

void loop() {
  Serial.println("A forward");  run(A_RPWM, A_LPWM,  TEST_SPEED); delay(2000); allStop(); delay(1000);
  Serial.println("A reverse");  run(A_RPWM, A_LPWM, -TEST_SPEED); delay(2000); allStop(); delay(1000);
  Serial.println("B forward");  run(B_RPWM, B_LPWM,  TEST_SPEED); delay(2000); allStop(); delay(1000);
  Serial.println("B reverse");  run(B_RPWM, B_LPWM, -TEST_SPEED); delay(2000); allStop(); delay(1000);
}
