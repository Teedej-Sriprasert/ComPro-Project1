#include <Wire.h>
#include <Servo.h>

#define MPU_ADDR 0x68   

// Motor speeds (0 - 255)
const int SPEED_FORWARD = 160;        // forward nominal
const int SPEED_CORRECT = 120;        // slower for correction
const int SPEED_TURN = 170;      

// Gyro integration tuning
const float GYRO_SENS = 131.0;        // LSB/(deg/s) for +/-250dps (MPU6050 default)
const float GYRO_BIAS = 0.0;          // initial bias estimate (tune or calibrate)
const unsigned long GYRO_READ_DT_MS = 10; // integration step in ms

void driveMotors(int leftSpeed, int rightSpeed) {
  // left motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(leftSpeed, 0, 255));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(-leftSpeed, 0, 255));
  }
}
  float readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // GYRO_XOUT_H register (start of gyro data)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  // we read GX,GY,GZ (each 2 bytes)
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  // convert gz raw to deg/s
  float gz_dps = ((float)gz) / GYRO_SENS;
  // apply bias if measured
  gz_dps -= GYRO_BIAS;
  return gz_dps;
}

// Positive angle = right turn, Negative = left turn
void spinByAngle(float targetDeg) {
  Serial.print("spinByAngle target: "); Serial.println(targetDeg);
  float accumulated = 0.0;
  unsigned long tPrev = millis();
  // Choose direction motor commands:
  int leftCmd = (targetDeg > 0) ? SPEED_TURN : -SPEED_TURN;
  int rightCmd = (targetDeg > 0) ? -SPEED_TURN : SPEED_TURN;

  driveMotors(leftCmd, rightCmd);
  while (abs(accumulated) < abs(targetDeg)) {
    unsigned long tNow = millis();
    float dt = (tNow - tPrev) / 1000.0; // seconds
    if (dt <= 0) { delay(2); continue; }
    float gz = readGyroZ(); // deg/s (positive when rotating robot in one direction per sensor mounting)
    // NOTE: sensor orientation sign may need flipping — if turns are opposite, invert gz sign
    accumulated += gz * dt; // theta = omega * time
    tPrev = tNow;
    delay(GYRO_READ_DT_MS);
    // optional safety timeout
    static unsigned long startMs;
    if (startMs == 0) startMs = millis();
    if (millis() - startMs > 5000) { // 5s timeout fallback
      Serial.println("Spin timeout!");
      break;
    }
  }
  stopMotors();
  delay(80);
  Serial.print("Spin done. Accumulated: "); Serial.println(accumulated);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // MPU init: wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // set to zero (wakes up)
  Wire.endTransmission(true);
}

void loop(){
    spinByAngle(-90.0); // left turn
    delay(1000);
    spinByAngle(90.0); // left turn
    delay(1000);
    spinByAngle(180.0); // left turn
    delay(1000);
    spinByAngle(-180.0); // left turn
    delay(1000);
}