/*
  auto_hardcoded.ino
  Hard-coded mission for RAI Auto Bot.
  - 4x Ultrasonic sensors: LEFT, FRONT, RIGHT, TOP
  - 2x DC motors via L298N
  - MPU6050 gyro used for turning (integrate gyro Z)
  - 1x Servo gripper to pickup & hold block
  - Uses struct, arrays, pointers, functions, loop
  NOTE: tune the TUNE_* values for your robot and arena.
*/

#include <Wire.h>
#include <Servo.h>

// -------------------- PINS --------------------
#define TRIG_LEFT 2
#define ECHO_LEFT 3
#define TRIG_FRONT 4
#define ECHO_FRONT 5
#define TRIG_RIGHT 6
#define ECHO_RIGHT 7
#define TRIG_TOP 8
#define ECHO_TOP 9

// L298N motor pins (change if your wiring differs)
#define IN1 10   // Left motor forward
#define IN2 11   // Left motor backward
#define ENA 5    // Left motor PWM (must be PWM pin)
#define IN3 12   // Right motor forward
#define IN4 13   // Right motor backward
#define ENB 6    // Right motor PWM (must be PWM pin)

// Servo pin
#define CLAW_PIN A0

// MPU6050 I2C address
#define MPU_ADDR 0x68

// -------------------- TUNING CONSTANTS --------------------
// Distances in cm
const float DESIRED_SIDE = 15.0;      // keep ~15 cm from wall
const float SIDE_TOL = 4.0;           // tolerance for side correction
const float TOP_CLEAR = 25.0;         // if top distance < this, something ahead (trigger turn)
const float FRONT_OBJECT = 9.0;       // front sensor threshold to detect block (cm)

// Motor speeds (0 - 255)
const int SPEED_FORWARD = 160;        // forward nominal
const int SPEED_CORRECT = 120;        // slower for correction
const int SPEED_TURN = 170;           // spin speed for turns (MPU controlled)

// Gyro integration tuning
const float GYRO_SENS = 131.0;        // LSB/(deg/s) for +/-250dps (MPU6050 default)
const float GYRO_BIAS = 0.0;          // initial bias estimate (tune or calibrate)
const unsigned long GYRO_READ_DT_MS = 10; // integration step in ms

// Servo angles (tune)
const int SERVO_OPEN = 10;
const int SERVO_CLOSED = 80;

// -------------------- DATA STRUCTURES --------------------
struct RobotState {
  bool hasBlock;
  int missionStep;   // 0=start, 1=go segment1, 2=pickup, 3=go segment2, 4=done
  float leftDist, frontDist, rightDist, topDist;
};

RobotState robot;

// Array usage: map sensor labels -> pins (for pointer example)
const int trigPins[4] = {TRIG_LEFT, TRIG_FRONT, TRIG_RIGHT, TRIG_TOP};
const int echoPins[4] = {ECHO_LEFT, ECHO_FRONT, ECHO_RIGHT, ECHO_TOP};

// Servo
Servo claw;

// Timer for gyro integration
unsigned long lastGyroMs = 0;

// -------------------- HELPER PROTOTYPES --------------------
long readUltrasonicPing(int trigPin, int echoPin);
void readAllUltrasonics(int *destArray, int n); // pointer usage
void maintainSideDistance();   // forward with simple side-correction using LEFT/RIGHT/TOP
void moveForwardTimed(int durationMs);
void spinByAngle(float targetDeg); // positive = right turn, negative = left turn
void stopMotors();
void driveMotors(int leftSpeed, int rightSpeed); // left/right signed speeds
float readGyroZ(); // raw gyro Z in deg/s (integrated over dt externally)
void pickUpBlock();
void printDiagnostics();

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();
  // MPU init: wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // set to zero (wakes up)
  Wire.endTransmission(true);

  // Ultrasonic pins
  for (int i = 0; i < 4; ++i) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  // Servo
  claw.attach(CLAW_PIN);
  claw.write(SERVO_OPEN);

  // init robot state
  robot.hasBlock = false;
  robot.missionStep = 1; // start mission step 1: go segment 1

  lastGyroMs = millis();

  Serial.println("Auto Hardcoded: Setup complete. Starting mission...");
}

// -------------------- MAIN LOOP --------------------
void loop() {
  // read ultrasonic sensors into an int array (array + pointer example)
  int rawDist[4];
  readAllUltrasonics(rawDist, 4);
  robot.leftDist = rawDist[0];
  robot.frontDist = rawDist[1];
  robot.rightDist = rawDist[2];
  robot.topDist = rawDist[3];

  // Basic mission state machine (hard-coded sequence)
  switch (robot.missionStep) {
    case 1:
      // Segment 1: move forward while keeping distance from wall using RIGHT as primary,
      // if TOP indicates obstacle ahead (top < TOP_CLEAR), do a right spin (turn 90) and advance to step 2
      maintainSideDistance();
      int step = 1
      // if (robot.topDist > 0 && robot.topDist < TOP_CLEAR && step == 1) {
      if (robot.topDist > 0 && robot.topDist < 5.0) {
        // obstacle ahead (a wall / corner): turn left 90 degrees
        stopMotors();
        delay(80);
        // Serial.println("Segment1: top detected obstacle -> turning LEFT 90");
        spinByAngle(-90.0); // left turn
        delay(120);
        step += 1;
      }
      if (robot.leftDist > 0 && robot.leftDist < DESIRED_SIDE  && robot.topDist > 20 && robot.topDist < 20+5 && step == 2){
        stopMotors();
        delay(80);
        spinByAngle(90.0);
        delay(120);
        step += 1;
      }
      if (robot.topDist > 0 && robot.topDist > 40+5 && robot.topDist < 40 && step == 3){
        stopMotors();
        delay(80);
        spinByAngle(90.0);
        delay(120);
        robot.missionStep = 2; // proceed to next segment
      }
      break;

    case 2:
      // Segment 2: move forward with side-correction using LEFT/RIGHT.
      // Look for object using FRONT sensor. When front < FRONT_OBJECT -> pickup.
      maintainSideDistance();

      // Find the obj

      // if (robot.frontDist > 0 && robot.frontDist < FRONT_OBJECT) {
      //   stopMotors();
      //   delay(100);
      //   Serial.println("Object detected in front -> picking up");
      //   pickUpBlock();
      //   robot.hasBlock = true;
      //   delay(300);
      //   // Back away a bit after pickup to prepare for next travel
      //   moveForwardTimed(-500); // negative for backward short
      //   delay(150);
      //   robot.missionStep = 3;
      // }
      // break;

    case 3:
      // Segment 3: after pickup, we need to go to target area.
      // Hard-code: spin left 90 (to change corridor), then go forward a timed segment,
      // then spin right 90 to re-align to target corridor, then go forward until top/clear area detected > OPEN_DIST
      Serial.println("Segment3: orient to target");
      spinByAngle(90.0); // left 90
      delay(120);
      // Short forward burst (time-based). Adjust duration for distance to target corridor.
      moveForwardTimed(1200); // move forward 1.2s (tune)
      delay(150);
      spinByAngle(90.0); // right 90
      delay(120);

      // Now move forward and detect an open area: when TOP is large (> some threshold) treat as open target zone.
      while (true) {
        // keep slight side correction
        maintainSideDistance();
        if (robot.topDist > 40 || robot.leftDist > 40 || robot.rightDist > 40) { // open area heuristics
          stopMotors();
          Serial.println("Target area probably reached (open area detected).");
          delay(200);
          break;
        }
        // update sensors
        int r[4]; readAllUltrasonics(r,4);
        robot.leftDist=r[0]; robot.frontDist=r[1]; robot.rightDist=r[2]; robot.topDist=r[3];
      }

      // drop (or hold) — you said "just hold it", but if you want to release, use drop code.
      // For demonstration: keep holding. If you want to drop at target, uncomment the next two lines:
      // claw.write(SERVO_OPEN); delay(400); // release
      robot.missionStep = 4;
      break;

    case 4:
      // Mission complete: go to exit zone or stop. We'll stop here.
      stopMotors();
      Serial.println("Mission step 4: DONE. Robot holding block.");
      robot.missionStep = 5;
      break;

    case 5:
      // Idle final state
      stopMotors();
      // Nothing else to do
      delay(1000);
      break;
  }

  // optional diagnostics print
  printDiagnostics();

  delay(80); // main loop pacing (also helps gyro integration delta)
}

// -------------------- FUNCTIONS --------------------

// Read a single ultrasonic (returns cm, or large number on timeout)
long readUltrasonicPing(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long dur = pulseIn(echoPin, HIGH, 25000); // 25 ms timeout (about 4.25 m)
  float dist = (dur * 0.0343) / 2.0;
  if (dur == 0) return 400; // no echo
  return (long)dist;
}

// Read all 4 sensors into destArray (pointer usage)
void readAllUltrasonics(int *destArray, int n) {
  for (int i = 0; i < n; ++i) {
    destArray[i] = (int)readUltrasonicPing(trigPins[i], echoPins[i]);
    delay(5); // small spacing between triggers
  }
}

// Drive motors with signed speed (-255..255)
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

  // right motor
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, constrain(rightSpeed, 0, 255));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, constrain(-rightSpeed, 0, 255));
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

// Simple maintainSideDistance: primary uses RIGHT distance to keep DESIRED_SIDE.
// If right is too large -> veer right; if too small -> veer left; fallback use left sensor.
void maintainSideDistance() {
  // update sensor readings quickly
  int arr[4]; readAllUltrasonics(arr, 4);
  robot.leftDist = arr[0];
  robot.frontDist = arr[1];
  robot.rightDist = arr[2];
  robot.topDist = arr[3];

  float error = 0.0;
  bool usingRight = true;
  if (robot.rightDist > 390) { // invalid reading -> fallback to left
    usingRight = false;
  }

  if (usingRight) {
    error = robot.rightDist - DESIRED_SIDE; // positive => too far, need to move right (steer right)
  } else {
    error = DESIRED_SIDE - robot.leftDist; // if left>DESIRED, steer left etc (invert logic)
  }

  // Simple proportional controller
  float Kp = 7.0; // steering sensitivity (tune)
  int steer = (int)(Kp * error);

  // limit steer value
  steer = constrain(steer, -60, 60);

  // compute left/right motor speeds
  int leftSpeed = SPEED_FORWARD - steer;
  int rightSpeed = SPEED_FORWARD + steer;

  // ensure speeds within range
  leftSpeed = constrain(leftSpeed, 80, 255);
  rightSpeed = constrain(rightSpeed, 80, 255);

  driveMotors(leftSpeed, rightSpeed);
}

// Move forward or backward for given duration (ms). Negative duration => backward.
void moveForwardTimed(int durationMs) {
  if (durationMs == 0) return;
  int dir = (durationMs > 0) ? 1 : -1;
  unsigned long t0 = millis();
  while (abs((long)millis() - (long)t0) < abs(durationMs)) {
    if (dir > 0) {
      driveMotors(SPEED_FORWARD, SPEED_FORWARD);
    }
    else {
      driveMotors(-SPEED_FORWARD, -SPEED_FORWARD);
    }
    delay(20);
  }
  stopMotors();
}

// Read gyro Z (deg/s) from MPU6050 (raw integration needed externally)
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

// Spin robot by targetDeg degrees using integrated gyro Z.
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

// Pickup: simply drive forward small amount if needed then close claw
void pickUpBlock() {
  // Approach slowly if needed
  driveMotors(100, 100);
  delay(250);
  stopMotors();
  delay(80);
  // close claw to hold
  claw.write(SERVO_CLOSED);
  delay(500);
}

// Diagnostics print helper
// void printDiagnostics() {
//   static unsigned long lastPrint = 0;
//   if (millis() - lastPrint < 400) return;
//   lastPrint = millis();
//   Serial.print("Step: "); Serial.print(robot.missionStep);
//   Serial.print("  L:"); Serial.print(robot.leftDist);
//   Serial.print(" F:"); Serial.print(robot.frontDist);
//   Serial.print(" R:"); Serial.print(robot.rightDist);
//   Serial.print(" T:"); Serial.print(robot.topDist);
//   Serial.print("  hasBlock:"); Serial.println(robot.hasBlock);
// }
