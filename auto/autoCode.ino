#include <Wire.h>
#include <Servo.h>

// ================== Pin Setup ==================
#define TRIG_FRONT 8
#define ECHO_FRONT A2

#define IN1 10
#define IN2 11
#define ENA 5
#define IN3 12
#define IN4 13
#define ENB 6

#define SERVO_LIFT_PIN 3
#define SERVO_GEAR_PIN 9

// ================== Parameters ==================
const int SPEED_LEFT = 120;
const int SPEED_RIGHT = 121;
const int SPEED_TURN = 100;

Servo servoLift;
Servo servoGear;
bool missionComplete = false;

// ================== Function Prototypes ==================
float readFrontDistance();
void driveForward();
void driveReverse();
void turnLeft(float angle);
void turnRight(float angle);
long readUltrasonicPing(int trigPin, int echoPin);
void goplaceBall();
void driveMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void spinByAngle(float targetDeg);
void liftUp();
void liftDown();
void gearRotate_open();
void gearRotate_close();
void gearReset();
void liftUp();
void liftDown();
void pickUpBlock();   // ✅ ประกาศชื่อไว้เฉยๆ
void placeBlock();    // ✅ ประกาศชื่อไว้เฉยๆ

// ================== Main Mission ==================
void goplaceBall() {
  
  
  // step 1
  while (readFrontDistance() > 5)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnRight(90);
  stopMotors();
  delay(500); // <--- FIXED: อ่านง่ายขึ้น (เดิมคือ -90)

  unsigned long start11 = millis();
  while (millis() - start11 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);

  // step 2
  while (readFrontDistance() > 26)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90); // <--- FIXED: อ่านง่ายขึ้น (เดิมคือ 90)
  stopMotors();
  delay(500);

  unsigned long start22 = millis();
  while (millis() - start22 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);

  // step 3: เดินไปข้างหน้าแทน ultrasonic
  unsigned long start3 = millis();
  while (millis() - start3 < 920) {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

    unsigned long start333 = millis();
  while (millis() - start333 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);

  // step 3 (ต่อ)
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  pickUpBlock();
  stopMotors();
  delay(500);
  
  // step 4: ถอยหลังแทน ultrasonic
  unsigned long start4 = millis();
  while (millis() - start4 < 1200) {
    driveReverse();
  }
  stopMotors();
  delay(500);

  while (readFrontDistance() > 24.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);

  turnLeft(90);
  stopMotors();
  delay(500);

  // step 5
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

  // step 6
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

  unsigned long start66 = millis();
  while (millis() - start66 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);
  
  // step 7
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnRight(90);
  stopMotors();
  delay(500);

  unsigned long start77 = millis();
  while (millis() - start77 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);

  // step 8
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }

  stopMotors();
  delay(500);

  turnRight(90);
  stopMotors();
  delay(500);

  unsigned long start88 = millis();
  while (millis() - start88 < 600) {
    driveReverse();
  }

  stopMotors();
  delay(500);
  // step 8 (ต่อ)
  while (readFrontDistance() > 25.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

  unsigned long start888 = millis();
  while (millis() - start888 < 600) {
    driveReverse();
  }
  
  // step 9
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnRight(90);
  stopMotors();
  delay(500);

  // step 9 (ต่อ)
  unsigned long start99 = millis();
  while (millis() - start99 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);

  // step 10 (ต่อ)
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnRight(90);
  stopMotors();
  delay(500);
  
  // step 10
  unsigned long start1010 = millis();
  while (millis() - start1010 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);

  // step 11
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

  unsigned long start1111 = millis();
  while (millis() - start1111 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);

//12
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

  unsigned long start1212 = millis();
  while (millis() - start1212 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);

  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
//13
  unsigned long start13 = millis();
  while (millis() - start13 < 1200) {
    driveReverse();
  }
  stopMotors();
  delay(500);

  while (readFrontDistance() > 24.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

//14
  unsigned long start14 = millis();
  while (millis() - start14 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);

  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnRight(90);
  stopMotors();
  delay(500);

  unsigned long start15 = millis();
  while (millis() - start15 < 600) {
    driveReverse();
  }

  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

  unsigned long start16 = millis();
  while (millis() - start16 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

  unsigned long start17 = millis();
  while (millis() - start17 < 600) {
    driveReverse();
  }

  stopMotors();
  delay(500);
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnRight(90);
  stopMotors();
  delay(500);

  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

  unsigned long start18 = millis();
  while (millis() - start18 < 600) {
    driveReverse();
  }

  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

  unsigned long start19 = millis();
  while (millis() - start19 < 600) {
    driveReverse();
  }

  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnRight(90);
  stopMotors();
  delay(500);

  unsigned long start20 = millis();
  while (millis() - start20 < 600) {
    driveReverse();
  }

  while (readFrontDistance() > 25.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnRight(90);
  stopMotors();
  delay(500);

  unsigned long start21 = millis();
  while (millis() - start21 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);

  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();
  delay(500);
  turnLeft(90);
  stopMotors();
  delay(500);

  unsigned long start211 = millis();
  while (millis() - start211 < 600) {
    driveReverse();
  }
  stopMotors();
  delay(500);
  
  while (readFrontDistance() > 5.0)
  {
    driveForward();
  }
  stopMotors();

}

// ================== Servo Control ==================
void liftUp() {
  servoLift.write(90);   // หมุนจาก 0 → 90
  Serial.println("Lift Up");
}

void liftDown() {
  servoLift.write(0);    // กลับลง 90 → 0
  Serial.println("Lift Down");
}

void gearRotate_open() {
  servoGear.write(0);    // แล้วกลับเป็น 0
  Serial.println("Gear Rotate");
}

void gearRotate_close() {
  servoGear.write(180);  // จาก 90 → 180
  delay(700);

}

void gearReset() {
  servoGear.write(90);   // ตั้งกลับที่ 90
  Serial.println("Gear Reset");
}

// ================== Ultrasonic ==================
float readFrontDistance() {
  return readUltrasonicPing(TRIG_FRONT, ECHO_FRONT);
}

long readUltrasonicPing(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long dur = pulseIn(echoPin, HIGH, 25000);
  float dist = (dur * 0.0343) / 2.0;
  if (dur == 0) return 400;
  return (long)dist;
}

// ================== Motor ==================
void driveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = leftSpeed * -1; // ถ้าต้องกลับทิศ

  // left
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(leftSpeed, 0, 255));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(-leftSpeed, 0, 255));
  }

  // right
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

void driveForward() {
  driveMotors(SPEED_LEFT, SPEED_RIGHT);
}

void driveReverse() {
  driveMotors(-SPEED_LEFT, -SPEED_RIGHT);
}

// ================== Spin by Timing ==================
void spinByAngle(float targetDeg) {
  const int TURN_SPEED = 110;
  const float TURN_TIME_90 = 380.0;
  const float TURN_TIME_PER_DEG = TURN_TIME_90 / 90.0;

  int dir = (targetDeg > 0) ? 1 : -1;
  float absDeg = abs(targetDeg);
  unsigned long turnTime = absDeg * TURN_TIME_PER_DEG;

  driveMotors(dir * TURN_SPEED, -dir * TURN_SPEED);
  delay(turnTime);
  stopMotors();

  Serial.print("Spin by timing done (deg): ");
  Serial.println(targetDeg);
}

void turnLeft(float angle) {
  spinByAngle(abs(angle));
}
void turnRight(float angle) {
  spinByAngle(-abs(angle));
}

// ================== Gripper Control ==================
void pickUpBlock() {
  liftDown();
  delay(500);
  delay(500);
  liftUp();
  delay(500);
}

void placeBlock() {
  liftDown();
  delay(500);
  gearReset();
  delay(500);
  liftUp();
  delay(500);
}

// ================== Setup ==================
void setup() {
  Serial.begin(9600);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  servoLift.attach(SERVO_LIFT_PIN);
  servoGear.attach(SERVO_GEAR_PIN);

  servoLift.write(0);
  servoGear.write(90);
  delay(500);

  Serial.println("Setup Complete");
}


// ================== Loop ==================
void loop() {
  if (!missionComplete) {
    goplaceBall();
    missionComplete = true;
    Serial.println("Mission Complete");
  }
}
