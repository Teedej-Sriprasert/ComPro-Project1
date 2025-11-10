#include <Wire.h>
#include <Servo.h>

// Pin setup ultrasonic
#define TRIG_LEFT 2
#define ECHO_LEFT 3
#define TRIG_FRONT 4
#define ECHO_FRONT 5
#define TRIG_RIGHT 6
#define ECHO_RIGHT 7
#define TRIG_TOP 8
#define ECHO_TOP 9

// L298N motor pins
#define IN1 10   // Left motor forward
#define IN2 11   // Left motor backward
#define ENA 5    // Left motor PWM (must be PWM pin)
#define IN3 12   // Right motor forward
#define IN4 13   // Right motor backward
#define ENB 6    // Right motor PWM

// servo
#define CLAW_PIN A0

// MPU6050
#define MPU_ADDR 0x68

struct BotState
{
    bool hasBlock;
    int missionStep;
    float leftDist, frontDist, rightDist, topDist;
};

BotState robot;

const int trigPins[4] = {TRIG_LEFT, TRIG_FRONT, TRIG_RIGHT, TRIG_TOP};
const int echoPins[4] = {ECHO_LEFT, ECHO_FRONT, ECHO_RIGHT, ECHO_TOP};

unsigned long lastGyroMs = 0;

// Tuning variable
// Distances in cm
// const float DESIRED_SIDE = 3.0;      // keep ~15 cm from wall
const float SIDE_TOL = 4.0;           // tolerance for side correction
const float TOP_CLEAR = 5.0;         // if top distance < this, something ahead (trigger turn)
// const float FRONT_OBJECT = 9.0;       // front sensor threshold to detect block (cm)

// Motor speeds (0 - 255)
const int SPEED_FORWARD = 160;        // forward nominal
const int SPEED_CORRECT = 120;        // slower for correction
const int SPEED_TURN = 170;           // spin speed for turns (MPU controlled)

// Gyro integration tuning
const float GYRO_SENS = 131.0;        // LSB/(deg/s) for +/-250dps (MPU6050 default)
const float GYRO_BIAS = 0.0;          // initial bias estimate (tune or calibrate)
const unsigned long GYRO_READ_DT_MS = 10; // integration step in ms

Servo claw;

// Servo angles (tune)
const int SERVO_OPEN = 10;
const int SERVO_CLOSED = 80;

// declar funciton
long readUltrasonicPing(int trigPin, int echoPin);

void goplaceBall()
{
    // step 1
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    delay(1000);
    spinByAngle(-90); // turn left
    
    // step 2
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 20.0 + 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    delay(1000);
    spinByAngle(90); // turn right
    
    //step 3
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 40.0 + 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    spinByAngle(90); // turn right
    delay(1000);
    stopMotors();
    turnToObj();
    
    // step 3 
    while (readUltrasonicPing(trigPins[1] , echoPins[1]) > 2.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    pickUpBlock();
    
    // step 4
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) < 15.0)
    {
      driveMotors(-SPEED_FORWARD , -SPEED_FORWARD);
    }
    stopMotors();
    spinByAngle(90); // turn right
    delay(1000);
    stopMotors();

    // step 5
        while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    spinByAngle(90); // turn right
    delay(1000);
    stopMotors();

    // step 6
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    spinByAngle(90); // turn right
    delay(1000);
    stopMotors();
    
    // step 7
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    spinByAngle(-90); // turn left
    delay(1000);
    stopMotors();

    // step 8
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 20.0 + 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    spinByAngle(90); // turn right
    delay(1000);
    stopMotors();

    // step 8
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    spinByAngle(-90); // turn left
    delay(1000);
    stopMotors();
    
    // step 9
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    spinByAngle(-90); // turn left
    delay(1000);
    stopMotors();
    
    // step 9
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    spinByAngle(-90); // turn left
    delay(1000);
    stopMotors();

    // step 9
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    spinByAngle(90); // turn right
    delay(1000);
    stopMotors();
    
    // step 10
    while (readUltrasonicPing(trigPins[3] , echoPins[3]) > 5.0)
    {
      driveMotors(SPEED_FORWARD , SPEED_FORWARD);
    }
    stopMotors();
    spinByAngle(90); // turn right
    delay(1000);
    stopMotors();
    placeBlock();

  }

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

void  driveMotors(int leftSpeed, int rightSpeed) {
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

void moveForwardDist(float dist){
    float currDist = (float)readUltrasonicPing(trigPins[3] , echoPins[3]);
    while (currDist < dist)
    {
        float currDist = (float)readUltrasonicPing(trigPins[3] , echoPins[3]);
        driveMotors(SPEED_FORWARD , SPEED_FORWARD);
        delay(500);
        stopMotors();
        delay(500);
    }
    return 0;
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

void pickUpBlock() {

  // close claw to hold
  claw.write(SERVO_CLOSED);
  delay(500);
}
void placeBlock() {

  // close claw to hold
  claw.write(SERVO_OPEN);
  delay(500);
}

void turnToObj(){

  float dis[7]; // -30 -20 -10 0 10 20 30
  int targetAngle = 0;
  // collecting data
  spinByAngle(-30);
  for (int i = 0; i < 7; i++)
  {
    dis[i] = readUltrasonicPing(trigPins[1] , echoPins[1]);
    spinByAngle(10);
  }

  float min = dis[0];
  for (int j = 0 ; j < 7 ; j++){
    if (dis[j] < min){
      min = dis[j];
      targetAngle = j;
    }
  }
  if (targetAngle > 3){ // dis[3] = 0
    spinByAngle(10*(targetAngle - 3));
    return 0;
  }
  else if (targetAngle < 3){
    spinByAngle(-10*(targetAngle - 3));
    return 0;
  }
  else {
    return 0;
  }
}

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0);    // set to zero (wakes up)
    Wire.endTransmission(true);

    // Ultrasonic pins
    for (int i = 0; i < 4; ++i) {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
  }
  // motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  robot.missionStep = 1;
  robot.hasBlock = false;

  claw.attach(CLAW_PIN);
  claw.write(SERVO_OPEN);

  lastGyroMs = millis();

}

void loop(){
  goplaceBall();
}