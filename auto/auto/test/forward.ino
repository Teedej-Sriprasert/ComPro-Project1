#include <Wire.h>
#include <Servo.h>

// L298N motor pins
#define IN1 10   // Left motor forward
#define IN2 11   // Left motor backward
#define ENA 5    // Left motor PWM (must be PWM pin)
#define IN3 12   // Right motor forward
#define IN4 13   // Right motor backward
#define ENB 6    // Right motor PWM

// Motor speeds (0 - 255)
const int SPEED_FORWARD = 160;        // forward nominal
const int SPEED_CORRECT = 120;        // slower for correction
const int SPEED_TURN = 170;   

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

void void setup()
{
      // motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

}

void void loop()
{
    driveMotors(SPEED_FORWARD,SPEED_FORWARD);
    delay(1500);
    stopMotors();
    delay(1500);
}
