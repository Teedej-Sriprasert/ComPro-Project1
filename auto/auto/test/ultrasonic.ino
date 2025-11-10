#define TRIG_LEFT 2
#define ECHO_LEFT 3
#define TRIG_FRONT 4
#define ECHO_FRONT 5
#define TRIG_RIGHT 6
#define ECHO_RIGHT 7
#define TRIG_TOP 8
#define ECHO_TOP 9

const int trigPins[4] = {TRIG_LEFT, TRIG_FRONT, TRIG_RIGHT, TRIG_TOP};
const int echoPins[4] = {ECHO_LEFT, ECHO_FRONT, ECHO_RIGHT, ECHO_TOP};

struct RobotState {
  bool hasBlock;
  int missionStep;   // 0=start, 1=go segment1, 2=pickup, 3=go segment2, 4=done
  float leftDist, frontDist, rightDist, topDist;
};

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

void setup() {
  // Initialize the serial communication at 9600 bits per second
  Serial.begin(9600);

    // Ultrasonic pins
  for (int i = 0; i < 4; ++i) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

void loop() {

  int rawDist[4];
  readAllUltrasonics(rawDist, 4);
  robot.leftDist = rawDist[0];
  robot.frontDist = rawDist[1];
  robot.rightDist = rawDist[2];
  robot.topDist = rawDist[3];

  Serial.println("left Dist = ");
  Serial.println(robot.leftDist);

  Serial.println("frontDist = ");
  Serial.println(robot.frontDist);
  
  Serial.println("right Dist = ");
  Serial.println(robot.rightDist);

  Serial.println("topDist = ");
  Serial.println(robot.topDist);

  delay(2000);
}