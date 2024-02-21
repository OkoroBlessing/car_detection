#include <Servo.h>

int servo_pin = 10;
int trig_pin = A1;
int echo_pin = A2;

// Motor pins
int ENA = 3;
int ENB = 11;
int in1 = 4;
int in2 = 5;
int in3 = 6;
int in4 = 7;

Servo my_servo;

void setup() {
  Serial.begin(115200);

  my_servo.attach(servo_pin);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop() {

  moveRobot();
}

void moveRobot() {
  float distance = calculateDistance();
  
  if (distance >= 20) {
    moveStop();
    delay(100);
    moveBackward();
    delay(500);
    moveStop();
    delay(100);

    Serial.println(distance);


    int distanceRight = lookRight();
    delay(200);
    int distanceLeft = lookLeft();
    delay(200);

    if (distanceRight < distanceLeft) {
      turnRight();
      delay(300);
      moveRobot();
    }
    else if(distanceRight > distanceLeft) {
      turnLeft();
      delay(200);
      moveRobot();
    }
  } else {
    moveForward();
  }
}

float calculateDistance() {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  long duration = pulseIn(echo_pin, HIGH);
  // float durtion_seconds = duration * 0.000001;
  // float distance = (340 * duration_seconds / 2) * 100;a
  float distance = duration * 0.0344 / 2; 

  return distance;
}

int lookRight() {
  my_servo.write(0);
  delay(500);
  int distance = calculateDistance();
  delay(100);
  my_servo.write(90);
  return distance;
}

int lookLeft() {
  my_servo.write(180);
  delay(500);
  int distance = calculateDistance();
  delay(100);
  my_servo.write(90);
  return distance;
}

void moveStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void moveForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ENA, 80);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(ENB, 80);
}

void moveBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ENA, 80);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(ENB, 80);
}

void turnRight() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ENA, 80);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(ENB, 0);
}

void turnLeft() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(ENB, 80);
}
