
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu(Wire);

// Motor pins
const int motor1pin1 = 2;
const int motor1pin2 = 3;
const int motor2pin1 = 4;
const int motor2pin2 = 5;

// Optional PWM speed control (connect ENA, ENB of motor driver to these pins)
const int motorEnable1 = 6;
const int motorEnable2 = 9;

// Optional status LED
const int ledPin = 13;

// Movement thresholds
const int threshold = 3000;      // Movement threshold (raw value)
const int deadzone = 200;        // Hysteresis zone

// Timeout fail-safe
unsigned long lastMovementTime = 0;
const unsigned long timeout = 5000; // 5 seconds

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();

  Serial.println("Calibrating MPU6050, do not move the sensor...");
  mpu.calcGyroOffsets(true);
  Serial.println("Calibration complete!");

  // Setup pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(motorEnable1, OUTPUT);
  pinMode(motorEnable2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  stopMotors();
}

void loop() {
  mpu.update();

  int accX = mpu.getRawAccX();
  int accY = mpu.getRawAccY();

  int absAccX = abs(accX);
  int absAccY = abs(accY);

  int speed = map(max(absAccX, absAccY), 0, 17000, 0, 255);
  speed = constrain(speed, 0, 255);
  analogWrite(motorEnable1, speed);
  analogWrite(motorEnable2, speed);

  if (accX > threshold + deadzone) {
    Serial.println("Forward");
    moveForward();
    lastMovementTime = millis();
  } else if (accX < -threshold - deadzone) {
    Serial.println("Backward");
    moveBackward();
    lastMovementTime = millis();
  } else if (accY > threshold + deadzone) {
    Serial.println("Right");
    turnRight();
    lastMovementTime = millis();
  } else if (accY < -threshold - deadzone) {
    Serial.println("Left");
    turnLeft();
    lastMovementTime = millis();
  } else if (millis() - lastMovementTime > timeout) {
    Serial.println("Fail-safe: No motion detected");
    stopMotors();
  } else {
    Serial.println("Stop");
    stopMotors();
  }

  delay(50);
}

void moveForward() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  digitalWrite(ledPin, LOW);
}

void moveBackward() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  digitalWrite(ledPin, LOW);
}

void turnLeft() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  digitalWrite(ledPin, LOW);
}

void turnRight() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  digitalWrite(ledPin, LOW);
}

void stopMotors() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  digitalWrite(ledPin, HIGH); // LED ON when stopped
}
