#include "MotorControl.h"
#include <Arduino.h> // Include Arduino library for pinMode, digitalWrite, etc.

// Initialize motor pins
void setupMotors() {
  pinMode(MOTOR1_IN1, OUTPUT);  // Left motor
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);  // Right motor
  pinMode(MOTOR2_IN2, OUTPUT);
}

// Move forward (both motors in forward direction)
void moveForward() {
  digitalWrite(MOTOR1_IN1, HIGH);  // Left motor forward
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, HIGH);  // Right motor forward
  digitalWrite(MOTOR2_IN2, LOW);
}

// Stop motors
void stopMotors() {
  digitalWrite(MOTOR1_IN1, LOW);
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);
  digitalWrite(MOTOR2_IN2, LOW);
  Serial.println("stopMotors() called: All pins set to LOW.");
}

// Turn left (right motor moves forward, left motor stops)
void turnLeft() {
  digitalWrite(MOTOR1_IN1, LOW);   // Left motor stop
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, HIGH);  // Right motor forward
  digitalWrite(MOTOR2_IN2, LOW);
  delay(300);  // Adjust turning duration
  stopMotors();
}

// Turn right (left motor moves forward, right motor stops)
void turnRight() {
  digitalWrite(MOTOR1_IN1, HIGH);  // Left motor forward
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);   // Right motor stop
  digitalWrite(MOTOR2_IN2, LOW);
  delay(300);  // Adjust turning duration
  stopMotors();
}

// Sharp left turn (left motor reverse, right motor forward)
void sharpLeft() {
  digitalWrite(MOTOR1_IN1, LOW);   // Left motor reverse
  digitalWrite(MOTOR1_IN2, HIGH);
  digitalWrite(MOTOR2_IN1, HIGH);  // Right motor forward
  digitalWrite(MOTOR2_IN2, LOW);
  delay(500);  // Adjust turning duration
  stopMotors();
}

// Sharp right turn (left motor forward, right motor reverse)
void sharpRight() {
  digitalWrite(MOTOR1_IN1, HIGH);  // Left motor forward
  digitalWrite(MOTOR1_IN2, LOW);
  digitalWrite(MOTOR2_IN1, LOW);   // Right motor reverse
  digitalWrite(MOTOR2_IN2, HIGH);
  delay(500);  // Adjust turning duration
  stopMotors();
}

// Reverse (both motors in reverse direction)
void moveReverse() {
  digitalWrite(MOTOR1_IN1, LOW);   // Left motor reverse
  digitalWrite(MOTOR1_IN2, HIGH);
  digitalWrite(MOTOR2_IN1, LOW);   // Right motor reverse
  digitalWrite(MOTOR2_IN2, HIGH);
}