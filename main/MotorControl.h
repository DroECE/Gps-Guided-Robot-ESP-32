#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

// Define motor control pins
#define MOTOR1_IN1 32  // Left motor
#define MOTOR1_IN2 33
#define MOTOR2_IN1 18  // Right motor
#define MOTOR2_IN2 19

// Function declarations
void setupMotors();
void moveForward();
void stopMotors();
void turnLeft();
void turnRight();
void sharpLeft();
void sharpRight();
void moveReverse();

#endif // MOTORCONTROL_H