#include "MotorControl.h"
#include "Arduino.h"

// Pin definitions for BTS7960 motor drivers
#define RPWM_PIN1 25  // Right PWM pin for motor 1
#define LPWM_PIN1 26  // Left PWM pin for motor 1
#define RPWM_PIN2 32  // Right PWM pin for motor 2
#define LPWM_PIN2 33  // Left PWM pin for motor 2

// Speed settings
#define DEFAULT_SPEED 130      // Default speed (0-255)
#define TURN_SPEED 100        // Speed durifng turns
#define SHARP_TURN_SPEED 150  // Speed during sharp turns

// Global instance
MotorControl motorControl;

MotorControl::MotorControl() {
    _currentSpeed = DEFAULT_SPEED;
}

void MotorControl::begin(int rpwmPin1, int lpwmPin1, int rpwmPin2, int lpwmPin2) {
    _rpwmPin1 = rpwmPin1;
    _lpwmPin1 = lpwmPin1;
    _rpwmPin2 = rpwmPin2;
    _lpwmPin2 = lpwmPin2;
    
    // Setup PWM channels
    setupPWMChannel(PWM_CHANNEL_R1, _rpwmPin1);
    setupPWMChannel(PWM_CHANNEL_L1, _lpwmPin1);
    setupPWMChannel(PWM_CHANNEL_R2, _rpwmPin2);
    setupPWMChannel(PWM_CHANNEL_L2, _lpwmPin2);
}

void MotorControl::setupPWMChannel(int channel, int pin) {
    ledcSetup(channel, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(pin, channel);
}

void MotorControl::setSpeed(int speed) {
    _currentSpeed = constrain(speed, 0, 255);
}

void MotorControl::moveForward() {
    // Motor 1
    ledcWrite(PWM_CHANNEL_R1, _currentSpeed / 2);
    ledcWrite(PWM_CHANNEL_L1, 0);
    
    // Motor 2
    ledcWrite(PWM_CHANNEL_R2, _currentSpeed / 2);
    ledcWrite(PWM_CHANNEL_L2, 0);
}

void MotorControl::moveBackward() {
    // Motor 1
    ledcWrite(PWM_CHANNEL_R1, 0);
    ledcWrite(PWM_CHANNEL_L1, _currentSpeed / 2);
    
    // Motor 2
    ledcWrite(PWM_CHANNEL_R2, 0);
    ledcWrite(PWM_CHANNEL_L2, _currentSpeed / 2);
}

void MotorControl::turnLeft() {
    // Motor 1 (slower)
    ledcWrite(PWM_CHANNEL_R1, TURN_SPEED / 4);
    ledcWrite(PWM_CHANNEL_L1, 0);
    
    // Motor 2 (faster)
    ledcWrite(PWM_CHANNEL_R2, TURN_SPEED);
    ledcWrite(PWM_CHANNEL_L2, 0);
}

void MotorControl::turnRight() {
    // Motor 1 (faster)
    ledcWrite(PWM_CHANNEL_R1, TURN_SPEED);
    ledcWrite(PWM_CHANNEL_L1, 0);
    
    // Motor 2 (slower)
    ledcWrite(PWM_CHANNEL_R2, TURN_SPEED / 4);
    ledcWrite(PWM_CHANNEL_L2, 0);
}

void MotorControl::sharpLeft() {
    // Motor 1 (backward)
    ledcWrite(PWM_CHANNEL_R1, 0);
    ledcWrite(PWM_CHANNEL_L1, SHARP_TURN_SPEED);
    
    // Motor 2 (forward)
    ledcWrite(PWM_CHANNEL_R2, SHARP_TURN_SPEED);
    ledcWrite(PWM_CHANNEL_L2, 0);
}

void MotorControl::sharpRight() {
    // Motor 1 (forward)
    ledcWrite(PWM_CHANNEL_R1, SHARP_TURN_SPEED);
    ledcWrite(PWM_CHANNEL_L1, 0);
    
    // Motor 2 (backward)
    ledcWrite(PWM_CHANNEL_R2, 0);
    ledcWrite(PWM_CHANNEL_L2, SHARP_TURN_SPEED);
}

void MotorControl::stopMotors() {
    // Stop all motors
    ledcWrite(PWM_CHANNEL_R1, 0);
    ledcWrite(PWM_CHANNEL_L1, 0);
    ledcWrite(PWM_CHANNEL_R2, 0);
    ledcWrite(PWM_CHANNEL_L2, 0);
}

// Global function implementations
void setupMotors() {
    motorControl.begin(RPWM_PIN1, LPWM_PIN1, RPWM_PIN2, LPWM_PIN2);
}

void moveForward() {
    motorControl.moveForward();
}

void moveBackward() {
    motorControl.moveBackward();
}

void turnLeft() {
    motorControl.turnLeft();
}

void turnRight() {
    motorControl.turnRight();
}

void sharpLeft() {
    motorControl.sharpLeft();
}

void sharpRight() {
    motorControl.sharpRight();
}

void stopMotors() {
    motorControl.stopMotors();
}
