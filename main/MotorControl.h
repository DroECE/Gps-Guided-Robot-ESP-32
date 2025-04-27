#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

class MotorControl {
public:
    // Constructor
    MotorControl();
    
    // Initialize motor pins
    void begin(int rpwmPin1, int lpwmPin1, int rpwmPin2, int lpwmPin2);
    
    // Basic movement functions
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();
    void sharpLeft();
    void sharpRight();
    void stopMotors();
    
    // Speed control
    void setSpeed(int speed);
    
private:
    // Motor control pins
    int _rpwmPin1;
    int _lpwmPin1;
    int _rpwmPin2;
    int _lpwmPin2;
    
    // Speed settings
    int _currentSpeed;
    static const int PWM_CHANNEL_R1 = 0;
    static const int PWM_CHANNEL_L1 = 1;
    static const int PWM_CHANNEL_R2 = 2;
    static const int PWM_CHANNEL_L2 = 3;
    static const int PWM_RESOLUTION = 8;
    static const int PWM_FREQUENCY = 1000;
    
    // Helper functions
    void setupPWMChannel(int channel, int pin);
};

// Global function declarations
void setupMotors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void sharpLeft();
void sharpRight();
void stopMotors();

#endif // MOTOR_CONTROL_H
