#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "BluetoothSerial.h"
#include "MotorControl.h"
#include <math.h>

// Pin Definitions for GPS
#define GPS_RX_PIN 16   // ESP32 RX Pin
#define GPS_TX_PIN 17   // ESP32 TX Pin

// Pin Definitions for Compass (HMC5883L I2C)
#define SDA_PIN 21      // ESP32 SDA Pin
#define SCL_PIN 22      // ESP32 SCL Pin

// GPS and Compass instances
TinyGPSPlus gps;                               // GPS library instance
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);  // Compass instance
HardwareSerial GPS_Serial(1);                  // Use HardwareSerial 1 for GPS

// Bluetooth Setup
BluetoothSerial SerialBT;
String receivedData = "";                      // Buffer for incoming Bluetooth data

// Navigation Variables
float currentLat = 0.0, currentLong = 0.0;     // Current GPS coordinates
int currentHeading = 0, headingError = 0;      // Compass heading and error
int targetHeading;                             // Target heading
int distanceToTarget = 0;                      // Distance to the target in meters

// State Management
bool stopFlag = false;                         // Stops all actions when true
bool targetReachedFlag = false;                // Indicates if the target has been reached
bool allTargetsReached = false;                // Indicates if all target coordinates have been reached

// Thresholds and Tolerances
#define HEADING_TOLERANCE 8                    // Degrees within which heading is considered correct
#define WAYPOINT_DIST_TOLERANCE 1.5            // Distance in meters to consider waypoint reached

// PID Controller Variables
float Kp = 1.0, Ki = 0, Kd = 0.8;              // PID constants (tune these for your system)
float previousError = 0;                       // Previous error for derivative term
float integral = 0;                            // Integral term for PID
float error_output = 0;                        // PID controller output

// Magnetic Declination for Compass
float magneticDeclination = -3.19; // Use this to correct for any bias errors from true north

// Compass Calibration Values
float magX_min = -35.00;  // Obtain this value from calibration sketch
float magY_min = -48.27;  // Obtain this value from calibration sketch
float magZ_min = -1.00;   // Obtain this value from calibration sketch
float magX_max = 47.64;   // Obtain this value from calibration sketch
float magY_max = 34.91;   // Obtain this value from calibration sketch
float magZ_max = 1.00;    // Obtain this value from calibration sketch
bool useCalibratedMag = true;  // Use calibration values if set to true

// Multiple Target Locations
struct Location {
    float latitude;
    float longitude;
};

Location targetLocations[6]; // Array to hold up to 6 target locations
int currentTargetIndex = 0;  // Index of the current target location

// Previous heading value for comparison
float previousHeading = -1;  // Initialize with an invalid heading to ensure the first print

void setup() {
    Serial.begin(115200);                                     // For debugging
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // GPS initialization

    // Initialize Bluetooth
    SerialBT.begin("ESP32-GPS-Mower");
    Serial.println("Bluetooth ready. Connect and send commands.");

    // Initialize Compass
    if (!compass.begin()) {
        Serial.println("Error: Compass not found!");
        while (1);
    }
    Serial.println("Compass initialized.");

    setupMotors();                                            // Initialize motor pins
    Serial.println("System ready. Waiting for coordinates...");
}

void loop() {
    handleBluetooth();

    if (stopFlag) {
        stopMotors();
        return;
    }

    updateGPS();
    updateCompass();
    navigateToTarget();
}

// GPS Data Reading
void updateGPS() {
    while (GPS_Serial.available() > 0) {
        if (gps.encode(GPS_Serial.read())) {
            if (gps.location.isValid()) {
                currentLat = gps.location.lat();
                currentLong = gps.location.lng();
                
                Serial.printf("Current Location: Lat = %.6f, Long = %.6f\n", currentLat, currentLong);
            }
        }
    }
}

// Compass Data Reading
void updateCompass() {
    sensors_event_t event;
    compass.getEvent(&event);
  
    float mag_x, mag_y;
    if (useCalibratedMag) {
        mag_x = mapf(event.magnetic.x, magX_min, magX_max, -100, 100);
        mag_y = mapf(event.magnetic.y, magY_min, magY_max, -100, 100);
    } else {
        mag_x = event.magnetic.x;
        mag_y = event.magnetic.y;
    }
    currentHeading = ((atan2(mag_y, mag_x) * 180) / PI) + magneticDeclination;

    if (currentHeading < 0) {
        currentHeading += 360;
    }
    if (currentHeading > 360) {
        currentHeading -= 360;
    }

    // Print only when the heading changes significantly
    if (abs(currentHeading - previousHeading) > 1.0) { // 1 degree threshold
        Serial.printf("Heading: %.2f°\n", currentHeading);
        previousHeading = currentHeading;
    }
}

// Bluetooth Data Handling
void handleBluetooth() {
    while (SerialBT.available()) {
        char incomingChar = SerialBT.read();
        if (incomingChar == '\n') {
            parseBluetoothData();
            receivedData = "";  // Clear buffer
        } else {
            receivedData += incomingChar;
        }
    }
}

// Parse Bluetooth data into coordinates or commands
void parseBluetoothData() {
    if (receivedData.startsWith("kp:")) {
        Kp = receivedData.substring(3).toFloat();
        Serial.printf("Kp updated to: %.2f\n", Kp);
        SerialBT.printf("Kp updated to: %.2f\n", Kp);
    } else if (receivedData.startsWith("ki:")) {
        Ki = receivedData.substring(3).toFloat();
        Serial.printf("Ki updated to: %.2f\n", Ki);
        SerialBT.printf("Ki updated to: %.2f\n", Ki);
    } else if (receivedData.startsWith("kd:")) {
        Kd = receivedData.substring(3).toFloat();
        Serial.printf("Kd updated to: %.2f\n", Kd);
        SerialBT.printf("Kd updated to: %.2f\n", Kd);
    } else if (receivedData == "l") {
        sendLocationViaBluetooth();
    } else if (receivedData == "s") {
        stopFlag = true;  // Set stop flag
        stopMotors();
        Serial.println("STOP command received. Motors stopped.");
        SerialBT.println("STOP command received. Motors stopped.");
    } else if (receivedData == "g") {
        stopFlag = false;  // Clear stop flag
        targetReachedFlag = false;  // Reset target reached flag for new command
        allTargetsReached = false;  // Reset all targets reached flag for new command
        Serial.println("GO command received. Resuming operation.");
        SerialBT.println("GO command received. Resuming operation.");
    } else {
        parseTargetLocations(receivedData);
    }
}

// Parse multiple target coordinates
void parseTargetLocations(String data) {
    char* dataCStr = strdup(data.c_str());
    char* token = strtok(dataCStr, ";");
    int index = 0;
    while (token != NULL && index < 6) {
        char* commaIndex = strchr(token, ',');
        if (commaIndex != NULL) {
            *commaIndex = '\0';
            targetLocations[index].latitude = atof(token);
            targetLocations[index].longitude = atof(commaIndex + 1);
            index++;
        }
        token = strtok(NULL, ";");
    }
    free(dataCStr);
    if (index > 0) {
        currentTargetIndex = 0;
        stopFlag = false;  // Clear stop flag if new targets are set
        targetReachedFlag = false;  // Reset target reached flag for new targets
        allTargetsReached = false;  // Reset all targets reached flag for new targets
        Serial.printf("New targets set: %d locations\n", index);
        SerialBT.printf("New targets set: %d locations\n", index);
    } else {
        Serial.println("Invalid Bluetooth data format!");
        SerialBT.println("Error: Use format <lat1>,<long1>;<lat2>,<long2>;... or send 'l', 's', or 'g'.");
    }
}

// Send current location via Bluetooth
void sendLocationViaBluetooth() {
    if (gps.location.isValid()) {
        int satellites = gps.satellites.value();
        float hdop = gps.hdop.hdop();

        SerialBT.printf("Current Location: Lat = %.6f, Long = %.6f\n", currentLat, currentLong);
        SerialBT.printf("Satellites: %d, HDOP: %.2f\n", satellites, hdop);

        Serial.printf("Satellites: %d, HDOP: %.2f\n", satellites, hdop);
    } else {
        SerialBT.println("GPS location unavailable.");
    }
}

// Navigation Logic
void navigateToTarget() {
    if (currentTargetIndex >= 6 || targetLocations[currentTargetIndex].latitude == 0.0 || targetLocations[currentTargetIndex].longitude == 0.0) {
        if (!allTargetsReached) {
            Serial.println("No target coordinates set or all targets reached.");
            allTargetsReached = true;  // Set flag to indicate message has been printed
        }
        return;
    }

    float targetLat = targetLocations[currentTargetIndex].latitude;
    float targetLong = targetLocations[currentTargetIndex].longitude;

    distanceToTarget = calculateDistance(targetLat, targetLong);
    targetHeading = calculateHeading(targetLat, targetLong);

    Serial.printf("Distance to Target: %d meters\n", distanceToTarget);
    Serial.printf("Target Heading: %d°, Current Heading: %d°\n", targetHeading, currentHeading);

    // If robot reaches target, stop motors and send message
    if (distanceToTarget <= WAYPOINT_DIST_TOLERANCE) {
        if (!targetReachedFlag) {
            Serial.println("Target reached!");
            stopMotors();  // Ensure motors are stopped
            targetReachedFlag = true;  // Mark as reached
            SerialBT.println("Robot has arrived at the target location.");  // Send notification

            // Add a 5-second stop at the destination before proceeding to the next target
            delay(5000);  // 5000 milliseconds = 5 seconds
            Serial.println("5-second stop complete. Proceeding to the next target.");
            currentTargetIndex++;  // Move to the next target
        }
        return;
    }

    pidController();
    move_robot();
}

// PID Controller for heading adjustment
void pidController() {
    headingError = targetHeading - currentHeading;
    if (headingError < -180) headingError += 360;
    if (headingError > 180) headingError -= 360;

    // Integral term calculation
    integral += headingError;

    // Derivative term calculation
    float derivative = headingError - previousError;

    // PID calculation
    error_output = Kp * headingError + Ki * integral + Kd * derivative;
    previousError = headingError;

    // Debugging statements
    Serial.printf("Heading Error: %d\n", headingError);
    Serial.printf("PID Output: %f\n", error_output);
}

// Calculate Distance to Target using spherical law of cosines
int calculateDistance(float targetLat, float targetLong) {
    float delta = radians(currentLong - targetLong);
    float sdlong = sin(delta);
    float cdlong = cos(delta);
    float lat1 = radians(currentLat);
    float lat2 = radians(targetLat);
    float slat1 = sin(lat1);
    float clat1 = cos(lat1);
    float slat2 = sin(lat2);
    float clat2 = cos(lat2);
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = sq(delta);
    delta += sq(clat2 * sdlong);
    delta = sqrt(delta);
    float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);
    return delta * 6372795;  // Earth's radius in meters
}

// Calculate Target Heading
int calculateHeading(float targetLat, float targetLong) {
    float dLon = radians(targetLong - currentLong);
    float y = sin(dLon) * cos(radians(targetLat));
    float x = cos(radians(currentLat)) * sin(radians(targetLat)) -
              sin(radians(currentLat)) * cos(radians(targetLat)) * cos(dLon);
    float bearing = atan2(y, x) * 180.0 / PI;
    return (bearing >= 0) ? bearing : (360.0 + bearing);
}

// Move Robot based on PID controller output
void move_robot() {
    if (abs(error_output) <= HEADING_TOLERANCE) {
        Serial.println("Moving forward...");
        moveForward();
    } else if (error_output > 0 && error_output < 60) {
        Serial.println("Adjusting right...");
        turnRight();
    } else if (error_output < 0 && error_output > -60) {
        Serial.println("Adjusting left...");
        turnLeft();
    } else if (error_output >= 60) {
        Serial.println("Sharp right turn...");
        sharpRight();
    } else if (error_output <= -60) {
        Serial.println("Sharp left turn...");
        sharpLeft();
    }
}

// Map function for floating-point values
double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}