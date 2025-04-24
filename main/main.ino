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

// Pin Definitions for Relay
#define RELAY_PIN 25    // Relay control pin

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
float Kp = 1.0, Ki = 0, Kd = 0.83;              // PID constants (tune these for your system)
float previousError = 0;                       // Previous error for derivative term
float integral = 0;                            // Integral term for PID
float error_output = 0;                        // PID controller output

// Magnetic Declination for Compass
float magneticDeclination = -3.19; // Use this to correct for any bias errors from true north

// Compass Calibration Values
float magX_min = -30.26;
float magY_min = -43.27;
float magZ_min = -1.00;
float magX_max = 42.73;
float magY_max = 30.27;
float magZ_max = 1.00;
bool useCalibratedMag = true;  // Use calibration values if set to true

// Multiple Target Locations
struct Location {
    float latitude;
    float longitude;
};

Location targetLocations[10];  // Increased to hold up to 10 target locations
int currentTargetIndex = 0;    // Index of the current target location

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

    // Initialize Relay
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // Ensure relay is off initially

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
        SerialBT.println("STOP ALL FUNCTIONS");
    } else if (receivedData == "g") {
        stopFlag = false;  // Clear stop flag
        targetReachedFlag = false;  // Reset target reached flag for new command
        allTargetsReached = false;  // Reset all targets reached flag for new command
        Serial.println("GO command received. Resuming operation.");
        SerialBT.println("RESUME ALL FUNCTIONS");
    } else if (receivedData == "pid") {
        sendPIDValuesViaBluetooth();
    } else if (receivedData == "c") {
        digitalWrite(RELAY_PIN, HIGH); // Turn relay on
        Serial.println("Relay ON command received.");
        SerialBT.println("Cutter ON");
    } else if (receivedData == "") {
        digitalWrite(RELAY_PIN, LOW); // Turn relay off
        Serial.println("Relay OFF command received.");
        SerialBT.println("Cutter OFF");
    } else {
        parseTargetLocations(receivedData);
    }
}

// Parse multiple target coordinates in [[lat1, long1], [lat2, long2]] format
void parseTargetLocations(String data) {
    int startIndex = data.indexOf("[[");
    int endIndex = data.lastIndexOf("]]");

    if (startIndex == -1 || endIndex == -1) {
        Serial.println("Invalid data format! Use [[lat1, long1], [lat2, long2], ...]");
        SerialBT.println("Error: Invalid format! Use [[lat1, long1], [lat2, long2], ...]");
        return;
    }

    data = data.substring(startIndex + 2, endIndex);  // Extract the content inside [[ ]]

    int index = 0;
    while (data.length() > 0 && index < 10) {  // Limit the number of coordinates to 10
        int commaIndex = data.indexOf(",");
        int closeIndex = data.indexOf("]");

        if (commaIndex == -1 || closeIndex == -1) break;

        float lat = data.substring(0, commaIndex).toFloat();
        float lng = data.substring(commaIndex + 1, closeIndex).toFloat();

        targetLocations[index].latitude = lat;
        targetLocations[index].longitude = lng;
        index++;

        data = data.substring(closeIndex + 2);  // Move to the next coordinate pair
    }

    if (index > 0) {
        currentTargetIndex = 0;
        stopFlag = false;  // Clear stop flag if new targets are set
        targetReachedFlag = false;  // Reset target reached flag for new targets
        allTargetsReached = false;  // Reset all targets reached flag for new targets
        Serial.printf("New targets set: %d locations\n", index);
        SerialBT.printf("New targets set: %d locations\n", index);
    } else {
        Serial.println("No valid coordinates found!");
        SerialBT.println("Error: No valid coordinates found!");
    }
}
