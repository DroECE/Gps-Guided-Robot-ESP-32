#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "BluetoothSerial.h"
#include "MotorControl.h"

// Pin Definitions for GPS
#define GPS_RX_PIN 16   // ESP32 RX Pin
#define GPS_TX_PIN 17   // ESP32 TX Pin

// Pin Definitions for HMC5883L Compass (I2C)
#define SDA_PIN 21      // ESP32 SDA Pin
#define SCL_PIN 22      // ESP32 SCL Pin

// Constants
const float EARTH_RADIUS = 6371000; // Earth's radius in meters
const float REACHED_THRESHOLD = 1.5; // Stop if within 1.5 meters of the target

// GPS Variables
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);
float currentLat = 0.0;
float currentLong = 0.0;

// Compass Variables (using Adafruit HMC5883L library)
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345); // Unique ID for the sensor
float currentHeading = 0.0;  // Heading of the mower

// Target Coordinates
float targetLat = 0.0;     // Target latitude
float targetLong = 0.0;    // Target longitude

// Bluetooth Setup
BluetoothSerial SerialBT;
String receivedData = "";

void setup() {
  Serial.begin(115200);          // For debugging
  
  // Initialize Bluetooth
  SerialBT.begin("ESP32-GPS-Mower");  // Bluetooth name
  Serial.println("Bluetooth is ready!");
  
  // Initialize GPS
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // RX = 16, TX = 17 (Adjust pins as per your wiring)
  Serial.println("GPS is initializing...");
  
  // Initialize Compass (Adafruit HMC5883L)
  if (!compass.begin()) {
    Serial.println("Couldn't find the sensor");
    while (1);
  }
  Serial.println("Compass initialized.");

  // Initialize Motors
  setupMotors();
}

void loop() {
  // Handle Bluetooth data
  handleBluetooth();

  // Read GPS data
  updateGPS();
  
  // Read Compass data
  updateCompass();
  
  // Handle navigation
  navigateToTarget();
}

// GPS Data Reading and Parsing
void updateGPS() {
  while (GPS_Serial.available() > 0) {
    if (gps.encode(GPS_Serial.read())) {
      if (gps.location.isUpdated()) {
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

  // Calculate heading (magnetic field)
  currentHeading = atan2(event.magnetic.y, event.magnetic.x) * 180.0 / PI;

  if (currentHeading < 0) {
    currentHeading += 360.0;  // Normalize to 0-360°
  }

  Serial.printf("Heading: %.2f°\n", currentHeading);
}

// Bluetooth Data Handling
void handleBluetooth() {
  if (SerialBT.available()) {
    char incomingChar = SerialBT.read();
    if (incomingChar == '\n') {
      parseBluetoothData();
      receivedData = "";
    } else {
      receivedData += incomingChar;
    }
  }
}

// Parse Bluetooth data into coordinates
void parseBluetoothData() {
  int commaIndex = receivedData.indexOf(',');
  if (commaIndex != -1) {
    targetLat = receivedData.substring(0, commaIndex).toFloat();
    targetLong = receivedData.substring(commaIndex + 1).toFloat();
    Serial.printf("Received Coordinates: Lat = %.6f, Long = %.6f\n", targetLat, targetLong);
  } else {
    Serial.println("Invalid Bluetooth data format!");
  }
}

// Navigation Logic
void navigateToTarget() {
  if (targetLat == 0.0 || targetLong == 0.0) {
    Serial.println("No target coordinates set.");
    return;
  }

  // Calculate distance and bearing to the target
  float distanceToTarget = calculateDistance(currentLat, currentLong, targetLat, targetLong);
  float targetBearing = calculateBearing(currentLat, currentLong, targetLat, targetLong);

  Serial.printf("Distance to Target: %.2f meters\n", distanceToTarget);
  Serial.printf("Target Bearing: %.2f°, Current Heading: %.2f°\n", targetBearing, currentHeading);

  if (distanceToTarget <= REACHED_THRESHOLD) {
    Serial.println("Target reached!");
    stopMotors();
    return;
  }

  // Calculate heading difference
  float headingDifference = targetBearing - currentHeading;
  if (headingDifference < -180) headingDifference += 360;
  if (headingDifference > 180) headingDifference -= 360;

  // Adjust direction or move forward
  if (abs(headingDifference) > 10) {
    if (headingDifference > 0) {
      Serial.println("Adjusting right...");
      turnRight();
    } else {
      Serial.println("Adjusting left...");
      turnLeft();
    }
  } else {
    Serial.println("Moving forward...");
    moveForward();
  }
}

// Haversine formula for distance calculation
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);

  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return EARTH_RADIUS * c; // Distance in meters
}

// Bearing calculation
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) -
            sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  float bearing = atan2(y, x) * 180.0 / PI;

  return (bearing >= 0) ? bearing : (360.0 + bearing); // Normalize to 0-360°
}
