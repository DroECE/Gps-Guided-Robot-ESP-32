
//Flash this and rotate your robot, after that enter any key on your serial monitor to show, max (x,y) and min (x,y)
// then put values to main.ino

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Compass instance
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);

// Calibration values
float magX_min = 10000.0, magY_min = 10000.0, magZ_min = 10000.0;
float magX_max = -10000.0, magY_max = -10000.0, magZ_max = -10000.0;

void setup() {
    Serial.begin(115200);
    
    // Initialize Compass
    if (!compass.begin()) {
        Serial.println("Error: Compass not found!");
        while (1);
    }
    
    Serial.println("Compass calibration. Move the sensor in all directions.");
    Serial.println("Press any key to stop calibration.");
}

void loop() {
    // Check if user pressed a key to stop calibration
    if (Serial.available()) {
        Serial.println("Calibration stopped.");
        printCalibrationValues();
        while (1);  // Stop the loop
    }

    // Get compass event
    sensors_event_t event;
    compass.getEvent(&event);

    // Update min and max values
    if (event.magnetic.x < magX_min) magX_min = event.magnetic.x;
    if (event.magnetic.x > magX_max) magX_max = event.magnetic.x;
    
    if (event.magnetic.y < magY_min) magY_min = event.magnetic.y;
    if (event.magnetic.y > magY_max) magY_max = event.magnetic.y;
    
    if (event.magnetic.z < magZ_min) magZ_min = event.magnetic.z;
    if (event.magnetic.z > magZ_max) magZ_max = event.magnetic.z;

    // Print current magnetometer values
    Serial.print("X: "); Serial.print(event.magnetic.x);
    Serial.print(" Y: "); Serial.print(event.magnetic.y);
    Serial.print(" Z: "); Serial.println(event.magnetic.z);

    delay(100);
}

void printCalibrationValues() {
    Serial.println("Calibration values:");
    Serial.print("magX_min = "); Serial.println(magX_min);
    Serial.print("magX_max = "); Serial.println(magX_max);
    Serial.print("magY_min = "); Serial.println(magY_min);
    Serial.print("magY_max = "); Serial.println(magY_max);
    Serial.print("magZ_min = "); Serial.println(magZ_min);
    Serial.print("magZ_max = "); Serial.println(magZ_max);
}
