#include <Arduino.h>
#include "ELVH_Sensor.h"

#define NUM_SENSORS 3

ELVH_Sensor sensors[NUM_SENSORS] = {
    ELVH_Sensor("150G-HAND-C-PSA4", 10),
    ELVH_Sensor("150G-HAND-C-PSA4", 11),
    ELVH_Sensor("L10D-HRRD-C-N2A4")
};

void setup() {
    Serial.begin(9600);

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].begin(); // Initialize each sensor
        sensors[i].setDesiredUnit("bar"); // Set desired unit for each sensor
    }
}

void loop() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].readSensorData();
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" Pressure: ");
        Serial.println(sensors[i].getPressure());
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" Temperature: ");
        Serial.println(sensors[i].getTemperature());
    }
    delay(1000); // Delay between reads
}
