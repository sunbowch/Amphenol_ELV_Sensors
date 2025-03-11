#include <Arduino.h>
#include "ELVH_SPI_Sensor.h"

#define NUM_SENSORS 3

ELVH_SPI_Sensor sensors[NUM_SENSORS];

void setup() {
    Serial.begin(9600);
    uint8_t csPins[NUM_SENSORS] = {10, 11, 12}; // Define CS pins for each sensor

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i].begin(csPins[i]);
        sensors[i].setSensorModel("L10D-001A"); // Example model, set accordingly
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
