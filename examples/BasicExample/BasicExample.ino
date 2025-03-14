#include <Arduino.h>
#include "ELVH_Sensor.h"

ELVH_Sensor sensor("150G-HAND-C-PSA4", 5); // Initialize with model and CS pin

void setup() {
    Serial.begin(115200);
    sensor.begin(); // Initialize the sensor
    sensor.setDesiredUnit(sensor.bar); // Set desired unit
}

void loop() {
    sensor.readSensorData(4); // Read 4 bytes by default
    Serial.print("Pressure: ");
    Serial.println(sensor.getPressure());
    Serial.print("Temperature: ");
    Serial.println(sensor.getTemperature());
    delay(1000); // Read data every second
}