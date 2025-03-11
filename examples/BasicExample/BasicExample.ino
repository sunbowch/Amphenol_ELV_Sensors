#include <Arduino.h>
#include <ELVH_SPI_Sensor.h>

ELVH_SPI_Sensor sensor;

void setup() {
    Serial.begin(115200);
    sensor.begin();
}

void loop() {
    sensor.readSensorData(4); // Read 4 bytes by default
    delay(1000); // Read data every second
}