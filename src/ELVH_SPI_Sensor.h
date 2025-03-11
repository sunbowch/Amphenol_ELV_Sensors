#ifndef ELVH_SPI_SENSOR_H
#define ELVH_SPI_SENSOR_H

#include <Arduino.h>
#include <SPI.h>

class ELVH_SPI_Sensor {
public:
    void begin(uint8_t csPin);
    void readSensorData(uint8_t bytesToRead = 4);
    int getStatus();
    float getPressure();
    float getTemperature();
    void setSensorModel(const char* model);

private:
    uint16_t minPressure;
    uint16_t maxPressure;
    float pFactor;
    uint16_t pOffset;
    uint16_t pressure;
    uint16_t temperature;
    int status;
    char sensorModel[20];
    uint8_t csPin; // New member variable to store the CS pin
    void readSPI(uint8_t bytesToRead);
    float convertPressure(uint16_t rawPressure);
    float convertTemperature(uint16_t rawTemperature);
};

#endif // ELVH_SPI_SENSOR_H