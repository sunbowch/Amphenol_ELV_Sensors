#ifndef ELVH_SENSOR_H
#define ELVH_SENSOR_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

class ELVH_Sensor {
public:
    ELVH_Sensor(const char* model, uint8_t csPin); // Constructor for SPI
    ELVH_Sensor(const char* model); // Constructor for I2C
    void begin(uint8_t csPin = SS); // Default to hardware CS pin if no csPin is defined
    void beginI2C();
    void readSensorData(uint8_t bytesToRead = 4);
    int getStatus();
    float getPressure();
    float getTemperature();
    void setSensorModel(const char* model);
    void setDesiredUnit(const char* unit); // New method to set the desired unit
    char sensorModel[20];
    void setCSPin(uint8_t csPin);

private:
    float minPressure;
    float maxPressure;
    float pFactor;
    uint16_t pOffset;
    uint16_t pressure;
    uint16_t temperature;
    int status;
    char unit[10]; // New member variable to store the desired unit
    
    uint8_t csPin; // New member variable to store the CS pin
    uint8_t i2cAddress; // New member variable to store the I2C address
    bool isI2C; // New member variable to indicate if the sensor is I2C
    void readSPI(uint8_t bytesToRead);
    void readI2C(uint8_t bytesToRead); // Updated method declaration
    void setPressureRangeAndTransferFunction(); // New method declaration
    float convertPressure(uint16_t rawPressure);
    float convertTemperature(uint16_t rawTemperature);
    float convertToDesiredUnit(float pressure); // New method to convert pressure to the desired unit
};

#endif // ELVH_SENSOR_H
