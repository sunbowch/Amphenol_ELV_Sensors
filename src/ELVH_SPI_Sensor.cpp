#include "ELVH_SPI_Sensor.h"
#include <Arduino.h>

void ELVH_SPI_Sensor::begin(uint8_t csPin) {
    this->csPin = csPin; // Store the CS pin
    Serial.begin(9600);
    SPI.begin();
    SPI.beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE0)); // Set SPI clock to 800 kHz
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH); // Ensure CS is high
}

void ELVH_SPI_Sensor::setSensorModel(const char* model) {
    strncpy(sensorModel, model, sizeof(sensorModel) - 1);
    sensorModel[sizeof(sensorModel) - 1] = '\0';
}

void ELVH_SPI_Sensor::readSensorData(uint8_t bytesToRead) {
    digitalWrite(csPin, LOW); // Pull CS low to start communication
    readSPI(bytesToRead);
    digitalWrite(csPin, HIGH); // Pull CS high to end communication
}

void ELVH_SPI_Sensor::readSPI(uint8_t bytesToRead) {
    uint32_t response = 0;
    for (int i = 0; i < bytesToRead; i++) {
        response <<= 8;
        response |= SPI.transfer(0x00); // Send dummy byte to receive data
    }

    uint8_t status;
    if (bytesToRead == 2) {
        status = (response >> 14) & 0x03;
        pressure = response & 0x3FFF;
        temperature = 0; // Temperature data is not available
    } else if (bytesToRead == 3) {
        status = (response >> 22) & 0x03;
        pressure = (response >> 8) & 0x3FFF;
        temperature = (response & 0xFF) << 8; // Only MSB of temperature is available
    } else if (bytesToRead == 4) {
        status = (response >> 30) & 0x03;
        pressure = (response >> 16) & 0x3FFF;
        temperature = response & 0xFFFF;
    } else {
        status = 0xFF; // Invalid status
        pressure = 0;
        temperature = 0;
    }

    Serial.print("Status: ");
    Serial.println(status, BIN);
    switch (status) {
        case 0b00:
            Serial.println("No error");
            Serial.print("Pressure: ");
            Serial.println(convertPressure(pressure));
            if (bytesToRead >= 3) {
                Serial.print("Temperature: ");
                Serial.println(convertTemperature(temperature));
            }
            break;
        case 0b10:
            Serial.println("No new data since last read");
            break;
        case 0b11:
            Serial.println("Error");
            break;
        default:
            Serial.println("Unknown status");
            break;
    }
}

float ELVH_SPI_Sensor::convertPressure(uint16_t rawPressure) {
    // Extract PPPP(P) and D from sensorModel
    char PPPP[6];
    char D;
    int length = strchr(sensorModel, '-') - sensorModel;
    strncpy(PPPP, sensorModel, length);
    PPPP[length] = '\0';
    D = sensorModel[length + 8];

    // Lookup table for pressure ranges (example values, replace with actual values)
    struct PressureRange {
        const char* range;
        float min;
        float max;
    } pressureRanges[] = {
        {"F50D", -0.5, 0.5}, 
        {"L01D", -1, 1}, 
        {"L02D", -2, 2}, 
        {"L04D", -4, 4}, 
        {"L05D", -5, 5}, 
        {"L10D", -10, 10}, 
        {"L20D", -20, 20}, 
        {"L30D", -30, 30}, 
        {"L60D", -60, 60}, 
        {"L01G", 0, 1}, 
        {"L02G", 0, 2}, 
        {"L04G", 0, 4}, 
        {"L05G", 0, 5}, 
        {"L10G", 0, 10}, 
        {"L20G", 0, 20}, 
        {"L30G", 0, 30}, 
        {"L60G", 0, 60}, 
        {"001D", -1, 1}, 
        {"005D", -5, 5}, 
        {"015D", -15, 15}, 
        {"030D", -30, 30}, 
        {"060D", -60, 60}, 
        {"001G", 0, 1}, 
        {"005G", 0, 5}, 
        {"015G", 0, 15}, 
        {"030G", 0, 30}, 
        {"060G", 0, 60}, 
        {"100G", 0, 100}, 
        {"150G", 0, 150}, 
        {"015A", 0, 15}, 
        {"030A", 0, 30}, 
        {"060A", 0, 60}, 
        {"100A", 0, 100}, 
        {"150A", 0, 150}, 
        {"M100D", -100, 100}, 
        {"M160D", -160, 160}, 
        {"M250D", -250, 250}, 
        {"M500D", -500, 500}, 
        {"M100G", 0, 100}, 
        {"M160G", 0, 160}, 
        {"M250G", 0, 250}, 
        {"M500G", 0, 500}, 
        {"B001D", -1, 1}, 
        {"B001G", 0, 1}, 
        {"BF25G", 0, 2.5}, 
        {"B005G", 0, 5}, 
        {"B010G", 0, 10}, 
        {"B001A", 0, 1}, 
        {"B002A", 0, 2}, 
        {"001D", -1, 1}, 
        {"001G", 0, 1}, 
        {"100G", 0, 100}, 
        {"MF25D", -2.5, 2.5}, 
        {"MF12D", -12.5, 12.5}, 
        {"M025D", -25, 25}, 
        {"M050D", -50, 50}, 
        {"M075D", -75, 75}, 
        {"M100D", -100, 100}, 
        {"M160D", -160, 160}, 
        {"M250D", -250, 250}, 
        {"M500D", -500, 500}, 
        {"MF25G", 0, 2.5}, 
        {"MF12G", 0, 12.5}, 
        {"M025G", 0, 25}, 
        {"M050G", 0, 50}, 
        {"M075G", 0, 75}, 
        {"M100G", 0, 100}, 
        {"M160G", 0, 160}, 
        {"M250G", 0, 250}, 
        {"M500G", 0, 500}, 
        {"MN50G", -500, 0}, 
        {"M611A", 600, 1100}, 
        {"B001D", -1, 1}, 
        {"BF25D", -2.5, 2.5}, 
        {"B005D", -5, 5}, 
        {"B010D", -10, 10}, 
        {"BN01G", -1, 0}, 
        {"B001G", 0, 1}, 
        {"BF25G", 0, 2.5}, 
        {"B005G", 0, 5}, 
        {"B010G", 0, 10}, 
        {"B001A", 0, 1}, 
        {"B002A", 0, 2}
    };
    float minPressure = 0.0;
    float maxPressure = 0.0;
    for (const auto& range : pressureRanges) {
        if (strcmp(PPPP, range.range) == 0) {
            minPressure = range.min;
            maxPressure = range.max;
            break;
        }
        else {
            Serial.println("Invalid sensor model");
        }
    }

    // Calculate the pressure based on the transfer function
    float pressure = 0.0;
    switch (D) {
        case 'A':
            pressure = minPressure + (maxPressure - minPressure) * (rawPressure - 1638) / (14745 - 1638);
            break;
        case 'B':
            pressure = minPressure + (maxPressure - minPressure) * (rawPressure - 819) / (15562 - 819);
            break;
        case 'C':
            pressure = minPressure + (maxPressure - minPressure) * (rawPressure - 819) / (13926 - 819);
            break;
        case 'D':
            pressure = minPressure + (maxPressure - minPressure) * (rawPressure - 655) / (15360 - 655);
            break;
        default:
            pressure = 0.0; // Invalid transfer function
            break;
    }

    return pressure;
}

float ELVH_SPI_Sensor::convertTemperature(uint16_t rawTemperature) {
    return rawTemperature * (200.0 / (2047.0)) - 50.0;
}

float ELVH_SPI_Sensor::getPressure() {
    return convertPressure(pressure);
}

float ELVH_SPI_Sensor::getTemperature() {
    return convertTemperature(temperature);
}