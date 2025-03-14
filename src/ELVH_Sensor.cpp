#include "ELVH_Sensor.h"
#include <Arduino.h>

// Constructor to initialize the sensor with a model and csPin for SPI
ELVH_Sensor::ELVH_Sensor(const char* model, uint8_t csPin) {
    setSensorModel(model);
    this->csPin = csPin;
}

// Constructor to initialize the sensor with a model for I2C
ELVH_Sensor::ELVH_Sensor(const char* model) {
    setSensorModel(model);
}

void ELVH_Sensor::begin() {
    setSensorParameters();
    if (isI2C) {
        beginI2C();
    } else {
        beginSPI(csPin);
    }
}

void ELVH_Sensor::beginSPI(uint8_t csPin) {
    this->csPin = csPin; // Store the CS pin
    this->isI2C = false; // Indicate that this is not an I2C sensor
    SPI.begin();
    SPI.beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE0)); // Set SPI clock to 800 kHz
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH); // Ensure CS is high
}

void ELVH_Sensor::beginI2C() {
    this->isI2C = true; // Indicate that this is an I2C sensor
    Wire.begin();
}

void ELVH_Sensor::setSensorModel(const char* model) {
    strncpy(sensorModel, model, sizeof(sensorModel) - 1);
    sensorModel[sizeof(sensorModel) - 1] = '\0';
}

void ELVH_Sensor::setSensorParameters() {
    // Extract PPPP(P) and D from sensorModel
    char PPPP[6];
    char D;
    int fullScaleSpan=16384;
    float pRef=0;
    int length = strchr(sensorModel, '-') - sensorModel;
    strncpy(PPPP, sensorModel, length);
    PPPP[length] = '\0';
    D = sensorModel[length + 10];
    
    // Lookup table for pressure ranges (example values, replace with actual values)
    struct PressureRange {
        const char* range;
        float min;
        float max;
    } pressureRanges[] = {
        {"F50D", -0.5, 0.5}, 
        {"L01D", -1, 1},{"L02D", -2, 2},{"L04D", -4, 4},{"L05D", -5, 5}, 
        {"L10D", -10, 10},{"L20D", -20, 20},{"L30D", -30, 30},{"L60D", -60, 60}, 
        {"L01G", 0, 1},{"L02G", 0, 2},{"L04G", 0, 4},{"L05G", 0, 5}, 
        {"L10G", 0, 10},{"L20G", 0, 20},{"L30G", 0, 30},{"L60G", 0, 60}, 
        {"001D", -1, 1},{"005D", -5, 5},{"015D", -15, 15},{"030D", -30, 30},{"060D", -60, 60}, 
        {"001G", 0, 1},{"005G", 0, 5},{"015G", 0, 15},{"030G", 0, 30},{"060G", 0, 60}, 
        {"100G", 0, 100},{"150G", 0, 150}, 
        {"015A", 0, 15},{"030A", 0, 30},{"060A", 0, 60},{"100A", 0, 100},{"150A", 0, 150}, 
        {"M100D", -100, 100},{"M160D", -160, 160},{"M250D", -250, 250},{"M500D", -500, 500}, 
        {"M100G", 0, 100},{"M160G", 0, 160},{"M250G", 0, 250},{"M500G", 0, 500}, 
        {"B001D", -1, 1},{"B001G", 0, 1},{"BF25G", 0, 2.5},{"B005G", 0, 5},{"B010G", 0, 10}, 
        {"B001A", 0, 1},{"B002A", 0, 2},
        {"001D", -1, 1},{"001G", 0, 1},{"100G", 0, 100}, 
        {"MF25D", -2.5, 2.5},{"MF12D", -12.5, 12.5},{"M025D", -25, 25},{"M050D", -50, 50}, 
        {"M075D", -75, 75},{"M100D", -100, 100},{"M160D", -160, 160},{"M250D", -250, 250}, 
        {"M500D", -500, 500}, 
        {"MF25G", 0, 2.5},{"MF12G", 0, 12.5}, 
        {"M025G", 0, 25},{"M050G", 0, 50},{"M075G", 0, 75},{"M100G", 0, 100}, 
        {"M160G", 0, 160},{"M250G", 0, 250},{"M500G", 0, 500},{"MN50G", -500, 0}, 
        {"M611A", 600, 1100},
        {"B001D", -1, 1},{"BF25D", -2.5, 2.5},
        {"B005D", -5, 5},{"B010D", -10, 10}, 
        {"BN01G", -1, 0},
        {"B001G", 0, 1}, 
        {"BF25G", 0, 2.5}, 
        {"B005G", 0, 5},{"B010G", 0, 10}, 
        {"B001A", 0, 1},{"B002A", 0, 2}
    };
    minPressure = 0.0;
    maxPressure = 16384.0;
    for (const auto& range : pressureRanges) {
        if (strcmp(PPPP, range.range) == 0) {
            minPressure = range.min;
            maxPressure = range.max;
            Serial.print("Pressure range: ");
            Serial.print(minPressure);
            Serial.print(" to ");
            Serial.println(maxPressure);
            break;
        }
        else {
            Serial.println("Invalid sensor model");
        }
    }
    if (maxPressure + minPressure == 0){   // differential sensors
        pRef = 0;
        switch (D) {
            case 'A':
                fullScaleSpan = 13108;
                pOffset = 8140;
                break;
            case 'B':
                fullScaleSpan = 14746;
                pOffset = 8140;
                break;
            case 'C':
                fullScaleSpan = 13108;
                pOffset = 7373;
                break;
            case 'D':
                fullScaleSpan = 14746;
                pOffset = 8028;
                break;
            default:
                pOffset = 8192;
                fullScaleSpan = 16384; 
                break;
        }
        
    }
    else{
        pRef = minPressure;
        switch (D) {
            case 'A':
                pOffset = 1638;
                fullScaleSpan = 13107;
                break;
            case 'B':
                pOffset = 819;
                fullScaleSpan = 14746;
                break;
            case 'C':
                pOffset = 819; 
                fullScaleSpan = 13107;
                break;
            case 'D':
                pOffset = 655;
                fullScaleSpan = 14746;
                break;
            default:
                pOffset = 0;
                fullScaleSpan = 16384; // Invalid transfer function
                break;
        }
    }
    pFactor = (maxPressure - minPressure) / fullScaleSpan;
    minPressure = pRef;

    // Set default unit based on the first character of the sensor model
    switch (sensorModel[0]) {
        case '0':
        case '1':
            unit = psi;
            break;
        case 'M':
            unit = mbar;
            break;
        case 'B':
            unit = bar;
            break;
        case 'L':
        case 'F':
            unit = inH2O;
            break;
        default:
            unit = psi;
            break;
    }

    // Find the 3rd '-' separator
    const char* thirdDash = strchr(strchr(strchr(sensorModel, '-') + 1, '-') + 1, '-');
    if (thirdDash != nullptr && thirdDash[1] != '\0') {
        char N = thirdDash[2]; // N is the 2nd character after the 3rd '-'
        Serial.print("N: ");
        Serial.println(N);
        if (N == 'S') {
            isI2C = false;
        } else if (N >= '2' && N <= '7') {
            isI2C = true;   
            i2cAddress = (N - '0') << 4 | 0x08; // Set I2C address to 0xN8
        }
    }
}

void ELVH_Sensor::setDesiredUnit(Unit unit) {
    dunit = unit;
}

float ELVH_Sensor::convertToDesiredUnit(float pressure) {
    float pressureInPsi = pressure;

    // Convert from the current unit to psi
    switch (unit) {
        case bar:
            pressureInPsi = pressure / 0.0689476;
            break;
        case mbar:
            pressureInPsi = pressure / 68.9476;
            break;
        case inH2O:
            pressureInPsi = pressure / 27.6807;
            break;
        case psi:
        default:
            break;
    }

    // Convert from psi to the desired unit
    switch (dunit) {
        case bar:
            return pressureInPsi * 0.0689476;
        case mbar:
            return pressureInPsi * 68.9476;
        case inH2O:
            return pressureInPsi * 27.6807;
        case psi:
        default:
            return pressureInPsi; // Default to psi if unit is not recognized
    }
}

void ELVH_Sensor::readSensorData(uint8_t bytesToRead) {
    if (isI2C) {
        readI2C(bytesToRead);
    } else {
        digitalWrite(csPin, LOW); // Pull CS low to start communication
        readSPI(bytesToRead);
        digitalWrite(csPin, HIGH); // Pull CS high to end communication
    }
}

void ELVH_Sensor::readI2C(uint8_t bytesToRead) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x00); // Dummy write to initiate read
    Wire.endTransmission(false); // Send repeated start

    Wire.requestFrom(i2cAddress, bytesToRead); // Request bytesToRead bytes

    if (Wire.available() >= 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        status = (msb >> 6) & 0x03;
        pressure = ((msb & 0x3F) << 8) | lsb;
    }

    if (bytesToRead >= 3 && Wire.available() >= 1) {
        uint8_t msb = Wire.read();
        temperature = msb << 3;
    }

    if (bytesToRead == 4 && Wire.available() >= 1) {
        uint8_t lsb = Wire.read();
        temperature |= (lsb >> 5) & 0x07;
    }

    Wire.endTransmission();
}

void ELVH_Sensor::readSPI(uint8_t bytesToRead) {
    uint32_t response = 0;
    for (int i = 0; i < bytesToRead; i++) {
        response <<= 8;
        response |= SPI.transfer(0x00); // Send dummy byte to receive data
    }

    if (bytesToRead == 2) {
        status = (response >> 14) & 0x03;
        pressure = response & 0x3FFF;
        temperature = 0; // Temperature data is not available
    } else if (bytesToRead == 3) {
        status = (response >> 22) & 0x03;
        pressure = (response >> 8) & 0x3FFF;
        temperature = (response & 0xFF) << 3; // Only MSB of temperature is available
    } else if (bytesToRead == 4) {
        status = (response >> 30) & 0x03;
        pressure = (response >> 16) & 0x3FFF;
        temperature = (response & 0xFFFF) >> 5;
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
            Serial.println(convertToDesiredUnit(convertPressure(pressure)));
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

float ELVH_Sensor::convertPressure(uint16_t rawPressure) {
    // Calculate the pressure based on the transfer function
    float pressure = minPressure + (rawPressure - pOffset) * pFactor;
    return pressure;
}

float ELVH_Sensor::convertTemperature(uint16_t rawTemperature) {
    return rawTemperature * (200.0 / (2047.0)) - 50.0;
}

float ELVH_Sensor::getPressure() {
    return convertToDesiredUnit(convertPressure(pressure));
}

float ELVH_Sensor::getTemperature() {
    return convertTemperature(temperature);
}

int ELVH_Sensor::getStatus() {
    switch (status) {
        case 0b00:
            Serial.println("Ready");
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
    return status;
}

// Method to set the CS pin to another value
void ELVH_Sensor::setCSPin(uint8_t csPin) {
    this->csPin = csPin;
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH); // Ensure CS is high
}