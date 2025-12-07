#include "ELVH_Sensor.h"
#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <SPI.h>
#include <Wire.h>

// File-local SPI initialization flag and setter (keeps SPI init tracking within this module)
static volatile bool spiInitialized = false;
static inline void setSPIInitializedLocal(bool initialized = true) { spiInitialized = initialized; }

// Provide local aliases used below so file uses its local flag instead of a global symbol
// Keep names different to avoid clashing with any global inline symbols.
// Replace usage in this file: where 'setSPIInitialized' and 'spiInitialized' are used,
// they will now resolve to local symbols by adding explicit calls below if necessary.

// Constructor to initialize the sensor with a model and csPin for SPI
ELVH_Sensor::ELVH_Sensor(const char* model, uint8_t csPin) {
    setSensorModel(model);
    // Use sentinel 255 to indicate no MCU CS assigned
    this->csPin = (csPin == 0) ? 255 : csPin;  // avoid default 0
    this->useMCP = false;
    this->mcpPtr = nullptr;
    this->zeroOffset = 0; // Initialize zero offset (raw value)
    this->pressureRef = absolute; // Default to absolute
}

// Constructor to initialize the sensor with a model for I2C
ELVH_Sensor::ELVH_Sensor(const char* model) {
    setSensorModel(model);
    this->csPin = 255; // sentinel: no CS assigned by default
    this->useMCP = false;
    this->mcpPtr = nullptr;
    this->zeroOffset = 0; // Initialize zero offset (raw value)
    this->pressureRef = absolute; // Default to absolute
}

// New: set external mutex for thread-safe MCP access
void ELVH_Sensor::setMCPMutex(void* mutex) {
    mcpMutex = mutex;
    Serial.println("ELVH_Sensor::setMCPMutex configured");
}

// Helper: lock MCP mutex if available
void ELVH_Sensor::mcpLock() {
    #ifdef configUSE_PREEMPTION
    // FreeRTOS environment detected
    if (mcpMutex != nullptr) {
        if (xSemaphoreTake((SemaphoreHandle_t)mcpMutex, portMAX_DELAY) != pdTRUE) {
            Serial.println("ELVH_Sensor::mcpLock ERROR: Failed to acquire mutex!");
            return;
        }
    }
    #endif
}

// Helper: unlock MCP mutex if available
void ELVH_Sensor::mcpUnlock() {
    #ifdef configUSE_PREEMPTION
    // FreeRTOS environment detected
    if (mcpMutex != nullptr) {
        xSemaphoreGive((SemaphoreHandle_t)mcpMutex);
    }
    #endif
}

// New method: configure MCP controller and MCP pin used as CS
void ELVH_Sensor::setMCP(Adafruit_MCP23X17* mcp, uint8_t csPin) {
    Serial.print("ELVH_Sensor::setMCP enter csPin="); Serial.println(csPin);
    Serial.flush();
    // Note: No mutex needed here - this is a setup function, not runtime I/O
    this->mcpPtr = mcp;
    // Validate csPin for MCP range
    if (csPin < 16 && mcpPtr != nullptr) {
        this->useMCP = true;
        this->csPin = csPin;
        mcpPtr->pinMode(csPin, OUTPUT);
        mcpPtr->digitalWrite(csPin, HIGH); // default deselected
        Serial.print("ELVH_Sensor::setMCP ok csPin="); Serial.println(csPin);
        Serial.flush();
    } else {
        this->useMCP = false;
        this->mcpPtr = nullptr;
        Serial.print("ELVH_Sensor::setMCP invalid csPin="); Serial.println(csPin);
        Serial.flush();
    }
    Serial.print("ELVH_Sensor::setMCP exit csPin="); Serial.println(csPin);
    Serial.flush();
}

void ELVH_Sensor::setCSPin(uint8_t csPin) {
    this->csPin = csPin;
    this->useMCP = false;
    this->mcpPtr = nullptr;
    if (csPin != 255 && csPin >= 11 && csPin <= 39) {
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH); // CS inactive (HIGH for active-low)
    } else {
        Serial.print("setCSPin: skipping pinMode for csPin=");
        Serial.println(csPin);
    }
    Serial.print("setCSPin: csPin="); Serial.print(csPin); Serial.print(", useMCP=");
    Serial.println(useMCP ? "YES" : "NO");
}

void ELVH_Sensor::begin( uint8_t csPin) {
    setSensorParameters();
    if (isI2C) {
        beginI2C();
    } else {
        beginSPI(csPin);
    }
}

void ELVH_Sensor::beginSPI(uint8_t csPin) {
    Serial.print("ELVH_Sensor::beginSPI enter csPin="); Serial.println(csPin);
    this->csPin = csPin == 0 ? 255 : csPin; // treat 0 as sentinel
    this->isI2C = false; // Indicate that this is not an I2C sensor

    // Ensure SPI.begin is called only once locally; if not called, auto-init (warn)
    if (!spiInitialized) {
        Serial.println("Warning: SPI not initialized globally; calling SPI.begin() with default pins.");
        SPI.begin(); // default VSPI pins (or your chosen default)
        SPI.beginTransaction(SPISettings(spiClock, MSBFIRST, SPI_MODE0));
        setSPIInitializedLocal(true);
        Serial.println("SPI.begin() (auto) done.");
    }
    delay(2);

    // Configure only valid MCU pins (>=11), or MCP pins
    if (!useMCP) {
        if (this->csPin != 255 && this->csPin >= 11 && this->csPin <= 39) {
            pinMode(this->csPin, OUTPUT);
            digitalWrite(this->csPin, HIGH); // CS inactive
        } else {
            Serial.print("beginSPI: not configuring MCU CS for pin "); Serial.println(this->csPin);
        }
    } else if (mcpPtr) {
        mcpPtr->pinMode(this->csPin, OUTPUT);
        mcpPtr->digitalWrite(this->csPin, HIGH); // CS inactive
    }
    Serial.print("ELVH_Sensor::beginSPI exit csPin="); Serial.println(csPin);
    Serial.flush();
    delay(2);
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
    Serial.println("ELVH_Sensor::setSensorParameters enter");
    Serial.flush();
    // Extract PPPP (first token) safely and transfer function D from PPPP's last char
    char PPPP[6] = {0};
    char D = '\0';
    int fullScaleSpan=16384;
    float pRef=0;
    const char* dash = strchr(sensorModel, '-');
    int length = dash ? (dash - sensorModel) : (int)strlen(sensorModel);
    if (length > (int)sizeof(PPPP)-1) length = sizeof(PPPP)-1;
    strncpy(PPPP, sensorModel, length);
    PPPP[length] = '\0';

    if (length > 0) {
        D = PPPP[length-1];
    } else {
        D = '\0';
    }

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

    bool found = false;
    for (const auto& range : pressureRanges) {
        if (strcmp(PPPP, range.range) == 0) {
            minPressure = range.min;
            maxPressure = range.max;
            Serial.print("Pressure range: ");
            Serial.print(minPressure);
            Serial.print(" to ");
            Serial.println(maxPressure);
            found = true;
            break;
        }
    }
    if (!found) {
        Serial.print("Invalid sensor model: ");
        Serial.println(sensorModel);
    }
    Serial.println("ELVH_Sensor::setSensorParameters exit");
    Serial.flush();

    if ((maxPressure + minPressure) == 0) { // differential sensors
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
    } else {
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
                fullScaleSpan = 16384;
                break;
        }
    }
    pFactor = (maxPressure - minPressure) / fullScaleSpan;
    minPressure = pRef;

    // Default unit based on first char
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

    // Find the 3rd '-' separator safely
    const char* p = sensorModel;
    const char* firstDash = strchr(p, '-');
    const char* secondDash = firstDash ? strchr(firstDash + 1, '-') : nullptr;
    const char* thirdDash = secondDash ? strchr(secondDash + 1, '-') : nullptr;
    if (thirdDash != nullptr && thirdDash[1] != '\0') {
        char N = thirdDash[2]; // first char after '-' then index 1/2 used earlier
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

void ELVH_Sensor::setDesiredUnit(Unit unit, PressureReference reference) {
    dunit = unit;
    pressureRef = reference;
}

// Private: set raw zero offset
void ELVH_Sensor::setZeroOffsetRaw(uint16_t rawOffset) {
    zeroOffset = rawOffset;
    Serial.print("ELVH_Sensor::setZeroOffsetRaw raw="); Serial.println(zeroOffset);
}

// Private: get raw zero offset
uint16_t ELVH_Sensor::getZeroOffsetRaw() const {
    return zeroOffset;
}

// Centralized unit conversion helpers
float ELVH_Sensor::unitToPsi(Unit u, float value) {
    switch (u) {
        case bar:   return value / 0.0689476f;
        case mbar:  return value / 68.9476f;
        case ubar:  return value / 68947.57f;
        case Pa:    return value / 6894.76f; // Assuming 1 Pa = 0.000145038 psi
        case inH2O: return value / 27.6807f;
        case psi:
        default:    return value;
    }
}

float ELVH_Sensor::psiToUnit(Unit u, float valuePsi) {
    switch (u) {
        case bar:   return valuePsi * 0.0689476f;
        case mbar:  return valuePsi * 68.9476f;
        case ubar:  return valuePsi * 68947.6f;
        case Pa:    return valuePsi * 6894.76f; // Assuming 1 Pa = 0.000145038 psi
        case inH2O: return valuePsi * 27.6807f;
        case psi:
        default:    return valuePsi;
    }
}

// Public: set zero offset using desired unit value
void ELVH_Sensor::setZeroOffset(float offsetInDesiredUnit) {
    // desired unit -> psi
    float offsetPsi = unitToPsi(dunit, offsetInDesiredUnit);
    // psi -> sensor native unit
    float native = psiToUnit(unit, offsetPsi);
    // native -> raw using inverse transfer function
    float rawF = (native - minPressure) / pFactor + pOffset;
    if (rawF < 0) rawF = 0;
    if (rawF > 65535.0f) rawF = 65535.0f;
    setZeroOffsetRaw(static_cast<uint16_t>(rawF));

    Serial.print("ELVH_Sensor::setZeroOffset desired="); Serial.print(offsetInDesiredUnit);
    Serial.print(" -> raw="); Serial.println(getZeroOffsetRaw());
}

// Public: get zero offset in desired unit
float ELVH_Sensor::getZeroOffset() const {
    // raw -> native pressure
    float native = minPressure + (static_cast<float>(getZeroOffsetRaw()) - pOffset) * pFactor;
    // native -> desired unit
    float psiVal = unitToPsi(unit, native);
    return psiToUnit(dunit, psiVal);
}

void ELVH_Sensor::measureZeroOffset() {
    readSensorData(2); // read pressure only
    setZeroOffsetRaw(pressure);
    Serial.print("ELVH_Sensor::measureZeroOffset raw="); Serial.println(getZeroOffsetRaw());
}

float ELVH_Sensor::convertToDesiredUnit(float pressure) {
    // Convert input from sensor native unit to psi, then to desired unit
    float pressureInPsi = unitToPsi(unit, pressure);
    return psiToUnit(dunit, pressureInPsi);
}

void ELVH_Sensor::readSensorData(uint8_t bytesToRead) {
    if (isI2C) {
        readI2C(bytesToRead);
    } else {
        if (useMCP && mcpPtr && csPin < 16) {
            mcpLock();
            mcpPtr->digitalWrite(csPin, LOW); // Assert CS
            while (mcpPtr->digitalRead(csPin) != LOW) {
                yield();
            }
        } else if (csPin < 255) {
            digitalWrite(csPin, LOW); // Assert CS
            while (digitalRead(csPin) != LOW) {
                yield();
            }
        }
        readSPI(bytesToRead);

        if (useMCP && mcpPtr && csPin < 16) {
            mcpPtr->digitalWrite(csPin, HIGH); // Deassert CS
            delayMicroseconds(2);
            mcpUnlock();
        } else if (csPin < 255) {
            digitalWrite(csPin, HIGH); // Deassert CS
            delayMicroseconds(2);
        }
    }
}

void ELVH_Sensor::readI2C(uint8_t bytesToRead) {

    const int MAX_RETRIES = 3;
    int attempt;
    int rc = -1;
    int received = 0;
    for (attempt = 0; attempt < MAX_RETRIES; ++attempt) {
        Wire.beginTransmission(i2cAddress);
        Wire.write(0x00); // Dummy write to initiate read
        rc = Wire.endTransmission(false); // Send repeated start
        if (rc != 0) {
            //Serial.print("ELVH_Sensor::readI2C endTransmission rc=");
            //Serial.println(rc);
            delay(10);
            continue;
        }

        // Request bytes and check how many were received
        received = Wire.requestFrom((int)i2cAddress, (int)bytesToRead);
        if (received == 0) {
            //Serial.print("ELVH_Sensor::readI2C requestFrom returned 0, attempt ");
            //Serial.println(attempt + 1);
            delay(10);
            continue;
        }

        // Now read bytes safely depending on how many arrived
        if (received >= 2 && Wire.available() >= 2) {
            uint8_t msb = Wire.read();
            uint8_t lsb = Wire.read();
            status = (msb >> 6) & 0x03;
            pressure = ((msb & 0x3F) << 8) | lsb;
        } else {
            // Not enough data to determine pressure; mark as error and retry
            status = 0xFF;
            pressure = 0;
            temperature = 0;
            if (received < 2) {
                Serial.println("ELVH_Sensor::readI2C not enough data for status+pressure");
                delay(10);
                continue;
            }
        }

        // Read temperature if present
        if (bytesToRead >= 3 && received >= 3 && Wire.available() >= 1) {
            uint8_t msb = Wire.read();
            temperature = msb << 3;
        } else if (bytesToRead >= 3) {
            // missing byte(s)
            Serial.println("ELVH_Sensor::readI2C missing MSB temperature");
        }

        if (bytesToRead == 4 && received >= 4 && Wire.available() >= 1) {
            uint8_t lsb = Wire.read();
            temperature |= (lsb >> 5) & 0x07;
        } else if (bytesToRead == 4) {
            Serial.println("ELVH_Sensor::readI2C missing LSB temperature");
        }

        // Success path
        break;
    }

    if (attempt == MAX_RETRIES) {
        // we failed all attempts; mark as error
        status = 0xFF;
        pressure = 0;
        temperature = 0;
        Serial.print("ELVH_Sensor::readI2C FAILED after ");
        Serial.print(MAX_RETRIES);
        Serial.print(" attempts, final rc=");
        Serial.print(rc);
        Serial.print(", received=");
        Serial.println(received);
    } else {
        // Debug print parsed values if successful
        //Serial.print("ELVH_Sensor::readI2C success status=0x");
        //Serial.print(status, HEX);
        //Serial.print(" pressure=");
        //Serial.print(pressure);
        //Serial.print(" temperature_raw=");
        //Serial.println(temperature);
    }
}

// Global wrapper: exposes a function that other modules can call to mark SPI as initialized.
// For safety we forward to the file-local setter; keep variable module-local.
void ELVH_Sensor::setSPIInitialized(bool initialized /* = true */) {
    setSPIInitializedLocal(initialized);
}

// Allow per-sensor SPI clock tuning
void ELVH_Sensor::setSPIClock(uint32_t hz) {
    if (hz < 100000) hz = 100000;
    if (hz > 800000) hz = 800000;
    spiClock = hz;
    Serial.print("ELVH_Sensor::setSPIClock set to "); Serial.println(spiClock);
}

void ELVH_Sensor::assertCS() {
    if (useMCP && mcpPtr && csPin < 16 && csPin >= 0) {
        mcpLock();
        mcpPtr->digitalWrite(csPin, LOW); // Assert CS (active-low)
        mcpUnlock();
    } else if (csPin != 255 && csPin >= 11 && csPin <= 39) {
        digitalWrite(csPin, LOW); // Assert CS (active-low)
    } else {
        Serial.print("assertCS: skipping for csPin="); Serial.println(csPin);
    }
}

void ELVH_Sensor::deassertCS() {
    if (useMCP && mcpPtr && csPin < 16 && csPin >= 0) {
        mcpLock();
        mcpPtr->digitalWrite(csPin, HIGH); // Deassert CS (active-low)
        mcpUnlock();
    } else if (csPin != 255 && csPin >= 11 && csPin <= 39) {
        digitalWrite(csPin, HIGH); // Deassert CS (active-low)
    } else {
        Serial.print("deassertCS: skipping for csPin="); Serial.println(csPin);
    }
}

void ELVH_Sensor::deselectCS() {
    deassertCS();
}

// readSPI: use assert/deassert and keep delay settle
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

    //Serial.print("Status: ");
    //Serial.println(status, BIN);
    switch (status) {
        case 0b00:
            //Serial.println("No error");
            //Serial.print("Pressure: ");
            //Serial.println(convertToDesiredUnit(convertPressure(pressure)));
            if (bytesToRead >= 3) {
                //Serial.print("Temperature: ");
                //Serial.println(convertTemperature(temperature));
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
    // Apply zero offset for gauge mode at the raw level
    uint16_t adjustedPressure = rawPressure;
    if (pressureRef == gauge) {
        adjustedPressure = rawPressure - zeroOffset;
    }
    
    // Calculate the pressure based on the transfer function
    float pressure = minPressure + (adjustedPressure - pOffset) * pFactor;
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

bool ELVH_Sensor::isBelow(float limit) {
    return getPressure() < limit;
}

bool ELVH_Sensor::isAbove(float limit) {
    return getPressure() > limit;
}

bool ELVH_Sensor::isBetween(float low, float high) {
    if (low > high) { float t = low; low = high; high = t; } // normalize bounds
    float p = getPressure();
    return p >= low && p <= high; // inclusive
}

int ELVH_Sensor::getStatus() {
    switch (status) {
        case 0b00:
            //Serial.println("Ready");
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

// New: override the I2C address at runtime (used for diagnostics or non-standard mapping)
void ELVH_Sensor::setI2CAddress(uint8_t addr) {
    i2cAddress = addr;
    isI2C = true;
    Serial.print("ELVH_Sensor::setI2CAddress set to 0x");
    Serial.println(i2cAddress, HEX);
}

// New: override I2C mode explicitly (force SPI when false)
void ELVH_Sensor::setI2CMode(bool mode) {
    isI2C = mode;
    Serial.print("ELVH_Sensor::setI2CMode set to ");
    Serial.println(isI2C ? "I2C" : "SPI");
}

// Add simple const getters to expose internal state safely
bool ELVH_Sensor::isI2CMode() const {
    return isI2C;
}

uint8_t ELVH_Sensor::getI2CAddress() const {
    return i2cAddress;
}

uint8_t ELVH_Sensor::getCSPin() const {
    return csPin;
}

const char* ELVH_Sensor::getModel() const {
    return sensorModel;
}
