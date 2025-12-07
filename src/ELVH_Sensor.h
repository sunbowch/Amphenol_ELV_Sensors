#ifndef ELVH_SENSOR_H
#define ELVH_SENSOR_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// Forward declaration for Adafruit MCP library to avoid an unnecessary heavy include
class Adafruit_MCP23X17;

class ELVH_Sensor {
public:
    enum Unit {
        psi,
        bar,
        mbar,
        ubar,
        Pa,
        inH2O
    };

    enum PressureReference {
        gauge = 0,
        absolute = 1
    };

    ELVH_Sensor(const char* model, uint8_t csPin); // Constructor for SPI
    ELVH_Sensor(const char* model); // Constructor for I2C
    void begin(uint8_t csPin);
    void beginSPI(uint8_t csPin = SS); // Default to hardware CS pin if no csPin is defined
    void beginI2C();
    void readSensorData(uint8_t bytesToRead = 4);
    int getStatus();
    float getPressure();
    float getTemperature();
    bool isBelow(float limit);
    bool isAbove(float limit);
    bool isBetween(float low, float high);
    void setSensorModel(const char* model);
    void setDesiredUnit(Unit unit, PressureReference reference = absolute); // Updated with reference mode
    // Public APIs in desired unit:
    void setZeroOffset(float offsetInDesiredUnit); // NEW: set offset using desired units
    float getZeroOffset() const; // returns offset in desired unit
    void measureZeroOffset(); // New method to measure and set zero offset from current reading
    char sensorModel[20];
    void setCSPin(uint8_t csPin);
    void setMCP(Adafruit_MCP23X17* mcp, uint8_t csPin); // New API to use MCP23X17 expander
    void setMCPMutex(void* mutex); // New: set external mutex for thread-safe MCP access

    // Force bus mode and runtime configuration
    void setI2CMode(bool mode);
    void setI2CAddress(uint8_t addr);

    // SPI helpers
    void setSPIClock(uint32_t hz);
    void deselectCS();

    // State accessors
    bool isI2CMode() const;
    uint8_t getI2CAddress() const;
    uint8_t getCSPin() const;

    // Allow test code or other modules to mark SPI initialized for this library.
    static void setSPIInitialized(bool initialized);

    // Expose model for higher level code to report or populate metadata arrays
    const char* getModel() const;

private:
    float minPressure;
    float maxPressure;
    float pFactor;
    uint16_t pOffset;
    uint16_t pressure;
    uint16_t temperature;
    int status;
    Unit unit;          //default unit of the sensor
    Unit dunit;         //unit to display
    PressureReference pressureRef; // New member for pressure reference mode
    uint16_t zeroOffset;   // Changed to raw unitless sensor value
    
    uint8_t csPin; // New member variable to store the CS pin
    uint8_t i2cAddress; // New member variable to store the I2C address
    bool isI2C; // New member variable to indicate if the sensor is I2C
    bool useMCP = false; // New member to support MCP-controlled CS
    Adafruit_MCP23X17* mcpPtr = nullptr; // Pointer for MCP instance
    void* mcpMutex = nullptr; // Pointer to external mutex (e.g., SemaphoreHandle_t for FreeRTOS)
    void readSPI(uint8_t bytesToRead);
    void readI2C(uint8_t bytesToRead); // Updated method declaration
    void setSensorParameters(); // New method declaration
    float convertPressure(uint16_t rawPressure);
    float convertTemperature(uint16_t rawTemperature);
    float convertToDesiredUnit(float pressure); // New method to convert pressure to the desired unit
    uint32_t spiClock = 800000; // default per-sensor SPI clock in Hz
    // CS helpers that toggle the CS for this sensor
    void assertCS();
    void deassertCS();

    // Raw offset storage and low-level helpers (unitless raw code)
    void setZeroOffsetRaw(uint16_t rawOffset);
    uint16_t getZeroOffsetRaw() const;

    // Centralized unit conversion helpers
    static float unitToPsi(Unit u, float value);
    static float psiToUnit(Unit u, float value);

    // Helper to safely access MCP with mutex protection
    void mcpLock();
    void mcpUnlock();
};

#endif // ELVH_SENSOR_H
