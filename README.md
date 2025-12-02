# Amphenol ELV Sensors Library

An Arduino library for interfacing with Amphenol ELV series digital pressure sensors via SPI or I2C.

## Features

- Supports both SPI and I2C communication modes
- Automatic bus mode detection based on sensor model
- Support for multiple sensor ranges and units (psi, bar, mbar, inH2O)
- Compatible with Arduino, ESP32, ESP8266, and SAMD boards
- Support for MCP23X17 GPIO expander for additional chip-select pins

## Supported Sensor Models

The library supports various Amphenol ELV series sensors including:
- **ELVH** series: High accuracy pressure sensors
- Differential sensors (e.g., L01D, L02D, M100D)
- Gauge sensors (e.g., L01G, 015G, 150G)
- Absolute sensors (e.g., 015A, 030A, M611A)

## Installation

### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps =
    https://github.com/sunbowch/Amphenol_ELV_Sensors
```

### Arduino IDE
1. Download this repository as a ZIP file
2. In Arduino IDE: Sketch > Include Library > Add .ZIP Library
3. Select the downloaded ZIP file

## Usage

### How It Works

The library uses a two-step process to read sensor data:

1. **`readSensorData()`** - Initiates communication over SPI or I2C and retrieves raw data from the sensor. This function performs the actual bus transaction and stores the values internally.
2. **Getter functions** (`getPressure()`, `getTemperature()`, `getStatus()`) - Return the processed values from the last read operation.

This design pattern allows you to:
- Read sensor data once per cycle, minimizing bus traffic
- Access the same reading multiple times without re-querying the sensor
- Check status before using pressure/temperature values
- Optimize timing-critical applications

**Important:** Always call `readSensorData()` before using any getter functions to ensure you're working with current values.

### Understanding bytesToRead Parameter

The `readSensorData(bytesToRead)` function accepts a parameter that determines how much data is read from the sensor:

- **`readSensorData(2)`** - Reads 2 bytes
  - Status (2 bits)
  - Pressure (14 bits)
  - Temperature data: **Not available** (returns 0)

- **`readSensorData(3)`** - Reads 3 bytes
  - Status (2 bits)
  - Pressure (14 bits)
  - Temperature MSB only (8 bits) - **Reduced accuracy**

- **`readSensorData(4)`** - Reads 4 bytes (default, recommended)
  - Status (2 bits)
  - Pressure (14 bits)
  - Temperature (11 bits) - **Full accuracy**

**Recommendation:** Use `readSensorData(4)` for most applications to get complete pressure and temperature data. Use fewer bytes only when temperature is not needed or to optimize communication speed.

### Basic SPI Example
```cpp
#include <Arduino.h>
#include "ELVH_Sensor.h"

ELVH_Sensor sensor("150G-HAND-C-PSA4", 5); // Model and CS pin

void setup() {
    Serial.begin(115200);
    sensor.begin();
    sensor.setDesiredUnit(ELVH_Sensor::bar);
}

void loop() {
    sensor.readSensorData(4);
    Serial.print("Pressure: ");
    Serial.println(sensor.getPressure());
    Serial.print("Temperature: ");
    Serial.println(sensor.getTemperature());
    delay(1000);
}
```

### I2C Example
```cpp
#include <Arduino.h>
#include "ELVH_Sensor.h"

ELVH_Sensor sensor("M100D-HAND-C-P2A4"); // I2C address determined by model

void setup() {
    Serial.begin(115200);
    sensor.begin();
    sensor.setDesiredUnit(ELVH_Sensor::mbar);
}

void loop() {
    sensor.readSensorData(4);
    Serial.print("Pressure: ");
    Serial.println(sensor.getPressure());
    delay(1000);
}
```

### Reading Data Efficiently

```cpp
void loop() {
    // Step 1: Trigger the data bus transaction
    sensor.readSensorData(4); // Reads 4 bytes: status, pressure, and temperature
    
    // Step 2: Check status before using values
    int status = sensor.getStatus();
    if (status == 0b00) { // No error
        // Step 3: Use getter functions to access the stored values
        Serial.print("Pressure: ");
        Serial.println(sensor.getPressure());
        Serial.print("Temperature: ");
        Serial.println(sensor.getTemperature());
    }
    
    delay(1000);
}
```

### Reading Pressure Only (Faster)

```cpp
void loop() {
    // Read only 2 bytes for faster communication
    sensor.readSensorData(2); // Status + Pressure only
    
    if (sensor.getStatus() == 0b00) {
        Serial.print("Pressure: ");
        Serial.println(sensor.getPressure());
        // getTemperature() would return 0
    }
    
    delay(100);
}
```

## API Reference

### Constructor
- `ELVH_Sensor(const char* model, uint8_t csPin)` - For SPI sensors
- `ELVH_Sensor(const char* model)` - For I2C sensors

### Methods
- `begin()` - Initialize the sensor
- `readSensorData(uint8_t bytesToRead = 4)` - **Trigger bus transaction** and read sensor data (2, 3, or 4 bytes)
- `getPressure()` - Get pressure in the desired unit from last read
- `getTemperature()` - Get temperature in Celsius from last read
- `getStatus()` - Get sensor status from last read
- `setDesiredUnit(Unit unit, PressureReference reference = absolute)` - Set output pressure unit and reference mode
- `setZeroOffset(uint16_t rawOffset)` - Set zero offset using raw sensor value (unitless)
- `getZeroOffset()` - Get current zero offset as raw sensor value
- `measureZeroOffset()` - Measure and set zero offset from current sensor reading
- `isBelow(float limit)` - Returns true if pressure is below the specified limit
- `isAbove(float limit)` - Returns true if pressure is above the specified limit
- `isBetween(float low, float high)` - Returns true if pressure is between the specified limits


### Status Codes
- `0b00` - Normal operation, data valid
- `0b10` - No new data since last read
- `0b11` - Diagnostic condition (error)

### Units
- `ELVH_Sensor::psi`
- `ELVH_Sensor::bar`
- `ELVH_Sensor::mbar`
- `ELVH_Sensor::inH2O`

### Pressure Reference Modes
- `ELVH_Sensor::absolute` - Absolute pressure (default)
- `ELVH_Sensor::gauge` - Gauge pressure (relative to zero offset)

### Using Gauge Pressure

To configure a sensor for gauge pressure readings:

```cpp
void setup() {
    Serial.begin(115200);
    sensor.begin();
    
    // Read current pressure (should be at atmospheric/reference condition)
    sensor.readSensorData(4);
    
    // Measure and set zero offset automatically
    sensor.measureZeroOffset();
    
    // Set to gauge mode with bar units
    sensor.setDesiredUnit(ELVH_Sensor::bar, ELVH_Sensor::gauge);
}

void loop() {
    sensor.readSensorData(4);
    
    // Now getPressure() returns gauge pressure (relative to zero offset)
    Serial.print("Gauge Pressure: ");
    Serial.println(sensor.getPressure());
    
    delay(1000);
}
```

**Note:** The zero offset is stored as a raw sensor value (unitless), so it remains valid even if you change the output unit later.

## Documentation

See the [ELV Series datasheet](datasheet/ELV_Series.pdf) for detailed sensor specifications.

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.
