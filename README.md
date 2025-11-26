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

## API Reference

### Constructor
- `ELVH_Sensor(const char* model, uint8_t csPin)` - For SPI sensors
- `ELVH_Sensor(const char* model)` - For I2C sensors

### Methods
- `begin()` - Initialize the sensor
- `readSensorData(uint8_t bytesToRead = 4)` - Read sensor data
- `getPressure()` - Get pressure in the desired unit
- `getTemperature()` - Get temperature in Celsius
- `getStatus()` - Get sensor status
- `setDesiredUnit(Unit unit)` - Set output pressure unit

### Units
- `ELVH_Sensor::psi`
- `ELVH_Sensor::bar`
- `ELVH_Sensor::mbar`
- `ELVH_Sensor::inH2O`

## Documentation

See the [ELV Series datasheet](datasheet/ELV_Series.pdf) for detailed sensor specifications.

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.
