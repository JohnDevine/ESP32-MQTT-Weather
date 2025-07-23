# Changelog

All notable changes to this ESP32 BH1750 Light Sensor MQTT project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.0.1] - 2025-07-20

### Added
- **BH1750 Light Sensor Integration**: I2C communication with BH1750 ambient light sensor
- **MQTT Publishing**: JSON data publishing with light readings in lux units  
- **Web Configuration**: Captive portal for WiFi and MQTT broker setup
- **System Monitoring**: WiFi signal, CPU temperature, system metrics
- **Automatic Version Management**: CMake-based version synchronization
- **Watchdog Protection**: MQTT-publish timeout monitoring with ESP32 reset
- **Clean Codebase**: Removed all UART/BMS template code, I2C-only implementation

### Features
- **Light Measurement**: 1-65535 lux range with 16-bit resolution
- **I2C Communication**: GPIO21 (SDA), GPIO22 (SCL), address 0x23
- **MQTT JSON**: Processor metrics + light sensor data in structured format
- **Web Interface**: Device configuration and system information display
- **OTA Updates**: Over-the-air firmware and filesystem updates
- **NVS Storage**: Persistent configuration across reboots

### Technical Details
- **Framework**: ESP-IDF with new I2C master driver
- **Sensor**: BH1750 continuous high-resolution mode
- **Data Structure**: `input_data_t` with `float sunlight` and `bool sensor_ok`
- **Version Control**: Single source of truth in CMakeLists.txt with auto-generated headers
