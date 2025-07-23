# Changelog

All notable changes to the ESP32-MQTT-WEATHER configurable sensor project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-07-23

### Added
- **Configurable Sensor System**: Runtime enable/disable of individual sensors through web interface
- **BH1750 Light Sensor**: Fully implemented I2C light sensor with lux readings
- **BME680 Stub**: Framework ready for BME680 environmental sensor implementation
- **Combined MQTT Output**: Single JSON message containing processor metrics and all enabled sensor data
- **Individual Sensor Configuration**: Each sensor has its own MQTT topic configuration
- **NVS Persistence**: Sensor configurations saved to non-volatile storage
- **Web Interface**: Enhanced parameters.html with sensor enable/disable controls
- **Comprehensive Documentation**: Added SENSOR_CONFIGURATION.md with detailed setup instructions

### Changed
- **Project Name**: Updated from "ESP32 MQTT Sensor Template" to "ESP32-MQTT-WEATHER"
- **MQTT Data Format**: Combined processor and sensor data into single message
- **Sensor Architecture**: Modular sensor system replacing template-based approach
- **Version**: Updated to 1.0.0 to reflect major feature addition

### Technical Details
- New sensor_config.h/c modules for sensor management
- sensor_bh1750.c with complete BH1750 implementation
- Updated main.c with configurable sensor reading loop
- Enhanced JSON output combining system and sensor data
- Added sensor initialization during startup based on configuration

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
