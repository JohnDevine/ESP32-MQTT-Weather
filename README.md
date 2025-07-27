# ESP32-MQTT-WEATHER Configurable Sensor System

## Overview
This is an ESP32-based configurable sensor monitoring system for IoT applications. It provides a complete foundation for collecting data from multiple sensors and publishing it to MQTT. The system includes web-based configuration, over-the-air updates, and robust connectivity with automatic recovery features.

**Key Feature**: Runtime configurable sensors! Enable/disable individual sensors through the web interface without code changes. Currently supports BH1750 light sensor and BME680 environmental sensor with framework ready for additional sensors.

## Features
- **Configurable Sensor Support**: Runtime enable/disable of individual sensors through web interface
- **Multiple Sensor Types**: BH1750 light sensor (implemented), BME680 environmental (implemented)
- **Individual MQTT Topics**: Each sensor publishes to its own configurable topic
- **MQTT Publishing**: Automatic data publishing with configurable intervals
- **Web Configuration**: Captive portal for easy WiFi, MQTT, and sensor setup
- **Over-the-Air Updates**: Remote firmware and file system updates
- **System Monitoring**: WiFi signal strength, CPU temperature, and system metrics
- **Watchdog Protection**: Automatic recovery from system failures
- **Persistent Configuration**: Settings stored in ESP32 NVS (Non-Volatile Storage)

## Customization and Extension Points
**NEW**: This project now includes a configurable sensor system! See [SENSOR_CONFIGURATION.md](SENSOR_CONFIGURATION.md) for detailed documentation.

### Quick Start with BH1750
The BH1750 light sensor is fully implemented and ready to use:
1. Connect BH1750 to ESP32 (VCC->3.3V, GND->GND, SDA->GPIO21, SCL->GPIO22)
2. Flash the firmware and enter configuration mode (hold boot button during power-on)
3. Enable BH1750 sensor in the web interface
4. Configure MQTT topic (default: `sensors/light/bh1750`)
5. Save and reboot - sensor data will be published automatically

### Quick Start with BME680
The BME680 environmental sensor is fully implemented and ready to use:
1. Connect BME680 to ESP32 (VCC->3.3V, GND->GND, SDA->GPIO21, SCL->GPIO22, SDO->GND)
2. Flash the firmware and enter configuration mode (hold boot button during power-on)
3. Enable BME680 sensor in the web interface
4. Configure MQTT topic (default: `sensors/environment/bme680`)
5. Save and reboot - sensor data will be published automatically

**Note**: BME680 uses I2C address 0x76 (SDO pin connected to GND). Both sensors share the same I2C bus without conflicts.

### Adding More Sensors
The framework supports easy addition of new sensors. BME680 environmental sensor is now fully implemented alongside BH1750.
Search for "TEMPLATE:" comments in the source code to find areas that need customization:
- **Sensor Configuration**: Pin assignments and communication protocol setup
- **Data Structure**: Define your sensor-specific data fields
- **Sensor Initialization**: Add your sensor's initialization sequence
- **Data Reading**: Implement your sensor's reading logic
- **JSON Output**: Customize the MQTT data format

## Supported Sensor Examples
This project includes a configurable sensor system with runtime enable/disable:

### Currently Implemented
- **BH1750**: Light sensor (I2C) - fully implemented with responsive readings
  - Default topic: `sensors/light/bh1750`  
  - Data: Light intensity in lux with real-time responsiveness
  - Implementation: One-time measurement mode for accurate readings
  - Configuration: Enable/disable via web interface

- **BME680**: Environmental sensor (I2C) - fully implemented for environmental monitoring
  - Default topic: `sensors/environment/bme680`
  - Data: Temperature (°C), humidity (%), pressure (hPa) 
  - Gas sensor: Basic resistance reading (advanced gas analysis via BSEC library - see enhancement below)
  - Configuration: Enable/disable via web interface

> **🚀 Future Enhancement - BSEC Library Integration**
> 
> **TODO**: Consider implementing Bosch BSEC (Bosch Sensortec Environmental Cluster) library for advanced BME680 gas sensor functionality.
> 
> **Benefits of BSEC Integration:**
> - **Professional Air Quality Monitoring**: Indoor Air Quality (IAQ) index (0-500 scale)
> - **Advanced Gas Analysis**: CO₂ equivalent and VOC equivalent measurements  
> - **Intelligent Calibration**: Self-calibrating algorithms with baseline management
> - **Breath VOC Detection**: Enhanced volatile organic compound detection
> - **Static IAQ Assessment**: Air quality without motion dependency
> - **Accuracy Enhancement**: Professional-grade sensor fusion algorithms
> 
> **Implementation Complexity**: Moderate (6/10) - requires proprietary library integration, calibration system, and extended testing period for baseline establishment.
> 
> **Estimated Benefits**: Transform basic environmental monitoring into professional-grade air quality assessment suitable for smart home automation, health monitoring, and industrial applications.
> 
> **Target Applications**: Smart HVAC control, indoor air quality alerts, health monitoring systems, building automation with air quality-based ventilation control.

### Framework Ready (Implementation Needed)

### Easily Adaptable For
- **DHT22**: Temperature and humidity sensor (GPIO)
- **DS18B20**: Temperature sensor (1-Wire)
- **MQ sensors**: Gas sensors (ADC)
- **Analog sensors**: Using ESP32's ADC
- **BME280**: Temperature, humidity, pressure (I2C/SPI)
- **SHT30**: Temperature and humidity (I2C)

See [SENSOR_CONFIGURATION.md](SENSOR_CONFIGURATION.md) for detailed sensor addition instructions.

## Hardware Components
- **ESP32 Development Board** (tested with ESP32 DevKit V1)
- **BH1750 Light Sensor** (optional, I2C interface)
- **BME680 Environmental Sensor** (optional, I2C interface) 
- **Other Sensor Modules** (customize based on your needs)
- **Power Supply**: 5V USB or 3.3V-5V power source
- **Pull-up resistors**: 4.7kΩ for I2C sensors (usually built-in on sensor modules)

## Default Pin Configuration (Customizable)
| Function      | ESP32 Pin     | Purpose                           |
|---------------|---------------|-----------------------------------|
| I2C SCL       | GPIO22        | I2C Clock (if using I2C sensors) |
| I2C SDA       | GPIO21        | I2C Data (if using I2C sensors)  |
| Status LED    | GPIO2         | System status and MQTT heartbeat |
| Boot Button   | GPIO0         | Enter configuration mode          |

**Note**: Pin assignments can be changed in the source code based on your sensor requirements.

## MQTT Data Structure

The ESP32 publishes sensor data in JSON format to the configured MQTT topic. The message includes both system metrics and data from all enabled sensors in a single combined message.

### Combined Data Message Format
```json
{
  // ESP32 processor metrics
  "processor": {
    "WiFiRSSI": 75,                    // WiFi signal strength (% - 0-100%)
    "IPAddress": "192.168.1.150",      // Current IP address
    "CPUTemperature": 32.0,            // ESP32 CPU temperature (°C - placeholder)
    "SoftwareVersion": "1.3.0",        // Firmware version
    "ChipID": "A1B2C3D4E5F6",         // ESP32 unique chip identifier
    "WDTRestartCount": 2               // Watchdog restart counter
  },
  
  // All enabled sensor data
  "sensors": {
    "bh1750": {                        // BH1750 light sensor (if enabled)
      "name": "BH1750 Light Sensor",
      "valid": true,
      "timestamp": 12345,
      "lux": 1250.5
    },
    "bme680": {                        // BME680 environmental sensor (if enabled)
      "name": "BME680 Environmental",
      "valid": true,
      "timestamp": 12345,
      "temperature": 25.2,
      "humidity": 65.5,
      "pressure": 1013.25,
      "gas_resistance": 0
    }
    // Additional sensors will be added here as they are enabled
  }
}
```

### Data Fields Description

#### Processor Metrics (System Information)
- **WiFiRSSI**: WiFi signal strength as percentage (0-100%)
- **IPAddress**: Current IP address assigned by DHCP
- **CPUTemperature**: ESP32 internal temperature (placeholder implementation)
- **SoftwareVersion**: Firmware version (auto-generated from CMakeLists.txt VERSION field)
- **ChipID**: Unique ESP32 chip identifier (MAC-based)
- **WDTRestartCount**: Number of watchdog-triggered restarts (reliability metric)

#### Sensor Data (Configurable)
Each enabled sensor appears as a separate object within the "sensors" section:
- **name**: Human-readable sensor name
- **valid**: Boolean indicating successful sensor reading
- **timestamp**: Timestamp of the reading in milliseconds
- **sensor-specific fields**: Data fields specific to each sensor type

### Data Publishing Behavior
- **Successful Reading**: MQTT message published with current data from all enabled sensors
- **Failed Reading**: No MQTT message published (prevents invalid data)
- **Partial Success**: Message published with data from successfully read sensors only
- **Watchdog Protection**: ESP32 resets if no successful MQTT publish within timeout period
## Configuration and Setup

### Initial Setup
1. **Hardware Connection**: Wire your sensor(s) to ESP32 according to your sensor's requirements
2. **Power On**: Connect ESP32 to power source via USB
3. **Configuration Mode**: 
   - Hold the boot button (GPIO0) during power-on for 10 seconds
   - ESP32 will create a WiFi access point named "ESP32-CONFIG-XXXX"
4. **Web Configuration**:
   - Connect to the ESP32 access point
   - Open any website in your browser (captive portal will redirect)
   - Configure WiFi credentials and MQTT broker settings
   - Set device name and sampling interval
5. **Normal Operation**: ESP32 will restart and begin collecting/publishing data

### Configuration Parameters
- **WiFi SSID/Password**: Network credentials for internet connectivity
- **MQTT Broker URL**: MQTT server address (e.g., mqtt://192.168.1.100)
- **Data Topic**: MQTT topic for sensor data publishing
- **Sample Interval**: Time between sensor readings (milliseconds)

### Web Interface Features
- **System Information**: View current configuration and system status
- **OTA Updates**: Upload new firmware or file system images
- **Configuration Export**: Download current settings as JSON
- **Reset Options**: Factory reset or watchdog counter reset

## Applications and Use Cases
This configurable sensor system can be adapted for various IoT monitoring applications:
- **Environmental Monitoring**: Temperature, humidity, air quality sensors
- **Smart Home**: Light, motion, door/window sensors  
- **Industrial IoT**: Pressure, vibration, current sensors
- **Agriculture**: Soil moisture, pH, light sensors
- **Energy Management**: Power consumption, solar panel monitoring
- **Security Systems**: Motion detection, door/window status
- **Weather Stations**: Temperature, humidity, pressure, rainfall
- **Building Automation**: HVAC monitoring and control

## MQTT Integration Examples

### Home Assistant Integration
```yaml
# configuration.yaml - Updated for combined sensor data format
sensor:
  # BH1750 Light Sensor
  - platform: mqtt
    name: "ESP32 Light Level"
    state_topic: "sensors/data"
    value_template: "{{ value_json.sensors.bh1750.lux if value_json.sensors.bh1750 else 'unavailable' }}"
    unit_of_measurement: "lx"
    availability_template: "{{ 'online' if value_json.sensors.bh1750.valid else 'offline' }}"
    
  # BME680 Environmental Sensor
  - platform: mqtt
    name: "ESP32 Temperature"
    state_topic: "sensors/data"
    value_template: "{{ value_json.sensors.bme680.temperature if value_json.sensors.bme680 else 'unavailable' }}"
    unit_of_measurement: "°C"
    availability_template: "{{ 'online' if value_json.sensors.bme680.valid else 'offline' }}"
    
  - platform: mqtt
    name: "ESP32 Humidity"
    state_topic: "sensors/data"
    value_template: "{{ value_json.sensors.bme680.humidity if value_json.sensors.bme680 else 'unavailable' }}"
    unit_of_measurement: "%"
    availability_template: "{{ 'online' if value_json.sensors.bme680.valid else 'offline' }}"
    
  - platform: mqtt
    name: "ESP32 Pressure"
    state_topic: "sensors/data"
    value_template: "{{ value_json.sensors.bme680.pressure if value_json.sensors.bme680 else 'unavailable' }}"
    unit_of_measurement: "hPa"
    availability_template: "{{ 'online' if value_json.sensors.bme680.valid else 'offline' }}"
    
  # System Information
  - platform: mqtt
    name: "ESP32 WiFi Signal"
    state_topic: "sensors/data"
    value_template: "{{ value_json.processor.WiFiRSSI }}"
    unit_of_measurement: "%"
    
  - platform: mqtt
    name: "ESP32 IP Address"
    state_topic: "sensors/data"
    value_template: "{{ value_json.processor.IPAddress }}"
    
binary_sensor:
  # Overall sensor status
  - platform: mqtt
    name: "ESP32 Online"
    state_topic: "sensors/data"
    value_template: "{{ 'ON' if value_json.processor else 'OFF' }}"
    payload_on: "ON"
    payload_off: "OFF"
```

### InfluxDB Line Protocol
```influxdb
# Combined sensor data - parse JSON and create multiple measurements
sensor_data,device=ESP32-Sensor,sensor=bh1750 lux=1250.5,valid=1 1642234567890000000
sensor_data,device=ESP32-Sensor,sensor=bme680 temperature=25.2,humidity=65.5,pressure=1013.25,gas_resistance=0,valid=1 1642234567890000000
system_data,device=ESP32-Sensor wifi_rssi=75,cpu_temp=32.0,wdt_restarts=2 1642234567890000000
```

### Node-RED Flow Example
```json
[
  {
    "id": "mqtt_in",
    "type": "mqtt in",
    "topic": "sensors/data",
    "broker": "mqtt_broker"
  },
  {
    "id": "json_parse",
    "type": "json"
  },
  {
    "id": "sensor_switch",
    "type": "switch",
    "property": "payload.data.sensor_value_1",
    "rules": [
      {"t": "lt", "v": "threshold", "vt": "num"},
      {"t": "gte", "v": "threshold", "vt": "num"}
    ]
  }
]
```

## Getting Started with Customization

To adapt this template for your specific sensor:

1. **Update Sensor Configuration**: 
   - Modify the `#define` statements for your sensor's I2C address and commands
   - Change pin assignments if needed

2. **Customize Data Structure**:
   - Edit the `sensor_data_t` struct to match your sensor's data fields
   - Update variable names and data types as needed

3. **Implement Sensor Functions**:
   - Replace `sensor_init()` with your sensor's initialization sequence
   - Modify `read_sensor_value()` to read your sensor's data format
   - Update `read_sensor_data()` to populate your data structure

4. **Update MQTT Format**:
   - Customize the JSON output in `publish_sensor_data_mqtt()`
   - Change field names and add/remove data fields as needed

5. **Update Documentation**:
   - Modify this README for your specific sensor and use case
   - Update the MQTT data structure documentation

## Troubleshooting

### Common Issues

#### Sensor Not Reading
- **Check Wiring**: Verify sensor connections according to your sensor's datasheet
- **Communication Protocol**: Ensure correct I2C address, SPI settings, or GPIO configuration
- **Power Supply**: Verify sensor voltage requirements (3.3V vs 5V)
- **Pull-up Resistors**: Add 4.7kΩ pull-ups for I2C if not already present
- **Pull-up Resistors**: Most BH1750 modules include built-in pull-ups

#### WiFi Connection Issues
- **Signal Strength**: Check WiFi signal strength in system info
- **Credentials**: Verify SSID and password in configuration
- **Network**: Ensure 2.4GHz WiFi (ESP32 doesn't support 5GHz)
- **Firewall**: Check if network blocks MQTT traffic

#### MQTT Publishing Problems
- **Broker Connection**: Verify MQTT broker URL and port
- **Network Connectivity**: Ensure ESP32 can reach MQTT broker
- **Topic Permissions**: Check if broker requires authentication
- **QoS Settings**: Verify MQTT quality of service configuration

**Common MQTT Connection Errors:**
```
E (xxxxx) esp-tls: [sock=xx] select() timeout
E (xxxxx) transport_base: Failed to open a new connection
E (xxxxx) mqtt_client: Error transport connect
```
**Solutions:**
- **Check MQTT Broker URL**: Ensure format is `mqtt://IP_ADDRESS` or `mqtt://IP_ADDRESS:PORT`
  - ✅ `mqtt://192.168.1.100` (uses default port 1883)
  - ✅ `mqtt://192.168.1.100:1883` (explicit port)
  - ❌ `192.168.1.100` (missing mqtt:// protocol)
- **Network Reachability**: Ping MQTT broker from same network as ESP32
- **Firewall Rules**: Check if port 1883 is blocked by router/firewall
- **Broker Status**: Verify MQTT broker service is running and accepting connections
- **Authentication**: If broker requires login, ensure credentials are configured
- **URL Format**: Use IP address instead of hostname if DNS issues suspected

#### Web Interface Not Accessible
- **Configuration Mode**: Hold boot button during power-on for 10 seconds
- **AP Mode**: Look for "ESP32-CONFIG-XXXX" WiFi network
- **Browser Issues**: Try different browser or clear cache
- **Firewall**: Disable firewall/antivirus temporarily

### Diagnostic Information
- **Serial Monitor**: Connect via USB for detailed log output
- **System Info**: Check `/sysinfo.json` endpoint for system status
- **LED Indicator**: GPIO2 LED blinks on successful MQTT publish
- **Watchdog Counter**: Monitor restart count for stability issues

### Factory Reset
1. Hold boot button during power-on for 10 seconds
2. Connect to ESP32-CONFIG-XXXX WiFi network
3. Access web interface
4. Use "Factory Reset" option to clear all settings

## Technical Specifications

### Power Consumption
- **ESP32**: ~80mA active, ~10µA deep sleep
- **BH1750**: ~120µA active, <1µA standby
- **Total System**: ~85-100mA during normal operation

### Performance
- **Sensor Resolution**: 16-bit (65,536 levels)
- **Measurement Time**: ~120ms (high resolution mode)
- **I2C Speed**: 100kHz (configurable up to 400kHz)
- **MQTT Publish Rate**: Configurable (default: 5 seconds)
- **WiFi Reconnection**: Automatic with exponential backoff

### Memory Usage
- **Flash**: ~1.5MB firmware, ~500KB file system
- **RAM**: ~200KB free heap during operation
- **NVS**: <4KB for configuration storage

## Development and Customization

### Build Environment
- **Framework**: ESP-IDF v4.4 or later
- **Platform**: PlatformIO or Arduino IDE
- **Compiler**: Xtensa GCC cross-compiler
- **Dependencies**: ESP-IDF components (WiFi, MQTT, NVS, I2C)

### Version Management

The project implements **automatic version management** with a single source of truth approach to eliminate version synchronization issues and ensure consistency across all project components.

#### Architecture Overview
```
CMakeLists.txt (VERSION "X.Y.Z") 
    ↓ (CMake configure_file)
include/project_version.h.in (template)
    ↓ (Auto-generated during build)
include/project_version.h (header with constants)
    ↓ (Included by source code)
All project components use same version
```

#### Key Files and Roles

**Source of Truth:**
- **`CMakeLists.txt`**: Contains `project(ESP32-MQTT-TEMPLATE VERSION "X.Y.Z")` 
  - **THIS IS THE ONLY FILE YOU SHOULD EDIT FOR VERSION CHANGES**
  - Format: `project(ESP32-MQTT-TEMPLATE VERSION "1.4.0")`
  - Supports semantic versioning (MAJOR.MINOR.PATCH)

**Template System:**
- **`include/project_version.h.in`**: CMake template file
  - Contains `#define PROJECT_VERSION "@PROJECT_VERSION@"`
  - Do not edit manually - CMake processes this file
  - Template placeholders are replaced during build

**Generated Files (DO NOT EDIT MANUALLY):**
- **`include/project_version.h`**: Auto-generated header
  - Created by CMake during build process
  - Contains actual version constants
  - Included by C/C++ source files
  - Regenerated on every build if CMakeLists.txt changes

#### Automated Integration Points

The version system automatically integrates with all project components:

**1. Firmware Boot Logs:**
```
I (1234) ESP32-MQTT-TEMPLATE: Software Version: 1.4.0
I (1235) ESP32-MQTT-TEMPLATE: Build Date: Jul 23 2025
```

**2. MQTT JSON Messages:**
```json
{
  "processor": {
    "SoftwareVersion": "1.4.0",
    "BuildDate": "Jul 23 2025",
    ...
  },
  "data": { ... }
}
```

**3. Web Interface:**
- System information page displays current version
- About page shows version and build information
- OTA update pages reference current version

**4. System API Endpoints:**
```json
GET /sysinfo.json
{
  "firmware_version": "1.4.0",
  "build_date": "Jul 23 2025",
  ...
}
```

#### How to Update Version

**Step 1: Edit CMakeLists.txt**
```cmake
# Change this line only:
project(ESP32-MQTT-TEMPLATE VERSION "1.5.0")
```

**Step 2: Build Project**
```bash
pio run
# or
cmake --build build
```

**Step 3: Automatic Propagation**
- CMake detects version change
- Regenerates `include/project_version.h`
- All source files pick up new version
- Build output shows new version
- Runtime logs display new version

#### Version Verification

**Check Current Version:**
- **Build logs**: Version appears in compilation output
- **Serial monitor**: Boot sequence shows version
- **Web interface**: System information page
- **MQTT messages**: Published in processor metrics
- **Source code**: `PROJECT_VERSION` constant

**Build-Time Verification:**
```bash
# Check generated header
cat include/project_version.h

# Verify CMake configuration
cmake --build build --verbose
```

#### Benefits of This System

✅ **Single Source of Truth**: Only one place to update versions  
✅ **Automatic Synchronization**: No manual file editing required  
✅ **Build-Time Generation**: Version always matches build  
✅ **Runtime Availability**: Version accessible in all code  
✅ **Zero Maintenance**: No version file management needed  
✅ **Error Prevention**: Eliminates version mismatch issues  

#### Troubleshooting Version Issues

**Problem: Old version still showing after update**
```bash
# Solution: Clean and rebuild
pio run -t clean
pio run
```

**Problem: Version header not found**
```bash
# Solution: Ensure template file exists
ls include/project_version.h.in
```

**Problem: CMake not detecting version change**
```bash
# Solution: Force CMake reconfiguration
rm -rf .pio/build
pio run
```

#### Version History

The project maintains semantic versioning (MAJOR.MINOR.PATCH):
- **MAJOR**: Breaking changes or major feature additions
- **MINOR**: New features, backward compatible
- **PATCH**: Bug fixes, minor improvements

Example progression:
- `1.0.0` - Initial BH1750 implementation
- `1.1.0` - Added web configuration interface
- `1.1.1` - Fixed MQTT connection timeout
- `1.2.0` - Added OTA update capability
- `1.3.0` - Enhanced sensor error handling
- `1.4.0` - Implemented automatic version management

### Adding Additional Sensors
The codebase is designed for easy expansion:

```c
// Add new sensor to data structure
typedef struct {
    float sunlight;     // BH1750 light sensor
    float temperature;  // Additional temperature sensor
    float humidity;     // Additional humidity sensor
    bool sensors_ok;    // Combined sensor status
} input_data_t;
```

### Customizing MQTT Topics
```c
// Multiple topic publishing
esp_mqtt_client_publish(client, "sensors/light/lux", lux_string, 0, 1, 0);
esp_mqtt_client_publish(client, "sensors/light/status", status_string, 0, 1, 0);
esp_mqtt_client_publish(client, "sensors/system/wifi", wifi_string, 0, 1, 0);
```

## License and Attribution
This project is based on the ESP32MQTTBase template and has been adapted for BH1750 light sensor applications. 

- **Original Template**: ESP32 MQTT Data Template
- **Sensor Integration**: BH1750 I2C Light Sensor
- **Current Version**: Specialized for ambient light monitoring applications

For technical support or contributions, please refer to the project documentation and issue tracker.
