# ESP32 Configurable Sensor System

This document explains how to use and extend the configurable sensor system that has been added to the ESP32 MQTT Weather project.

## Overview

The configurable sensor system allows you to:
- Enable/disable individual sensors at runtime through the web interface
- Configure individual MQTT topics for each sensor
- Add new sensors without modifying the core application logic
- Store sensor configuration persistently in NVS

## Currently Supported Sensors

### BH1750 Light Sensor
- **Status**: Fully implemented
- **Interface**: I2C (address 0x23)
- **Data**: Light intensity in lux
- **Default Topic**: `sensors/light/bh1750`

### BME680 Environmental Sensor  
- **Status**: Stub implementation (placeholder)
- **Interface**: I2C/SPI
- **Data**: Temperature, humidity, pressure, gas resistance
- **Default Topic**: `sensors/environment/bme680`
- **Note**: Implementation needs to be completed

### Future Sensors (Framework Ready)
- DHT22 Temperature/Humidity
- DS18B20 Temperature  
- BME280 Climate
- SHT30 Temperature/Humidity
- TSL2561 Light Sensor
- MQ135 Air Quality

## Configuration Interface

### Web Interface
1. Connect to the ESP32's configuration portal
2. Navigate to the "Sensor Configuration" section
3. Enable/disable sensors using checkboxes
4. Configure MQTT topics for each sensor
5. Click "Save & Reboot" to apply changes

### Configuration Storage
- Sensor configurations are stored in NVS (non-volatile storage)
- Settings persist across reboots
- Factory reset will restore default sensor configurations

## MQTT Publishing

### Individual Sensor Topics
Each enabled sensor publishes to its own configurable MQTT topic with the format:
```json
{
  "sensor": {
    "name": "BH1750 Light Sensor",
    "type": "BH1750 Light Sensor", 
    "valid": true,
    "timestamp": 12345
  },
  "data": {
    "lux": 1250.5
  }
}
```

### Topic Configuration
- Default topics follow the pattern: `sensors/{category}/{sensor_name}`
- Topics can be customized through the web interface
- Maximum topic length: 63 characters

## Adding New Sensors

### 1. Define Sensor Type
Add your sensor to the `sensor_type_t` enum in `sensor_config.h`:
```c
typedef enum {
    SENSOR_TYPE_BH1750 = 0,
    SENSOR_TYPE_BME680 = 1,
    SENSOR_TYPE_YOUR_SENSOR = 2,  // Add here
    // ...
    SENSOR_TYPE_MAX_COUNT
} sensor_type_t;
```

### 2. Add Data Structure
Add your sensor's data fields to the union in `sensor_config.h`:
```c
union {
    // ... existing sensors ...
    struct {                     // Your sensor
        float your_value;        // Your sensor's data
        uint16_t raw_reading;    // Additional fields
    } your_sensor;
} data;
```

### 3. Update Default Configuration
Add default configuration in `sensor_config.c`:
```c
static const sensor_config_t default_configs[SENSOR_TYPE_MAX_COUNT] = {
    // ... existing sensors ...
    {
        .type = SENSOR_TYPE_YOUR_SENSOR,
        .enabled = false,
        .name = "Your Sensor Name",
        .mqtt_topic = "sensors/category/your_sensor",
        .sample_interval_ms = 0
    },
};
```

### 4. Implement Sensor Driver
Create `sensor_your_sensor.c` with:
```c
esp_err_t sensor_your_sensor_init(void) {
    // Initialize your sensor
    return ESP_OK;
}

esp_err_t sensor_your_sensor_read(sensor_data_t* data) {
    // Read from your sensor
    data->type = SENSOR_TYPE_YOUR_SENSOR;
    data->data.your_sensor.your_value = read_your_sensor();
    data->valid = true;
    data->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return ESP_OK;
}
```

### 5. Add to Initialization
In `main.c`, add initialization for your sensor:
```c
if (sensor_config_is_enabled(SENSOR_TYPE_YOUR_SENSOR)) {
    esp_err_t err = sensor_your_sensor_init();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Your sensor initialized successfully");
    }
}
```

### 6. Add to Reading Loop
In `sensor_config.c`, add your sensor to the reading cases:
```c
case SENSOR_TYPE_YOUR_SENSOR:
    result = sensor_your_sensor_read(&data);
    break;
```

### 7. Add JSON Serialization
In `sensor_config.c`, add JSON formatting:
```c
case SENSOR_TYPE_YOUR_SENSOR:
    if (data->valid) {
        cJSON_AddNumberToObject(sensor_data_obj, "your_value", data->data.your_sensor.your_value);
    }
    break;
```

### 8. Update Web Interface
In `parameters.html`, add configuration UI:
```html
<div style="border: 1px solid #e0e0e0; border-radius: 8px; padding: 1em; margin-bottom: 1em;">
    <label style="display: flex; align-items: center; margin-bottom: 0.5em;">
        <input type="checkbox" name="sensor_your_sensor_enabled" id="sensor_your_sensor_enabled" style="margin-right: 0.5em;">
        <strong>Your Sensor Name</strong>
    </label>
    <label style="margin-top: 0.5em;">MQTT Topic:
        <input type="text" name="sensor_your_sensor_topic" id="sensor_your_sensor_topic" value="sensors/category/your_sensor" maxlength="63">
    </label>
</div>
```

And add to the JavaScript configuration loading:
```javascript
if (cfg.sensors.your_sensor) {
    document.getElementById('sensor_your_sensor_enabled').checked = cfg.sensors.your_sensor.enabled || false;
    document.getElementById('sensor_your_sensor_topic').value = cfg.sensors.your_sensor.topic || 'sensors/category/your_sensor';
}
```

### 9. Update Parameter Handler
In `main.c`, add parameter processing:
```c
// Your Sensor
if (strstr(buf, "sensor_your_sensor_enabled=on")) {
    sensor_config_set_enabled(SENSOR_TYPE_YOUR_SENSOR, true);
} else {
    sensor_config_set_enabled(SENSOR_TYPE_YOUR_SENSOR, false);
}

char *your_sensor_topic_ptr = strstr(buf, "sensor_your_sensor_topic=");
if (your_sensor_topic_ptr) {
    char topic_in[64], topic_dec[64];
    sscanf(your_sensor_topic_ptr + 25, "%63[^&]", topic_in);  // Adjust offset
    url_decode(topic_dec, topic_in, sizeof(topic_dec));
    sensor_config_set_mqtt_topic(SENSOR_TYPE_YOUR_SENSOR, topic_dec);
}
```

## Hardware Connections

### BH1750 Light Sensor
```
ESP32    BH1750
-----    ------
3.3V  -> VCC
GND   -> GND  
GPIO21-> SDA
GPIO22-> SCL
GND   -> ADDR (for address 0x23)
```

### Adding I2C Sensors
The BH1750 implementation provides a shared I2C bus that other I2C sensors can use:
```c
i2c_master_bus_handle_t bus = bh1750_get_i2c_bus_handle();
// Use this bus handle for other I2C sensors
```

### Pin Configuration
Default I2C pins (can be changed in sensor_bh1750.c):
- **SDA**: GPIO21
- **SCL**: GPIO22
- **Pull-ups**: Enabled internally

## Building and Running

### Build the Project
```bash
idf.py build
```

### Flash the Device
```bash
idf.py flash monitor
```

### Configuration
1. Hold boot button during power-on for 10 seconds to enter config mode
2. Connect to "ESP32-CONFIG-XXXX" WiFi network
3. Configure WiFi, MQTT, and enable desired sensors
4. Device will reboot and begin reading enabled sensors

## Troubleshooting

### Sensor Not Reading
1. Check hardware connections
2. Verify sensor is enabled in configuration
3. Check I2C address (use `i2cdetect` if available)
4. Review ESP32 serial logs for error messages

### MQTT Not Publishing
1. Verify MQTT broker configuration
2. Check network connectivity
3. Ensure at least one sensor is enabled and working
4. Review MQTT topic configuration

### Configuration Not Saving
1. Check NVS partition availability
2. Verify web interface shows current settings
3. Use factory reset if needed

## Integration Examples

### Home Assistant
```yaml
sensor:
  - platform: mqtt
    name: "Light Level"
    state_topic: "sensors/light/bh1750"
    value_template: "{{ value_json.data.lux }}"
    unit_of_measurement: "lx"
    
  - platform: mqtt
    name: "Sensor Status"
    state_topic: "sensors/light/bh1750"
    value_template: "{{ value_json.sensor.valid }}"
```

### Node-RED
The individual sensor topics make it easy to create specific flows for each sensor type without parsing complex combined messages.

## Future Enhancements

- Individual sample intervals per sensor
- Sensor calibration parameters
- Sensor health monitoring
- Automatic sensor discovery
- Sleep mode support for battery operation
- OTA updates for sensor firmware
