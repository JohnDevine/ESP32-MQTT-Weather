/*
 * ESP32 MQTT Configurable Sensor System
 * 
 * This header file defines the configurable sensor system that allows
 * individual sensors to be enabled/disabled at runtime through the web
 * interface, with settings stored in NVS.
 */

#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// Shared I2C bus handle (initialized by BH1750, used by other sensors)
extern i2c_master_bus_handle_t i2c_bus_handle;

// Maximum number of sensors supported
#define MAX_SENSORS 10

// Sensor type enumeration
typedef enum {
    SENSOR_TYPE_BH1750 = 0,    // Light sensor
    SENSOR_TYPE_BME680 = 1,    // Environmental sensor (temp, humidity, pressure, gas)
    SENSOR_TYPE_DHT22 = 2,     // Temperature and humidity sensor
    SENSOR_TYPE_DS18B20 = 3,   // Temperature sensor
    SENSOR_TYPE_MQ135 = 4,     // Air quality sensor
    SENSOR_TYPE_BME280 = 5,    // Temperature, humidity, pressure sensor
    SENSOR_TYPE_SHT30 = 6,     // Temperature and humidity sensor
    SENSOR_TYPE_TSL2561 = 7,   // Light sensor
    SENSOR_TYPE_WIND_SPEED = 8, // Wind speed sensor (SN-3000-FSJT-N01, RS485)
    SENSOR_TYPE_MAX_COUNT      // Keep this last - indicates max sensor types
} sensor_type_t;

// Sensor configuration structure
typedef struct {
    sensor_type_t type;
    bool enabled;                    // Whether sensor is enabled
    char name[32];                   // Human-readable sensor name
    char mqtt_topic[64];             // MQTT topic for this sensor
    uint32_t sample_interval_ms;     // Individual sample interval (0 = use global)
    // Add more configuration parameters as needed
} sensor_config_t;

// Sensor data structure - generic container for any sensor data
typedef struct {
    sensor_type_t type;
    bool valid;                      // Whether the data is valid
    
    // Generic data fields - use based on sensor type
    union {
        struct {                     // BH1750 Light sensor
            float lux;
        } bh1750;
        
        struct {                     // BME680 Environmental sensor
            float temperature;       // Celsius
            float humidity;          // Relative humidity %
            float pressure;          // hPa
            float gas_resistance;    // Ohms
        } bme680;
        
        struct {                     // DHT22 Temperature/Humidity
            float temperature;       // Celsius
            float humidity;          // Relative humidity %
        } dht22;
        
        struct {                     // DS18B20 Temperature
            float temperature;       // Celsius
        } ds18b20;
        
        struct {                     // MQ135 Air Quality
            float ppm;               // Parts per million
            uint16_t raw_adc;        // Raw ADC reading
        } mq135;
        
        struct {                     // BME280
            float temperature;       // Celsius
            float humidity;          // Relative humidity %
            float pressure;          // hPa
        } bme280;
        
        struct {                     // SHT30
            float temperature;       // Celsius
            float humidity;          // Relative humidity %
        } sht30;
        
        struct {                     // TSL2561
            float lux;
            uint16_t visible;        // Visible light
            uint16_t infrared;       // Infrared light
        } tsl2561;
        
        struct {                     // Wind Speed Sensor (SN-3000-FSJT-N01)
            float wind_speed;        // Current instantaneous wind speed (m/s)
            float wind_speed_avg;    // Rolling average wind speed (m/s)
            float wind_gust;         // Rolling maximum gust speed (m/s)
            uint16_t raw_reading;    // Raw Modbus register value
        } wind_speed;
    } data;
} sensor_data_t;

// Function declarations
esp_err_t sensor_config_init(void);
esp_err_t sensor_config_load_from_nvs(void);
esp_err_t sensor_config_save_to_nvs(void);

// Sensor configuration management
esp_err_t sensor_config_set_enabled(sensor_type_t type, bool enabled);
bool sensor_config_is_enabled(sensor_type_t type);
sensor_config_t* sensor_config_get(sensor_type_t type);
const char* sensor_config_get_name(sensor_type_t type);
esp_err_t sensor_config_set_name(sensor_type_t type, const char* name);
const char* sensor_config_get_mqtt_topic(sensor_type_t type);
esp_err_t sensor_config_set_mqtt_topic(sensor_type_t type, const char* topic);

// Sensor data management
esp_err_t sensor_read_all_enabled(void);
sensor_data_t* sensor_data_get(sensor_type_t type);
bool sensor_data_is_valid(sensor_type_t type);

// Sensor-specific functions
esp_err_t sensor_bh1750_init(void);
esp_err_t sensor_bh1750_read(sensor_data_t* data);
bool bh1750_is_initialized(void);

esp_err_t sensor_bme680_init(void);
esp_err_t sensor_bme680_read(sensor_data_t* data);
bool bme680_is_initialized(void);

esp_err_t sensor_wind_speed_init(void);
esp_err_t sensor_wind_speed_read(sensor_data_t* data);
bool wind_speed_is_initialized(void);

// Wind speed configuration functions
esp_err_t sensor_wind_speed_set_config(uint32_t reading_interval_ms, uint32_t avg_period_ms, uint32_t gust_period_ms);
esp_err_t sensor_wind_speed_get_config(uint32_t* reading_interval_ms, uint32_t* avg_period_ms, uint32_t* gust_period_ms);

// I2C bus management (for sharing between sensors)
#include "driver/i2c_master.h"
i2c_master_bus_handle_t bh1750_get_i2c_bus_handle(void);

// JSON serialization for MQTT
char* sensor_data_to_json(sensor_type_t type, const sensor_data_t* data);

#endif // SENSOR_CONFIG_H
