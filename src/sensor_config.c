/*
 * ESP32 MQTT Configurable Sensor System Implementation
 */

#include "sensor_config.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "SENSOR_CONFIG";

// NVS namespace for sensor configuration
#define SENSOR_NVS_NAMESPACE "sensor_cfg"

// Global sensor configuration array
static sensor_config_t sensor_configs[SENSOR_TYPE_MAX_COUNT];
static sensor_data_t sensor_data_cache[SENSOR_TYPE_MAX_COUNT];
static bool config_initialized = false;

// Default sensor configurations
static const sensor_config_t default_configs[SENSOR_TYPE_MAX_COUNT] = {
    // BH1750 Light Sensor
    {
        .type = SENSOR_TYPE_BH1750,
        .enabled = false,
        .name = "BH1750 Light Sensor",
        .mqtt_topic = "sensors/light/bh1750",
        .sample_interval_ms = 0  // Use global interval
    },
    // BME680 Environmental Sensor
    {
        .type = SENSOR_TYPE_BME680,
        .enabled = false,
        .name = "BME680 Environmental",
        .mqtt_topic = "sensors/environment/bme680",
        .sample_interval_ms = 0
    },
    // DHT22 Temperature/Humidity
    {
        .type = SENSOR_TYPE_DHT22,
        .enabled = false,
        .name = "DHT22 Temp/Humidity",
        .mqtt_topic = "sensors/climate/dht22",
        .sample_interval_ms = 0
    },
    // DS18B20 Temperature
    {
        .type = SENSOR_TYPE_DS18B20,
        .enabled = false,
        .name = "DS18B20 Temperature",
        .mqtt_topic = "sensors/temperature/ds18b20",
        .sample_interval_ms = 0
    },
    // MQ135 Air Quality
    {
        .type = SENSOR_TYPE_MQ135,
        .enabled = false,
        .name = "MQ135 Air Quality",
        .mqtt_topic = "sensors/airquality/mq135",
        .sample_interval_ms = 0
    },
    // BME280
    {
        .type = SENSOR_TYPE_BME280,
        .enabled = false,
        .name = "BME280 Climate",
        .mqtt_topic = "sensors/climate/bme280",
        .sample_interval_ms = 0
    },
    // SHT30
    {
        .type = SENSOR_TYPE_SHT30,
        .enabled = false,
        .name = "SHT30 Temp/Humidity",
        .mqtt_topic = "sensors/climate/sht30",
        .sample_interval_ms = 0
    },
    // TSL2561
    {
        .type = SENSOR_TYPE_TSL2561,
        .enabled = false,
        .name = "TSL2561 Light Sensor",
        .mqtt_topic = "sensors/light/tsl2561",
        .sample_interval_ms = 0
    }
};

esp_err_t sensor_config_init(void) {
    if (config_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing sensor configuration system");
    
    // Initialize with default configurations
    memcpy(sensor_configs, default_configs, sizeof(default_configs));
    
    // Clear sensor data cache
    memset(sensor_data_cache, 0, sizeof(sensor_data_cache));
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Load configuration from NVS
    sensor_config_load_from_nvs();
    
    config_initialized = true;
    ESP_LOGI(TAG, "Sensor configuration system initialized");
    
    return ESP_OK;
}

esp_err_t sensor_config_load_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(SENSOR_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS namespace, using defaults: %s", esp_err_to_name(err));
        return err;
    }
    
    for (int i = 0; i < SENSOR_TYPE_MAX_COUNT; i++) {
        char key_enabled[32];
        char key_name[32];
        char key_topic[32];
        char key_interval[32];
        
        snprintf(key_enabled, sizeof(key_enabled), "s%d_enabled", i);
        snprintf(key_name, sizeof(key_name), "s%d_name", i);
        snprintf(key_topic, sizeof(key_topic), "s%d_topic", i);
        snprintf(key_interval, sizeof(key_interval), "s%d_interval", i);
        
        // Load enabled status
        uint8_t enabled = 0;
        size_t required_size = sizeof(enabled);
        err = nvs_get_blob(nvs_handle, key_enabled, &enabled, &required_size);
        if (err == ESP_OK) {
            sensor_configs[i].enabled = (enabled != 0);
        }
        
        // Load sensor name
        size_t name_size = sizeof(sensor_configs[i].name);
        err = nvs_get_str(nvs_handle, key_name, sensor_configs[i].name, &name_size);
        if (err != ESP_OK) {
            // Keep default name if not found
            ESP_LOGD(TAG, "Using default name for sensor %d", i);
        }
        
        // Load MQTT topic
        size_t topic_size = sizeof(sensor_configs[i].mqtt_topic);
        err = nvs_get_str(nvs_handle, key_topic, sensor_configs[i].mqtt_topic, &topic_size);
        if (err != ESP_OK) {
            // Keep default topic if not found
            ESP_LOGD(TAG, "Using default topic for sensor %d", i);
        }
        
        // Load sample interval
        uint32_t interval;
        required_size = sizeof(interval);
        err = nvs_get_blob(nvs_handle, key_interval, &interval, &required_size);
        if (err == ESP_OK) {
            sensor_configs[i].sample_interval_ms = interval;
        }
        
        ESP_LOGI(TAG, "Loaded sensor %d: enabled=%d, name='%s', topic='%s', interval=%lu", 
                 i, sensor_configs[i].enabled, sensor_configs[i].name, sensor_configs[i].mqtt_topic, 
                 sensor_configs[i].sample_interval_ms);
    }
    
    nvs_close(nvs_handle);
    return ESP_OK;
}

esp_err_t sensor_config_save_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(SENSOR_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace for writing: %s", esp_err_to_name(err));
        return err;
    }
    
    for (int i = 0; i < SENSOR_TYPE_MAX_COUNT; i++) {
        char key_enabled[32];
        char key_name[32];
        char key_topic[32];
        char key_interval[32];
        
        snprintf(key_enabled, sizeof(key_enabled), "s%d_enabled", i);
        snprintf(key_name, sizeof(key_name), "s%d_name", i);
        snprintf(key_topic, sizeof(key_topic), "s%d_topic", i);
        snprintf(key_interval, sizeof(key_interval), "s%d_interval", i);
        
        // Save enabled status
        uint8_t enabled = sensor_configs[i].enabled ? 1 : 0;
        err = nvs_set_blob(nvs_handle, key_enabled, &enabled, sizeof(enabled));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save enabled status for sensor %d", i);
        }
        
        // Save sensor name
        err = nvs_set_str(nvs_handle, key_name, sensor_configs[i].name);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save name for sensor %d", i);
        }
        
        // Save MQTT topic
        err = nvs_set_str(nvs_handle, key_topic, sensor_configs[i].mqtt_topic);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save topic for sensor %d", i);
        }
        
        // Save sample interval
        err = nvs_set_blob(nvs_handle, key_interval, &sensor_configs[i].sample_interval_ms, sizeof(sensor_configs[i].sample_interval_ms));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save interval for sensor %d", i);
        }
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sensor configuration saved to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to commit sensor configuration to NVS");
    }
    
    return err;
}

esp_err_t sensor_config_set_enabled(sensor_type_t type, bool enabled) {
    if (type >= SENSOR_TYPE_MAX_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    sensor_configs[type].enabled = enabled;
    ESP_LOGI(TAG, "Sensor %s %s", sensor_configs[type].name, enabled ? "enabled" : "disabled");
    
    return ESP_OK;
}

bool sensor_config_is_enabled(sensor_type_t type) {
    if (type >= SENSOR_TYPE_MAX_COUNT) {
        return false;
    }
    
    return sensor_configs[type].enabled;
}

sensor_config_t* sensor_config_get(sensor_type_t type) {
    if (type >= SENSOR_TYPE_MAX_COUNT) {
        return NULL;
    }
    
    return &sensor_configs[type];
}

const char* sensor_config_get_name(sensor_type_t type) {
    if (type >= SENSOR_TYPE_MAX_COUNT) {
        return "Unknown";
    }
    
    return sensor_configs[type].name;
}

const char* sensor_config_get_mqtt_topic(sensor_type_t type) {
    if (type >= SENSOR_TYPE_MAX_COUNT) {
        return "sensors/unknown";
    }
    
    return sensor_configs[type].mqtt_topic;
}

esp_err_t sensor_config_set_mqtt_topic(sensor_type_t type, const char* topic) {
    if (type >= SENSOR_TYPE_MAX_COUNT || !topic) {
        return ESP_ERR_INVALID_ARG;
    }
    
    strncpy(sensor_configs[type].mqtt_topic, topic, sizeof(sensor_configs[type].mqtt_topic) - 1);
    sensor_configs[type].mqtt_topic[sizeof(sensor_configs[type].mqtt_topic) - 1] = '\0';
    
    return ESP_OK;
}

esp_err_t sensor_config_set_name(sensor_type_t type, const char* name) {
    if (type >= SENSOR_TYPE_MAX_COUNT || !name) {
        return ESP_ERR_INVALID_ARG;
    }
    
    strncpy(sensor_configs[type].name, name, sizeof(sensor_configs[type].name) - 1);
    sensor_configs[type].name[sizeof(sensor_configs[type].name) - 1] = '\0';
    
    return ESP_OK;
}

sensor_data_t* sensor_data_get(sensor_type_t type) {
    if (type >= SENSOR_TYPE_MAX_COUNT) {
        return NULL;
    }
    
    return &sensor_data_cache[type];
}

bool sensor_data_is_valid(sensor_type_t type) {
    if (type >= SENSOR_TYPE_MAX_COUNT) {
        return false;
    }
    
    return sensor_data_cache[type].valid;
}

esp_err_t sensor_read_all_enabled(void) {
    ESP_LOGI(TAG, "Reading all enabled sensors");
    
    bool any_success = false;
    
    for (int i = 0; i < SENSOR_TYPE_MAX_COUNT; i++) {
        if (!sensor_configs[i].enabled) {
            continue;
        }
        
        sensor_data_t data;
        esp_err_t result = ESP_FAIL;
        
        switch (i) {
            case SENSOR_TYPE_BH1750:
                result = sensor_bh1750_read(&data);
                break;
            case SENSOR_TYPE_BME680:
                result = sensor_bme680_read(&data);
                break;
            default:
                ESP_LOGW(TAG, "Sensor type %d reading not implemented", i);
                continue;
        }
        
        if (result == ESP_OK && data.valid) {
            // Store the data in cache
            sensor_data_cache[i] = data;
            any_success = true;
            ESP_LOGI(TAG, "Successfully read %s sensor", sensor_configs[i].name);
        } else {
            ESP_LOGE(TAG, "Failed to read %s sensor", sensor_configs[i].name);
            sensor_data_cache[i].valid = false;
        }
    }
    
    return any_success ? ESP_OK : ESP_FAIL;
}

char* sensor_data_to_json(sensor_type_t type, const sensor_data_t* data) {
    if (!data || type >= SENSOR_TYPE_MAX_COUNT) {
        return NULL;
    }
    
    cJSON *json = cJSON_CreateObject();
    cJSON *sensor_info = cJSON_CreateObject();
    cJSON *sensor_data_obj = cJSON_CreateObject();
    
    // Add sensor metadata
    cJSON_AddStringToObject(sensor_info, "name", sensor_config_get_name(type));
    cJSON_AddStringToObject(sensor_info, "type", sensor_config_get_name(type));
    cJSON_AddBoolToObject(sensor_info, "valid", data->valid);
    
    // Add sensor-specific data based on type
    switch (type) {
        case SENSOR_TYPE_BH1750:
            if (data->valid) {
                cJSON_AddNumberToObject(sensor_data_obj, "lux", data->data.bh1750.lux);
            }
            break;
            
        case SENSOR_TYPE_BME680:
            if (data->valid) {
                cJSON_AddNumberToObject(sensor_data_obj, "temperature", data->data.bme680.temperature);
                cJSON_AddNumberToObject(sensor_data_obj, "humidity", data->data.bme680.humidity);
                cJSON_AddNumberToObject(sensor_data_obj, "pressure", data->data.bme680.pressure);
                cJSON_AddNumberToObject(sensor_data_obj, "gas_resistance", data->data.bme680.gas_resistance);
            }
            break;
            
        case SENSOR_TYPE_DHT22:
            if (data->valid) {
                cJSON_AddNumberToObject(sensor_data_obj, "temperature", data->data.dht22.temperature);
                cJSON_AddNumberToObject(sensor_data_obj, "humidity", data->data.dht22.humidity);
            }
            break;
            
        case SENSOR_TYPE_DS18B20:
            if (data->valid) {
                cJSON_AddNumberToObject(sensor_data_obj, "temperature", data->data.ds18b20.temperature);
            }
            break;
            
        case SENSOR_TYPE_MQ135:
            if (data->valid) {
                cJSON_AddNumberToObject(sensor_data_obj, "ppm", data->data.mq135.ppm);
                cJSON_AddNumberToObject(sensor_data_obj, "raw_adc", data->data.mq135.raw_adc);
            }
            break;
            
        case SENSOR_TYPE_BME280:
            if (data->valid) {
                cJSON_AddNumberToObject(sensor_data_obj, "temperature", data->data.bme280.temperature);
                cJSON_AddNumberToObject(sensor_data_obj, "humidity", data->data.bme280.humidity);
                cJSON_AddNumberToObject(sensor_data_obj, "pressure", data->data.bme280.pressure);
            }
            break;
            
        case SENSOR_TYPE_SHT30:
            if (data->valid) {
                cJSON_AddNumberToObject(sensor_data_obj, "temperature", data->data.sht30.temperature);
                cJSON_AddNumberToObject(sensor_data_obj, "humidity", data->data.sht30.humidity);
            }
            break;
            
        case SENSOR_TYPE_TSL2561:
            if (data->valid) {
                cJSON_AddNumberToObject(sensor_data_obj, "lux", data->data.tsl2561.lux);
                cJSON_AddNumberToObject(sensor_data_obj, "visible", data->data.tsl2561.visible);
                cJSON_AddNumberToObject(sensor_data_obj, "infrared", data->data.tsl2561.infrared);
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown sensor type: %d", type);
            break;
    }
    
    // Combine into final JSON
    cJSON_AddItemToObject(json, "sensor", sensor_info);
    cJSON_AddItemToObject(json, "data", sensor_data_obj);
    
    char *json_string = cJSON_Print(json);
    cJSON_Delete(json);
    
    return json_string;
}
