/*
 * BH1750 Light Sensor Implementation
 * 
 * This module implements the BH1750 digital light sensor driver
 * with I2C communication for the ESP32 MQTT sensor system.
 */

#include "sensor_config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BH1750";

// BH1750 I2C Configuration
#define BH1750_I2C_ADDR_LOW     0x23    // ADDR pin connected to GND
#define BH1750_I2C_ADDR_HIGH    0x5C    // ADDR pin connected to VCC
#define BH1750_I2C_ADDR         BH1750_I2C_ADDR_LOW  // Default address

// BH1750 Commands
#define BH1750_POWER_DOWN       0x00    // Power down the sensor
#define BH1750_POWER_ON         0x01    // Power on the sensor
#define BH1750_RESET            0x07    // Reset data register
#define BH1750_CONT_HIGH_RES    0x10    // Continuous high resolution mode (1 lux resolution)
#define BH1750_CONT_HIGH_RES2   0x11    // Continuous high resolution mode 2 (0.5 lux resolution)
#define BH1750_CONT_LOW_RES     0x13    // Continuous low resolution mode (4 lux resolution)
#define BH1750_ONE_TIME_HIGH_RES 0x20   // One time high resolution mode
#define BH1750_ONE_TIME_HIGH_RES2 0x21  // One time high resolution mode 2
#define BH1750_ONE_TIME_LOW_RES 0x23    // One time low resolution mode

// Timing definitions
#define BH1750_CONVERSION_TIME_MS   180  // Maximum conversion time in milliseconds

// I2C Configuration
#define I2C_MASTER_SCL_IO       GPIO_NUM_22
#define I2C_MASTER_SDA_IO       GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ      100000
#define I2C_MASTER_TIMEOUT_MS   1000

// Global I2C handles (shared with other sensors)
i2c_master_bus_handle_t i2c_bus_handle = NULL;  // Make this global for other sensors
static i2c_master_dev_handle_t bh1750_dev_handle = NULL;
static bool bh1750_initialized = false;

// Function to initialize I2C bus (shared with other sensors)
static esp_err_t bh1750_init_i2c(void) {
    if (i2c_bus_handle != NULL) {
        // I2C bus already initialized
        return ESP_OK;
    }
    
    // Configure I2C master bus
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus creation failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master bus initialized successfully");
    return ESP_OK;
}

// Function to send command to BH1750
static esp_err_t bh1750_send_command(uint8_t command) {
    esp_err_t ret = i2c_master_transmit(bh1750_dev_handle, &command, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command 0x%02X: %s", command, esp_err_to_name(ret));
    }
    return ret;
}

// Function to read data from BH1750
static esp_err_t bh1750_read_data(uint16_t *raw_data) {
    uint8_t data[2];
    esp_err_t ret = i2c_master_receive(bh1750_dev_handle, data, 2, I2C_MASTER_TIMEOUT_MS);
    
    if (ret == ESP_OK) {
        *raw_data = (data[0] << 8) | data[1];
    } else {
        ESP_LOGE(TAG, "Failed to read data: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t sensor_bh1750_init(void) {
    if (bh1750_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing BH1750 light sensor");
    
    // Initialize I2C bus
    esp_err_t ret = bh1750_init_i2c();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure BH1750 device on the I2C bus
    i2c_device_config_t bh1750_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BH1750_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(i2c_bus_handle, &bh1750_cfg, &bh1750_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BH1750 device add failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Power on the sensor
    ret = bh1750_send_command(BH1750_POWER_ON);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for power on
    
    // Reset the sensor
    ret = bh1750_send_command(BH1750_RESET);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for reset
    
    // Note: We don't set continuous mode here as we use one-time measurements
    // in the read function for more responsive and accurate readings
    
    bh1750_initialized = true;
    ESP_LOGI(TAG, "BH1750 sensor initialized successfully (one-time measurement mode)");
    
    return ESP_OK;
}

esp_err_t sensor_bh1750_read(sensor_data_t* data) {
    if (!bh1750_initialized) {
        ESP_LOGE(TAG, "BH1750 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize data structure
    memset(data, 0, sizeof(sensor_data_t));
    data->type = SENSOR_TYPE_BH1750;
    data->valid = false;
    
    // Trigger a fresh one-time high resolution measurement
    esp_err_t ret = bh1750_send_command(BH1750_ONE_TIME_HIGH_RES);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger measurement");
        return ret;
    }
    
    // Wait for measurement to complete (180ms for high resolution)
    vTaskDelay(pdMS_TO_TICKS(BH1750_CONVERSION_TIME_MS));
    
    uint16_t raw_data;
    ret = bh1750_read_data(&raw_data);
    
    if (ret == ESP_OK) {
        // Convert raw data to lux
        // BH1750 formula: Lux = raw_data / 1.2 (for high resolution mode)
        float lux = (float)raw_data / 1.2f;
        
        // Store data
        data->data.bh1750.lux = lux;
        data->valid = true;
        
        ESP_LOGI(TAG, "BH1750 reading: %.1f lux", lux);
    } else {
        ESP_LOGE(TAG, "Failed to read BH1750 data");
        data->data.bh1750.lux = -1.0f; // Error value
    }
    
    return ret;
}

// Function to get I2C bus handle for other sensors
i2c_master_bus_handle_t bh1750_get_i2c_bus_handle(void) {
    return i2c_bus_handle;
}

// Function to check if BH1750 is initialized
bool bh1750_is_initialized(void) {
    return bh1750_initialized;
}
