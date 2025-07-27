/*
 * I2C Bus Scanner - Debug Tool
 * 
 * This utility scans the I2C bus and reports all responding devices
 * and their chip IDs. Useful for diagnosing sensor issues.
 */

#include "i2c_scanner.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "I2C_SCANNER";

// External I2C bus handle
extern i2c_master_bus_handle_t i2c_bus_handle;

void i2c_scanner_scan_bus(void) {
    if (i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "=== I2C Bus Scanner ===");
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    
    int device_count = 0;
    
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        // Create temporary device handle for testing
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
        };
        
        i2c_master_dev_handle_t test_handle = NULL;
        esp_err_t add_ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &test_handle);
        
        if (add_ret == ESP_OK) {
            // Try to read from address 0x00 (common register)
            uint8_t reg_addr = 0x00;
            uint8_t data;
            esp_err_t read_ret = i2c_master_transmit_receive(test_handle, &reg_addr, 1, &data, 1, 1000);
            
            if (read_ret == ESP_OK) {
                ESP_LOGI(TAG, "✓ DEVICE FOUND at address 0x%02X, register 0x00 = 0x%02X", addr, data);
                device_count++;
                
                // Try to read chip ID from common locations
                uint8_t chip_id_regs[] = {0xD0, 0x0F, 0x75, 0x00, 0xFF};
                for (int i = 0; i < 5; i++) {
                    uint8_t chip_id_reg = chip_id_regs[i];
                    uint8_t chip_id;
                    esp_err_t id_ret = i2c_master_transmit_receive(test_handle, &chip_id_reg, 1, &chip_id, 1, 1000);
                    if (id_ret == ESP_OK) {
                        ESP_LOGI(TAG, "  Register 0x%02X = 0x%02X", chip_id_reg, chip_id);
                    }
                }
            } else if (read_ret == ESP_ERR_TIMEOUT) {
                // Device ACK'ed address but didn't respond to read - still a device
                ESP_LOGI(TAG, "✓ DEVICE FOUND at address 0x%02X (ACK only, no data response)", addr);
                device_count++;
            }
            
            // Clean up
            i2c_master_bus_rm_device(test_handle);
        }
        
        // Small delay between tests
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "=== Scan Complete ===");
    ESP_LOGI(TAG, "Found %d device(s) on I2C bus", device_count);
    
    if (device_count == 0) {
        ESP_LOGW(TAG, "No I2C devices found - check wiring:");
        ESP_LOGW(TAG, "  SDA: GPIO21");
        ESP_LOGW(TAG, "  SCL: GPIO22");
        ESP_LOGW(TAG, "  VCC: 3.3V");
        ESP_LOGW(TAG, "  GND: GND");
    }
}

void i2c_scanner_identify_chip_id(uint8_t address) {
    if (i2c_bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "=== Detailed Analysis of Device at 0x%02X ===", address);
    
    // Create device handle
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };
    
    i2c_master_dev_handle_t dev_handle = NULL;
    esp_err_t add_ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &dev_handle);
    
    if (add_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create device handle: %s", esp_err_to_name(add_ret));
        return;
    }
    
    // Read multiple registers to identify the device
    struct {
        uint8_t reg;
        const char* name;
    } regs[] = {
        {0x00, "Status/Control"},
        {0x0F, "WHO_AM_I/ID"},
        {0x75, "WHO_AM_I (MPU)"},
        {0xD0, "Chip ID (BME/BMP)"},
        {0xF7, "Temp MSB"},
        {0xFA, "Pressure MSB"},
        {0xFD, "Humidity MSB"},
    };
    
    for (int i = 0; i < sizeof(regs)/sizeof(regs[0]); i++) {
        uint8_t data;
        esp_err_t ret = i2c_master_transmit_receive(dev_handle, &regs[i].reg, 1, &data, 1, 1000);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  0x%02X (%s): 0x%02X", regs[i].reg, regs[i].name, data);
        } else {
            ESP_LOGW(TAG, "  0x%02X (%s): READ FAILED (%s)", regs[i].reg, regs[i].name, esp_err_to_name(ret));
        }
    }
    
    // Clean up
    i2c_master_bus_rm_device(dev_handle);
    
    ESP_LOGI(TAG, "=== Analysis Complete ===");
}
