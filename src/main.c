/*
 * ESP32 MQTT Sensor Template
 * 
 * This project implements an ESP32-based sensor monitoring system that reads
 * data from various sensors and publishes the measurements to an MQTT broker.
 * 
 * FEATURES:
 * - Configurable sensor reading (I2C, SPI, GPIO, or other protocols)
 * - MQTT connectivity with automatic reconnection handling
 * - WiFi configuration with captive portal for easy setup
 * - Web-based configuration interface for all parameters
 * - Watchdog timer for system reliability
 * - Persistent configuration storage (NVS)
 * - Automatic version management and build automation
 * 
 * HARDWARE REQUIREMENTS:
 * - ESP32 development board
 * - Sensor(s) of your choice (configure protocol and pins as needed)
 * - Pull-up resistors for I2C lines if using I2C (typically 4.7kΩ)
 * 
 * DATA OUTPUT:
 * - Configurable sensor data (modify data structure as needed)
 * - Sensor status and processor metrics
 * - JSON format via MQTT for integration with home automation systems
 * 
 * CUSTOMIZATION POINTS:
 * Search for "TEMPLATE:" comments throughout the code to find areas that need
 * customization for your specific sensor(s) and use case.
 */

/*
 * CONDITIONAL LOGGING CONFIGURATION
 * 
 * To enable debug logging for this specific file, uncomment the line below.
 * This allows ESP_LOGD() calls to be compiled in and displayed when the 
 * global log level is set to DEBUG or VERBOSE.
 * 
 * Log Levels (from highest to lowest priority):
 * - ESP_LOG_ERROR   (E) - Error conditions
 * - ESP_LOG_WARN    (W) - Warning conditions  
 * - ESP_LOG_INFO    (I) - Informational messages
 * - ESP_LOG_DEBUG   (D) - Debug messages
 * - ESP_LOG_VERBOSE (V) - Verbose debug messages
 * 
 * To set global log level in menuconfig:
 * Component config > Log output > Default log verbosity
 * 
 * To set at runtime: esp_log_level_set("*", ESP_LOG_DEBUG);
 */
// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// Standard C input/output library
#include <stdio.h>
// FreeRTOS real-time operating system definitions
#include "freertos/FreeRTOS.h"
// FreeRTOS task management
#include "freertos/task.h"
// ESP32 GPIO driver
#include "driver/gpio.h"
// TEMPLATE: Include sensor-specific drivers here
// Examples:
// - I2C: #include "driver/i2c_master.h"
// - SPI: #include "driver/spi_master.h"
// - UART: #include "driver/uart.h"
// - ADC: #include "esp_adc/adc_oneshot.h"
#include "driver/i2c_master.h"  // Remove if not using I2C
// ESP-IDF logging library
#include "esp_log.h"
// Standard C string manipulation library (for memcmp)
#include <string.h> 

// Added for Wi-Fi and MQTT
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"
#include "cJSON.h" // For JSON formatting
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_efuse.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include "esp_chip_info.h"
#include "esp_ota_ops.h"
#include "esp_flash.h"
#include "spi_flash_mmap.h"
#include "project_version.h"  // Single source of truth for version
#include "esp_partition.h"
#include "esp_http_server.h"

// Watchdog timer includes
#include "esp_task_wdt.h"

// System monitoring includes
#include "esp_netif.h"

// Software watchdog variables
static uint32_t watchdog_timeout_ms = 0;  // Watchdog timeout (10× sample interval)
static TickType_t last_successful_publish = 0;  // Last successful MQTT publish time
static bool watchdog_enabled = false;  // Whether watchdog is enabled
static bool mqtt_publish_success = false;  // Flag for successful MQTT publish

// Debug logging flag - set to false to reduce log output (except watchdog logs)
static bool debug_logging = false;

#define WIFI_NVS_NAMESPACE "wifi_cfg"
#define WIFI_NVS_KEY_SSID "ssid"
#define WIFI_NVS_KEY_PASS "pass"
#define MQTT_NVS_KEY_URL "broker_url"
#define NVS_KEY_SAMPLE_INTERVAL "sample_interval"
#define NVS_KEY_DATA_TOPIC "data_topic"
#define NVS_KEY_WATCHDOG_COUNTER "watchdog_cnt"
#define DEFAULT_WIFI_SSID "yourSSID"
#define DEFAULT_WIFI_PASS "yourpassword"
// Temporary: Using public test broker - change this to your local broker once network is confirmed
#define DEFAULT_MQTT_BROKER_URL "mqtt://test.mosquitto.org:1883"
// Original: #define DEFAULT_MQTT_BROKER_URL "mqtt://192.168.1.100"
#define DEFAULT_SAMPLE_INTERVAL 5000L

static char wifi_ssid[33] = DEFAULT_WIFI_SSID;
static char wifi_pass[65] = DEFAULT_WIFI_PASS;
static char mqtt_broker_url[128] = DEFAULT_MQTT_BROKER_URL;
// TEMPLATE: Update default topic name for your sensor type
char data_topic[41] = "Data/Sensor";  // Default MQTT topic for sensor data
static long sample_interval_ms = DEFAULT_SAMPLE_INTERVAL;
static uint32_t watchdog_reset_counter = 0;  // Watchdog timer reset counter

#define BOOT_BTN_GPIO GPIO_NUM_0

// TEMPLATE: Configure your sensor communication pins and settings here
// Example I2C configuration (remove if not using I2C)
#define I2C_MASTER_SCL_IO           GPIO_NUM_22    // SCL GPIO pin
#define I2C_MASTER_SDA_IO           GPIO_NUM_21    // SDA GPIO pin
#define I2C_MASTER_FREQ_HZ          100000         // I2C master clock frequency (100kHz)
#define I2C_MASTER_TIMEOUT_MS       1000

// TEMPLATE: Replace with your sensor's I2C address and commands (remove if not using I2C)
// Example sensor configuration
#define SENSOR_I2C_ADDR             0x23           // Replace with your sensor's I2C address
#define SENSOR_CMD_INIT             0x01           // Replace with initialization command
#define SENSOR_CMD_READ             0x10           // Replace with read command
// Add more sensor-specific commands as needed

// TEMPLATE: Configure communication handles (modify based on your sensor type)
// I2C handles (remove if not using I2C)
static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t sensor_dev_handle;

// Tag used for logging messages from this module
static const char *TAG = "DATA_READER";

// Wi-Fi Configuration
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

// MQTT Configuration
#define MQTT_BROKER_URL "mqtt://192.168.1.5"

// Global variable for MQTT client
static esp_mqtt_client_handle_t mqtt_client;
static int wifi_retry_num = 0;

// TEMPLATE: Define your sensor data structure here
// Replace this with your specific sensor data fields
typedef struct {
    // TEMPLATE: Add your sensor-specific data fields here
    // Examples:
    // float temperature;    // Temperature in Celsius
    // float humidity;       // Relative humidity percentage
    // float pressure;       // Atmospheric pressure in hPa
    // int32_t light_level;  // Light intensity in lux
    // bool sensor_1_ok;     // First sensor status
    // bool sensor_2_ok;     // Second sensor status
    
    // Example fields (customize for your sensors):
    float sensor_value_1;    // Primary sensor reading
    float sensor_value_2;    // Secondary sensor reading (optional)
    bool sensor_ok;          // Overall sensor status flag
    
    // Add more fields as needed for your specific sensors
} sensor_data_t;

// Global instance of sensor data
static sensor_data_t current_sensor_data;


// PutInputCodeHere: Define your protocol-specific data structures here
// Example: If you need to store parsed fields from your input data
// typedef struct {
//     uint8_t id;
//     uint32_t value;
//     char strval[48];
//     int is_ascii;
// } parsed_field_t;
// #define MAX_PARSED_FIELDS 32
// static parsed_field_t parsed_fields[MAX_PARSED_FIELDS];
// static int parsed_fields_count = 0;

// Forward declarations for functions defined later in this file

// Forward declarations for NVS functions
static void load_watchdog_counter_from_nvs(void);
static void save_watchdog_counter_to_nvs(void);

// Forward declaration for DNS hijack task
void dns_hijack_task(void *pvParameter);

void url_decode(char *dst, const char *src, size_t dstsize);

// Forward declarations for HTTP handlers
static esp_err_t params_json_get_handler(httpd_req_t *req);
static esp_err_t params_update_post_handler(httpd_req_t *req);
static esp_err_t parameters_html_handler(httpd_req_t *req);
static esp_err_t captive_redirect_handler(httpd_req_t *req);
static esp_err_t sysinfo_json_get_handler(httpd_req_t *req);
static esp_err_t ota_firmware_handler(httpd_req_t *req);
static esp_err_t ota_filesystem_handler(httpd_req_t *req);

// New forward declarations for Wi-Fi and MQTT
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);
static void mqtt_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);
static bool test_mqtt_broker_connectivity(const char* broker_url);
static bool check_wifi_connection(void);
void wifi_init_sta(void);
static void mqtt_app_start(void);
void publish_sensor_data_mqtt(const sensor_data_t *sensor_data_ptr); // Publish sensor data to MQTT

// Forward declaration for DNS hijack task
void dns_hijack_task(void *pvParameter);

// Helper function to extract a 16-bit unsigned integer from a buffer.
// Assumes big-endian byte order (most significant byte first).
// buffer: Pointer to the byte array.
// offset: Starting index in the buffer from where to read the 16-bit integer.
// Returns the extracted 16-bit unsigned integer.
uint16_t unpack_u16_be(const uint8_t *buffer, int offset) {
    // Combine two consecutive bytes into a 16-bit value.
    // buffer[offset] is the most significant byte, buffer[offset+1] is the least significant.
    return ((uint16_t)buffer[offset] << 8) | buffer[offset + 1];
}

// Helper function to extract an 8-bit unsigned integer (a single byte) from a buffer.
// buffer: Pointer to the byte array.
// offset: Index in the buffer from where to read the 8-bit integer.
// Returns the extracted 8-bit unsigned integer.
uint8_t unpack_u8(const uint8_t *buffer, int offset) {
    // Directly return the byte at the specified offset.
    return buffer[offset];
}

// TEMPLATE: I2C Master initialization (modify for your sensor communication protocol)
// Remove this entire function if not using I2C
static esp_err_t init_i2c_master(void) {
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

    // TEMPLATE: Configure your sensor device on the I2C bus
    i2c_device_config_t sensor_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SENSOR_I2C_ADDR,  // Use your sensor's I2C address
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(i2c_bus_handle, &sensor_cfg, &sensor_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor device add failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

// TEMPLATE: Sensor initialization function
// Replace this with your specific sensor initialization sequence
static esp_err_t sensor_init(void) {
    esp_err_t ret;
    uint8_t cmd_data;

    // TEMPLATE: Add your sensor-specific initialization sequence here
    // Example I2C sensor initialization:
    
    // Power on or initialize the sensor
    cmd_data = SENSOR_CMD_INIT;  // Replace with your sensor's init command
    ret = i2c_master_transmit(sensor_dev_handle, &cmd_data, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for sensor to initialize

    // TEMPLATE: Add additional configuration commands for your sensor
    // Example: Set measurement mode, resolution, etc.
    // cmd_data = SENSOR_CMD_CONFIG;
    // ret = i2c_master_transmit(sensor_dev_handle, &cmd_data, 1, I2C_MASTER_TIMEOUT_MS);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Sensor configuration failed: %s", esp_err_to_name(ret));
    //     return ret;
    // }

    ESP_LOGI(TAG, "Sensor initialized successfully");
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for first measurement to be ready
    return ESP_OK;
}

// TEMPLATE: Sensor reading function
// Replace this with your specific sensor reading implementation
static esp_err_t read_sensor_value(float *value1, float *value2) {
    esp_err_t ret;
    uint8_t data[4];  // Adjust size based on your sensor's data format

    // TEMPLATE: Read data from your sensor
    // Example I2C sensor read:
    ret = i2c_master_receive(sensor_dev_handle, data, 2, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor read failed: %s", esp_err_to_name(ret));
        *value1 = -1.0f; // Error value
        *value2 = -1.0f; // Error value
        return ret;
    }

    // TEMPLATE: Convert raw data to meaningful values
    // Example conversion (customize for your sensor):
    uint16_t raw_data = (data[0] << 8) | data[1];
    *value1 = raw_data / 100.0f;  // Example conversion - adjust for your sensor
    *value2 = raw_data / 200.0f;  // Example second value - remove if not needed

    ESP_LOGD(TAG, "Sensor raw data: %d, value1: %.2f, value2: %.2f", raw_data, *value1, *value2);
    return ESP_OK;
}

// TEMPLATE: Main sensor data reading function
// Replace this function with your specific sensor reading logic
static bool read_sensor_data() {
    ESP_LOGI(TAG, "Reading sensor data...");
    
    // Initialize sensor data fields to default/invalid values
    current_sensor_data.sensor_value_1 = 0.0f;
    current_sensor_data.sensor_value_2 = 0.0f;
    current_sensor_data.sensor_ok = false;
    
    // TEMPLATE: Read from your sensor(s)
    // Example I2C sensor reading:
    float value1, value2;
    esp_err_t ret = read_sensor_value(&value1, &value2);
    
    if (ret == ESP_OK && value1 >= 0) {
        current_sensor_data.sensor_value_1 = value1;
        current_sensor_data.sensor_value_2 = value2;  // Remove if not needed
        current_sensor_data.sensor_ok = true;
        ESP_LOGI(TAG, "Sensor reading successful: %.2f, %.2f", value1, value2);
        
        // Publish MQTT data only on successful sensor read
        publish_sensor_data_mqtt(&current_sensor_data);
        ESP_LOGI(TAG, "MQTT data published after successful sensor read");
        return true;
    } else {
        current_sensor_data.sensor_value_1 = -1.0f;  // Error value
        current_sensor_data.sensor_value_2 = -1.0f;  // Error value
        current_sensor_data.sensor_ok = false;
        ESP_LOGE(TAG, "Sensor reading failed - skipping MQTT publish");
        return false;
    }
}

// Place these functions before app_main so they are visible to it
void load_wifi_config_from_nvs() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        size_t ssid_len = sizeof(wifi_ssid);
        size_t pass_len = sizeof(wifi_pass);
        if (nvs_get_str(nvs_handle, WIFI_NVS_KEY_SSID, wifi_ssid, &ssid_len) != ESP_OK) {
            strncpy(wifi_ssid, DEFAULT_WIFI_SSID, sizeof(wifi_ssid)-1);
            nvs_set_str(nvs_handle, WIFI_NVS_KEY_SSID, wifi_ssid);
        }
        if (nvs_get_str(nvs_handle, WIFI_NVS_KEY_PASS, wifi_pass, &pass_len) != ESP_OK) {
            strncpy(wifi_pass, DEFAULT_WIFI_PASS, sizeof(wifi_pass)-1);
            nvs_set_str(nvs_handle, WIFI_NVS_KEY_PASS, wifi_pass);
        }
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    } else {
        strncpy(wifi_ssid, DEFAULT_WIFI_SSID, sizeof(wifi_ssid)-1);
        strncpy(wifi_pass, DEFAULT_WIFI_PASS, sizeof(wifi_pass)-1);
    }
}

void load_mqtt_config_from_nvs() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        size_t url_len = sizeof(mqtt_broker_url);
        if (nvs_get_str(nvs_handle, MQTT_NVS_KEY_URL, mqtt_broker_url, &url_len) != ESP_OK) {
            strncpy(mqtt_broker_url, DEFAULT_MQTT_BROKER_URL, sizeof(mqtt_broker_url)-1);
            nvs_set_str(nvs_handle, MQTT_NVS_KEY_URL, mqtt_broker_url);
        }
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    } else {
        strncpy(mqtt_broker_url, DEFAULT_MQTT_BROKER_URL, sizeof(mqtt_broker_url)-1);
    }
}

void load_sample_interval_from_nvs() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        int64_t val = 0;
        if (nvs_get_i64(nvs_handle, NVS_KEY_SAMPLE_INTERVAL, &val) == ESP_OK && val > 0) {
            sample_interval_ms = (long)val;
        } else {
            sample_interval_ms = DEFAULT_SAMPLE_INTERVAL;
            nvs_set_i64(nvs_handle, NVS_KEY_SAMPLE_INTERVAL, (int64_t)sample_interval_ms);
        }
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    } else {
        sample_interval_ms = DEFAULT_SAMPLE_INTERVAL;
    }
}

void load_data_topic_from_nvs() {
    ESP_LOGI(TAG, "=== LOADING DATA TOPIC FROM NVS ===");
    ESP_LOGI(TAG, "Initial data_topic value: '%s'", data_topic);
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    ESP_LOGI(TAG, "NVS open result: %s", esp_err_to_name(err));
    
    if (err == ESP_OK) {
        size_t topic_len = sizeof(data_topic);
        ESP_LOGI(TAG, "Attempting to read NVS key '%s', buffer size: %d", NVS_KEY_DATA_TOPIC, topic_len);
        
        esp_err_t get_err = nvs_get_str(nvs_handle, NVS_KEY_DATA_TOPIC, data_topic, &topic_len);
        ESP_LOGI(TAG, "NVS get_str result: %s", esp_err_to_name(get_err));
        
        if (get_err != ESP_OK) {
            ESP_LOGW(TAG, "Data topic not found in NVS (error: %s), using default: Data/BH1750", esp_err_to_name(get_err));
            strncpy(data_topic, "Data/BH1750", sizeof(data_topic)-1);
            data_topic[sizeof(data_topic)-1] = '\0';
            esp_err_t set_err = nvs_set_str(nvs_handle, NVS_KEY_DATA_TOPIC, data_topic);
            ESP_LOGI(TAG, "Default Data topic save result: %s", esp_err_to_name(set_err));
            nvs_commit(nvs_handle);
        } else {
            ESP_LOGI(TAG, "Successfully loaded Data topic from NVS: '%s' (length: %d)", data_topic, topic_len);
        }
        nvs_close(nvs_handle);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS for Data topic: %s", esp_err_to_name(err));
        strncpy(data_topic, "Data/BH1750", sizeof(data_topic)-1);
        data_topic[sizeof(data_topic)-1] = '\0';
    }
    ESP_LOGI(TAG, "=== FINAL DATA TOPIC: '%s' ===", data_topic);
}

void save_data_topic_to_nvs(const char *topic) {
    nvs_handle_t nvs_handle;
    if (nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
        nvs_set_str(nvs_handle, NVS_KEY_DATA_TOPIC, topic);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
}

void load_watchdog_counter_from_nvs() {
    ESP_LOGI(TAG, "=== LOADING WATCHDOG COUNTER FROM NVS ===");
    ESP_LOGI(TAG, "Initial watchdog_reset_counter value: %lu", watchdog_reset_counter);
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    ESP_LOGI(TAG, "NVS open result: %s", esp_err_to_name(err));
    
    if (err == ESP_OK) {
        uint32_t val = 0;
        esp_err_t get_err = nvs_get_u32(nvs_handle, NVS_KEY_WATCHDOG_COUNTER, &val);
        ESP_LOGI(TAG, "NVS get_u32 result: %s", esp_err_to_name(get_err));
        
        if (get_err == ESP_OK) {
            watchdog_reset_counter = val;
            ESP_LOGI(TAG, "Successfully loaded watchdog counter from NVS: %lu", watchdog_reset_counter);
        } else {
            ESP_LOGW(TAG, "Watchdog counter not found in NVS (error: %s), using default: 0", esp_err_to_name(get_err));
            watchdog_reset_counter = 0;
            esp_err_t set_err = nvs_set_u32(nvs_handle, NVS_KEY_WATCHDOG_COUNTER, watchdog_reset_counter);
            ESP_LOGI(TAG, "Default watchdog counter save result: %s", esp_err_to_name(set_err));
            nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS for watchdog counter: %s", esp_err_to_name(err));
        watchdog_reset_counter = 0;
    }
    ESP_LOGI(TAG, "=== FINAL WATCHDOG COUNTER: %lu ===", watchdog_reset_counter);
}

void save_watchdog_counter_to_nvs() {
    ESP_LOGI(TAG, "=== SAVING WATCHDOG COUNTER TO NVS ===");
    ESP_LOGI(TAG, "Saving watchdog_reset_counter: %lu", watchdog_reset_counter);
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        esp_err_t set_err = nvs_set_u32(nvs_handle, NVS_KEY_WATCHDOG_COUNTER, watchdog_reset_counter);
        ESP_LOGI(TAG, "NVS set_u32 result: %s", esp_err_to_name(set_err));
        if (set_err == ESP_OK) {
            nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS for saving watchdog counter: %s", esp_err_to_name(err));
    }
}




// SPIFFS init
void init_spiffs() {
    ESP_LOGI(TAG, "Initializing SPIFFS...");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SPIFFS: %d kB total, %d kB used", total / 1024, used / 1024);
    }
}

// HTTP handler for /params.json
esp_err_t params_json_get_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "=== PARAMS.JSON REQUEST ===");
    
    // Debug: Show counter before reloading from NVS
    ESP_LOGI(TAG, "*** DEBUG: watchdog_reset_counter before reload: %lu ***", watchdog_reset_counter);
    
    // Reload watchdog counter from NVS to ensure latest value is displayed
    load_watchdog_counter_from_nvs();
    
    // Debug: Show counter after reloading from NVS
    ESP_LOGI(TAG, "*** DEBUG: watchdog_reset_counter after reload: %lu ***", watchdog_reset_counter);
    
    ESP_LOGI(TAG, "Current data_topic variable: '%s'", data_topic);
    ESP_LOGI(TAG, "Current wifi_ssid: '%s'", wifi_ssid);
    ESP_LOGI(TAG, "Current mqtt_broker_url: '%s'", mqtt_broker_url);
    ESP_LOGI(TAG, "Current sample_interval_ms: %ld", sample_interval_ms);
    ESP_LOGI(TAG, "Current watchdog_reset_counter: %lu", watchdog_reset_counter);
    
    char buf[512];
    snprintf(buf, sizeof(buf), "{\"ssid\":\"%s\",\"password\":\"%s\",\"mqtt_url\":\"%s\",\"sample_interval\":%ld,\"data_topic\":\"%s\",\"watchdog_reset_counter\":%lu}", 
             wifi_ssid, wifi_pass, mqtt_broker_url, sample_interval_ms, data_topic, watchdog_reset_counter);
    ESP_LOGI(TAG, "Sending params.json response: %s", buf);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "=== PARAMS.JSON RESPONSE SENT ===");
    return ESP_OK;
}

// Global flag to signal shutdown
static bool system_shutting_down = false;

// Reboot task function
static void reboot_task(void *param) {
    ESP_LOGI(TAG, "Reboot task started, shutting down gracefully...");
    
    // Set shutdown flag to signal other tasks to stop
    system_shutting_down = true;
    
    // Give time for HTTP response to complete
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "Disabling task watchdog before restart...");
    esp_task_wdt_deinit();
    
    // Give a bit more time for tasks to see the shutdown flag
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "Performing system restart NOW...");
    fflush(stdout);  // Ensure log message is sent
    
    esp_restart();
    
    // This line should never be reached
    vTaskDelete(NULL);
}

// HTTP handler for /update (POST)
esp_err_t params_update_post_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "=== UPDATE REQUEST RECEIVED ===");
    
    char buf[512];
    int ret = httpd_req_recv(req, buf, sizeof(buf)-1);
    if (ret <= 0) {
        ESP_LOGE(TAG, "Failed to receive update data");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = 0;
    
    ESP_LOGI(TAG, "Received update data: %s", buf);
    
    char ssid[33] = "", pass[65] = "", mqtt_url[128] = "";
    long interval = 5000;
    sscanf(strstr(buf, "ssid=")+5, "%32[^&]", ssid);
    sscanf(strstr(buf, "password=")+9, "%64[^&]", pass);
    sscanf(strstr(buf, "mqtt_url=")+9, "%127[^&]", mqtt_url);
    sscanf(strstr(buf, "sample_interval=")+15, "%ld", &interval);  // Fixed: "sample_interval=" is 15 chars, not 16
    // Decode URL-encoded values
    char ssid_dec[33], pass_dec[65], url_dec[128];
    url_decode(ssid_dec, ssid, sizeof(ssid_dec));
    url_decode(pass_dec, pass, sizeof(pass_dec));
    url_decode(url_dec, mqtt_url, sizeof(url_dec));
    strncpy(wifi_ssid, ssid_dec, sizeof(wifi_ssid)-1);
    strncpy(wifi_pass, pass_dec, sizeof(wifi_pass)-1);
    strncpy(mqtt_broker_url, url_dec, sizeof(mqtt_broker_url)-1);
    sample_interval_ms = interval;
    // Save to NVS
    nvs_handle_t nvs_handle;
    nvs_open(WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    nvs_set_str(nvs_handle, WIFI_NVS_KEY_SSID, wifi_ssid);
    nvs_set_str(nvs_handle, WIFI_NVS_KEY_PASS, wifi_pass);
    nvs_set_str(nvs_handle, MQTT_NVS_KEY_URL, mqtt_broker_url);
    nvs_set_i64(nvs_handle, NVS_KEY_SAMPLE_INTERVAL, (int64_t)sample_interval_ms);
    // Handle Data topic
    ESP_LOGI(TAG, "=== PROCESSING DATA TOPIC ===");
    ESP_LOGI(TAG, "Current data_topic before update: '%s'", data_topic);
    
    char data_topic_in[41] = "";
    char data_topic_dec[41];
    char *data_topic_ptr = strstr(buf, "data_topic=");
    if (data_topic_ptr) {
        sscanf(data_topic_ptr + 11, "%40[^&]", data_topic_in);  // Fixed: "data_topic=" is 11 chars, not 10
        ESP_LOGI(TAG, "Raw Data topic from form: '%s'", data_topic_in);
        
        url_decode(data_topic_dec, data_topic_in, sizeof(data_topic_dec));
        ESP_LOGI(TAG, "URL decoded Data topic: '%s'", data_topic_dec);
        
        strncpy(data_topic, data_topic_dec, sizeof(data_topic)-1);
        data_topic[sizeof(data_topic)-1] = '\0';
        ESP_LOGI(TAG, "Data topic updated to: '%s'", data_topic);
    } else {
        ESP_LOGW(TAG, "No data_topic found in request data: %s", buf);
    }
    
    esp_err_t nvs_err = nvs_set_str(nvs_handle, NVS_KEY_DATA_TOPIC, data_topic);
    ESP_LOGI(TAG, "Saved Data topic '%s' to NVS with key '%s': %s", data_topic, NVS_KEY_DATA_TOPIC, esp_err_to_name(nvs_err));
    ESP_LOGI(TAG, "=== DATA TOPIC PROCESSING COMPLETE ===");
    
    ESP_LOGI(TAG, "=== PARAMETER UPDATE PROCESSING COMPLETE ===");
    
    // Handle Watchdog Reset Counter
    ESP_LOGI(TAG, "=== PROCESSING WATCHDOG RESET COUNTER ===");
    ESP_LOGI(TAG, "Current watchdog_reset_counter before update: %lu", watchdog_reset_counter);
    
    char *watchdog_counter_ptr = strstr(buf, "watchdog_reset_counter=");
    if (watchdog_counter_ptr) {
        uint32_t counter_value = 0;
        sscanf(watchdog_counter_ptr + 23, "%lu", &counter_value);
        ESP_LOGI(TAG, "Watchdog counter from form: %lu", counter_value);
        
        watchdog_reset_counter = counter_value;
        ESP_LOGI(TAG, "Watchdog reset counter updated to: %lu", watchdog_reset_counter);
    } else {
        ESP_LOGW(TAG, "No watchdog_reset_counter found in request data: %s", buf);
    }
    
    esp_err_t watchdog_nvs_err = nvs_set_u32(nvs_handle, NVS_KEY_WATCHDOG_COUNTER, watchdog_reset_counter);
    ESP_LOGI(TAG, "Saved watchdog counter %lu to NVS with key '%s': %s", watchdog_reset_counter, NVS_KEY_WATCHDOG_COUNTER, esp_err_to_name(watchdog_nvs_err));
    ESP_LOGI(TAG, "=== WATCHDOG RESET COUNTER PROCESSING COMPLETE ===");
    
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Configuration saved successfully. Sending response...");
    httpd_resp_sendstr(req, "Saved. Rebooting...");
    
    ESP_LOGI(TAG, "Response sent. Starting reboot sequence...");
    // Give the HTTP response time to be sent
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Schedule reboot in a separate task to avoid blocking the HTTP response
    ESP_LOGI(TAG, "Creating reboot task...");
    BaseType_t task_created = xTaskCreate(reboot_task, "reboot_task", 2048, NULL, 1, NULL);
    if (task_created == pdPASS) {
        ESP_LOGI(TAG, "Reboot task created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create reboot task");
    }
    
    return ESP_OK;
}

// HTTP handler for /parameters.html
esp_err_t parameters_html_handler(httpd_req_t *req) {
    FILE *f = fopen("/spiffs/parameters.html", "r");
    if (!f) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    char buf[512];
    httpd_resp_set_type(req, "text/html");
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        httpd_resp_send_chunk(req, buf, n);
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// Captive portal redirect handler
esp_err_t captive_redirect_handler(httpd_req_t *req) {
    ESP_LOGD(TAG, "Captive portal request: %s", req->uri);
    
    // If the request is already for /parameters.html, serve it directly
    if (strcmp(req->uri, "/parameters.html") == 0) {
        return parameters_html_handler(req);
    }
    
    // Check for other valid endpoints and serve them directly
    if (strcmp(req->uri, "/params.json") == 0) {
        return params_json_get_handler(req);
    }
    if (strcmp(req->uri, "/sysinfo.json") == 0) {
        return sysinfo_json_get_handler(req);
    }
    if (strcmp(req->uri, "/update") == 0) {
        return params_update_post_handler(req);
    }
    
    // For any other request, redirect to the configuration page
    ESP_LOGD(TAG, "Redirecting %s to /parameters.html", req->uri);
    
    const char* redirect_html = 
        "<!DOCTYPE html>\n"
        "<html>\n"
        "<head>\n"
        "<meta charset=\"UTF-8\">\n"
        "<title>ESP32 Configuration</title>\n"
        "<meta http-equiv=\"refresh\" content=\"0; url=/parameters.html\">\n"
        "</head>\n"
        "<body>\n"
        "<p>Redirecting to configuration page...</p>\n"
        "<p>If you are not redirected automatically, <a href=\"/parameters.html\">click here</a>.</p>\n"
        "</body>\n"
        "</html>";
    
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_set_hdr(req, "Expires", "0");
    httpd_resp_send(req, redirect_html, strlen(redirect_html));
    
    return ESP_OK;
}

// Handler for /sysinfo.json
static esp_err_t sysinfo_json_get_handler(httpd_req_t *req) {
    char resp[512];
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    // Use esp_app_get_description if available, else fallback
    #if ESP_IDF_VERSION_MAJOR >= 5
    const esp_app_desc_t *app_desc = esp_app_get_description();
    #else
    const esp_app_desc_t *app_desc = esp_ota_get_app_description();
    #endif
    const char *idf_ver = esp_get_idf_version();
    
    // Use version from single source of truth (project_version.h)
    const char *version_str = PROJECT_VERSION;
    
    esp_reset_reason_t reason = esp_reset_reason();
    const char *reason_str = "Unknown";
    switch (reason) {
        case ESP_RST_POWERON: reason_str = "Power-on"; break;
        case ESP_RST_EXT: reason_str = "External"; break;
        case ESP_RST_SW: reason_str = "Software"; break;
        case ESP_RST_PANIC: reason_str = "Panic"; break;
        case ESP_RST_INT_WDT: reason_str = "Interrupt WDT"; break;
        case ESP_RST_TASK_WDT: reason_str = "Task WDT"; break;
        case ESP_RST_WDT: reason_str = "Other WDT"; break;
        case ESP_RST_DEEPSLEEP: reason_str = "Deep Sleep"; break;
        case ESP_RST_BROWNOUT: reason_str = "Brownout"; break;
        case ESP_RST_SDIO: reason_str = "SDIO"; break;
        default: break;
    }
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char chip_id[18];
    snprintf(chip_id, sizeof(chip_id), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    // Get flash chip information
    uint32_t flash_id = 0x000000; // Placeholder
    uint32_t flash_size = 0;
    
    // Get actual flash size - ESP32 DevKit V1 typically has 4MB
    // Use the more reliable method of checking all partitions to estimate total flash
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    uint32_t max_offset = 0;
    while (it != NULL) {
        const esp_partition_t *partition = esp_partition_get(it);
        uint32_t partition_end = partition->address + partition->size;
        if (partition_end > max_offset) {
            max_offset = partition_end;
        }
        it = esp_partition_next(it);
    }
    if (it) esp_partition_iterator_release(it);
    
    // Round up to nearest MB and ensure it's at least 4MB for ESP32 DevKit V1
    if (max_offset > 0) {
        flash_size = ((max_offset + 1024*1024 - 1) / (1024*1024)) * 1024*1024; // Round up to nearest MB
        if (flash_size < 4*1024*1024) flash_size = 4*1024*1024; // Minimum 4MB for ESP32 DevKit V1
    } else {
        flash_size = 4 * 1024 * 1024; // Default 4MB for ESP32 DevKit V1
    }
    snprintf(resp, sizeof(resp),
        "{"
        "\"version\":\"%s\","  // Program Version
        "\"build\":\"%s %s\"," // Build Date & Time
        "\"sdk\":\"%s\","      // Core/SDK Version
        "\"restart_reason\":\"%s\"," // Restart Reason
        "\"chip_id\":\"%s\","  // ESP Chip Id
        "\"flash_id\":\"%06lX\"," // Flash Chip Id
        "\"flash_size\":\"%lu KB\""
        "}",
        version_str,  // Use version from version.txt file
        app_desc->date, app_desc->time,
        idf_ver,
        reason_str,
        chip_id,
        (unsigned long)(flash_id & 0xFFFFFF),
        (unsigned long)(flash_size / 1024)
    );
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// OTA Firmware Update Handler
static esp_err_t ota_firmware_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "OTA firmware handler called - Content-Length: %d", req->content_len);
    ESP_LOGI(TAG, "Request method: %d, URI: %s", req->method, req->uri);
    
    // Add CORS headers
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    
    // Handle preflight requests
    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *ota_partition = NULL;
    esp_err_t err = ESP_OK;
    char *buf = NULL;
    const size_t buffer_size = 2048;  // Reduced buffer size for better memory management
    int received = 0;
    int remaining = req->content_len;

    ESP_LOGI(TAG, "Starting OTA firmware update, size: %d bytes", remaining);
    
    // Allocate buffer on heap to avoid stack overflow
    buf = malloc(buffer_size);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate %d bytes for upload buffer", buffer_size);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    // Get next OTA partition
    ota_partition = esp_ota_get_next_update_partition(NULL);
    if (!ota_partition) {
        ESP_LOGE(TAG, "No OTA partition found");
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%lx", 
             ota_partition->subtype, ota_partition->address);

    // Begin OTA
    err = esp_ota_begin(ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    // Receive and write firmware data
    int timeout_count = 0;
    int zero_recv_count = 0;
    int last_fw_progress = -1;  // Move static variable to local scope
    
    while (remaining > 0) {
        int chunk_size = (remaining < buffer_size) ? remaining : buffer_size;
        int recv_len = httpd_req_recv(req, buf, chunk_size);
        
        if (recv_len < 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                timeout_count++;
                ESP_LOGW(TAG, "Socket timeout #%d, continuing... (%d bytes remaining)", timeout_count, remaining);
                if (timeout_count > 15) {  // Increased tolerance for large files
                    ESP_LOGE(TAG, "Too many timeouts, aborting");
                    esp_ota_abort(ota_handle);
                    free(buf);
                    httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "Upload timeout - too many socket timeouts");
                    return ESP_FAIL;
                }
                vTaskDelay(pdMS_TO_TICKS(200));  // Longer delay for large file processing
                continue;
            }
            ESP_LOGE(TAG, "File reception failed with error: %d", recv_len);
            esp_ota_abort(ota_handle);
            free(buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File reception failed - connection error");
            return ESP_FAIL;
        }
        
        if (recv_len == 0) {
            zero_recv_count++;
            ESP_LOGW(TAG, "Received 0 bytes #%d, continuing... (%d bytes remaining)", zero_recv_count, remaining);
            if (zero_recv_count > 8) {  // Increased tolerance for large files
                ESP_LOGE(TAG, "Too many zero-byte receives, connection likely closed");
                esp_ota_abort(ota_handle);
                free(buf);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Connection closed unexpectedly");
                return ESP_FAIL;
            }
            vTaskDelay(pdMS_TO_TICKS(100));  // Longer delay before retry
            continue;
        }
        
        // Reset counters on successful receive
        timeout_count = 0;
        zero_recv_count = 0;

        err = esp_ota_write(ota_handle, buf, recv_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            free(buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed - firmware corrupted?");
            return ESP_FAIL;
        }

        received += recv_len;
        remaining -= recv_len;
        
        // Log progress every 5% for large files
        int progress = (received * 100) / req->content_len;
        if (progress != last_fw_progress && progress % 5 == 0) {
            ESP_LOGI(TAG, "Firmware Progress: %d%% (%d/%d bytes, free heap: %lu)", 
                     progress, received, req->content_len, (unsigned long)esp_get_free_heap_size());
            last_fw_progress = progress;
        }
        
        // Yield to other tasks periodically for large uploads
        if ((received % (buffer_size * 4)) == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));  // Brief yield every ~8KB
        }
    }

    free(buf);  // Clean up allocated buffer

    // End OTA
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Firmware validation failed - invalid binary");
        } else {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        }
        return ESP_FAIL;
    }

    // Set boot partition
    err = esp_ota_set_boot_partition(ota_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA firmware update successful, rebooting...");
    httpd_resp_sendstr(req, "Firmware Update Successful! Rebooting...");
    
    // Reboot after a short delay
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    
    return ESP_OK;
}

// OTA Filesystem Update Handler
static esp_err_t ota_filesystem_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "OTA filesystem handler called - Content-Length: %d", req->content_len);
    ESP_LOGI(TAG, "Request method: %d, URI: %s", req->method, req->uri);
    
    // Add CORS headers
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    
    // Handle preflight requests
    if (req->method == HTTP_OPTIONS) {
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }
    
    const esp_partition_t *spiffs_partition = NULL;
    esp_err_t err = ESP_OK;
    char *buf = NULL;
    const size_t buffer_size = 2048;  // Reduced buffer size for better memory management
    int received = 0;
    int remaining = req->content_len;

    ESP_LOGI(TAG, "Starting OTA filesystem update, size: %d bytes", remaining);
    
    // Allocate buffer on heap to avoid stack overflow
    buf = malloc(buffer_size);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate %d bytes for upload buffer", buffer_size);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Memory allocation failed");
        return ESP_FAIL;
    }

    // Find SPIFFS partition
    spiffs_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
    if (!spiffs_partition) {
        ESP_LOGE(TAG, "No SPIFFS partition found");
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No SPIFFS partition found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Erasing SPIFFS partition at offset 0x%lx, size: %lu bytes", 
             spiffs_partition->address, spiffs_partition->size);

    // Erase the partition first
    err = esp_partition_erase_range(spiffs_partition, 0, spiffs_partition->size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase SPIFFS partition: %s", esp_err_to_name(err));
        free(buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to erase SPIFFS partition");
        return ESP_FAIL;
    }

    // Write filesystem data
    size_t offset = 0;
    int timeout_count = 0;
    int zero_recv_count = 0;
    int last_fs_progress = -1;  // Move static variable to local scope
    
    while (remaining > 0) {
        int chunk_size = (remaining < buffer_size) ? remaining : buffer_size;
        int recv_len = httpd_req_recv(req, buf, chunk_size);
        
        if (recv_len < 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                timeout_count++;
                ESP_LOGW(TAG, "Socket timeout #%d, continuing... (%d bytes remaining)", timeout_count, remaining);
                if (timeout_count > 15) {  // Increased tolerance for large files
                    ESP_LOGE(TAG, "Too many timeouts, aborting");
                    free(buf);
                    httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "Upload timeout - too many socket timeouts");
                    return ESP_FAIL;
                }
                vTaskDelay(pdMS_TO_TICKS(200));  // Longer delay for large file processing
                continue;
            }
            ESP_LOGE(TAG, "File reception failed with error: %d", recv_len);
            free(buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File reception failed - connection error");
            return ESP_FAIL;
        }
        
        if (recv_len == 0) {
            zero_recv_count++;
            ESP_LOGW(TAG, "Received 0 bytes #%d, continuing... (%d bytes remaining)", zero_recv_count, remaining);
            if (zero_recv_count > 8) {  // Increased tolerance for large files
                ESP_LOGE(TAG, "Too many zero-byte receives, connection likely closed");
                free(buf);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Connection closed unexpectedly");
                return ESP_FAIL;
            }
            vTaskDelay(pdMS_TO_TICKS(100));  // Longer delay before retry
            continue;
        }
        
        // Reset counters on successful receive
        timeout_count = 0;
        zero_recv_count = 0;

        err = esp_partition_write(spiffs_partition, offset, buf, recv_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_partition_write failed: %s", esp_err_to_name(err));
            free(buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Partition write failed - flash error");
            return ESP_FAIL;
        }

        received += recv_len;
        remaining -= recv_len;
        offset += recv_len;
        
        // Log progress every 5% for large files
        int progress = (received * 100) / req->content_len;
        if (progress != last_fs_progress && progress % 5 == 0) {
            ESP_LOGI(TAG, "SPIFFS Progress: %d%% (%d/%d bytes, free heap: %lu)", 
                     progress, received, req->content_len, (unsigned long)esp_get_free_heap_size());
            last_fs_progress = progress;
        }
        
        // Yield to other tasks periodically for large uploads
        if ((received % (buffer_size * 4)) == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));  // Brief yield every ~8KB
        }
    }

    free(buf);  // Clean up allocated buffer
    ESP_LOGI(TAG, "OTA filesystem update successful, rebooting...");
    httpd_resp_sendstr(req, "Filesystem Update Successful! Rebooting...");
    
    // Reboot after a short delay
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    
    return ESP_OK;
}



static void start_ap_and_captive_portal() {
    ESP_LOGI(TAG, "Starting AP and captive portal...");
    
    // Start AP mode only for configuration (no STA mode to avoid connection attempts)
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Set to AP mode only - do not enable STA to avoid connection attempts
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_LOGI(TAG, "WiFi set to AP mode only for configuration");
    
    // Configure AP with a unique name based on MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char ap_ssid[32];
    snprintf(ap_ssid, sizeof(ap_ssid), "ESP32-CONFIG-%02X%02X", mac[4], mac[5]);
    
    wifi_config_t ap_config = {
        .ap = {
            .ssid_len = 0,
            .password = "",
            .max_connection = 4,
            .authmode = WIFI_AUTH_OPEN,
            .beacon_interval = 100
        }
    };
    strncpy((char*)ap_config.ap.ssid, ap_ssid, sizeof(ap_config.ap.ssid));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    // Set AP IP and DHCP
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    
    // Stop DHCP server before configuring IP
    esp_netif_dhcps_stop(ap_netif);
    
    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr = esp_ip4addr_aton("192.168.4.1");
    ip_info.netmask.addr = esp_ip4addr_aton("255.255.255.0");
    ip_info.gw.addr = esp_ip4addr_aton("192.168.4.1");
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    
    // Configure DHCP server options
    esp_netif_dhcps_option(ap_netif, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &ip_info.ip, sizeof(ip_info.ip));
    
    esp_netif_dhcps_start(ap_netif);

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi AP started: %s (IP: 192.168.4.1)", ap_ssid);
    
    // Wait a moment for WiFi to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Start SPIFFS
    ESP_LOGI(TAG, "About to call init_spiffs()...");
    init_spiffs();
    ESP_LOGI(TAG, "init_spiffs() completed successfully");
    
    // Start HTTP server
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 16;  // Increase limit
    config.stack_size = 8192;      // Increase stack size
    
    // Configure for large file uploads (OTA)
    config.recv_wait_timeout = 120;     // 2 minutes timeout for receiving data
    config.send_wait_timeout = 120;     // 2 minutes timeout for sending data
    config.lru_purge_enable = true;     // Enable LRU purging of connections
    config.max_open_sockets = 4;        // Limit concurrent connections
    config.backlog_conn = 2;            // Reduce backlog
    
    ESP_LOGI(TAG, "HTTP server config: recv_timeout=%d, send_timeout=%d, max_sockets=%d", 
             config.recv_wait_timeout, config.send_wait_timeout, config.max_open_sockets);
    
    esp_err_t server_err = httpd_start(&server, &config);
    if (server_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(server_err));
        return;
    }
    
    ESP_LOGI(TAG, "HTTP server started successfully");
    
    // Register handlers in order of specificity (most specific first)
    httpd_uri_t params_json = { .uri = "/params.json", .method = HTTP_GET, .handler = params_json_get_handler, .user_ctx = NULL };
    httpd_uri_t params_update = { .uri = "/update", .method = HTTP_POST, .handler = params_update_post_handler, .user_ctx = NULL };
    httpd_uri_t parameters_html = { .uri = "/parameters.html", .method = HTTP_GET, .handler = parameters_html_handler, .user_ctx = NULL };
    httpd_uri_t sysinfo_json = { .uri = "/sysinfo.json", .method = HTTP_GET, .handler = sysinfo_json_get_handler, .user_ctx = NULL };
    httpd_uri_t ota_firmware = { .uri = "/ota/firmware", .method = HTTP_POST, .handler = ota_firmware_handler, .user_ctx = NULL };
    httpd_uri_t ota_filesystem = { .uri = "/ota/filesystem", .method = HTTP_POST, .handler = ota_filesystem_handler, .user_ctx = NULL };
    httpd_uri_t ota_firmware_options = { .uri = "/ota/firmware", .method = HTTP_OPTIONS, .handler = ota_firmware_handler, .user_ctx = NULL };
    httpd_uri_t ota_filesystem_options = { .uri = "/ota/filesystem", .method = HTTP_OPTIONS, .handler = ota_filesystem_handler, .user_ctx = NULL };
    
    // Register specific handlers first
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &params_json));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &params_update));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &parameters_html));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &sysinfo_json));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ota_firmware));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ota_filesystem));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ota_firmware_options));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ota_filesystem_options));
    
    // Register wildcard handler last to catch all other requests
    httpd_uri_t captive = { .uri = "/*", .method = HTTP_GET, .handler = captive_redirect_handler, .user_ctx = NULL };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &captive));
    
    ESP_LOGI(TAG, "All HTTP handlers registered successfully");
    ESP_LOGI(TAG, "Captive portal ready - connect to '%s' and visit any website", ap_ssid);
}

void ap_config_task(void *pvParameter) {
    ESP_LOGI(TAG, "Starting AP configuration mode...");
    
    // Debug: Show watchdog counter before loading from NVS
    ESP_LOGI(TAG, "*** DEBUG: watchdog_reset_counter before NVS load (AP mode): %lu ***", watchdog_reset_counter);
    
    // Load NVS values before starting AP and HTTP server
    load_wifi_config_from_nvs();
    load_mqtt_config_from_nvs();
    load_sample_interval_from_nvs();
    load_data_topic_from_nvs();
    load_watchdog_counter_from_nvs();
    
    // Debug: Show watchdog counter after loading from NVS
    ESP_LOGI(TAG, "*** DEBUG: watchdog_reset_counter after NVS load (AP mode): %lu ***", watchdog_reset_counter);
    
    ESP_LOGI(TAG, "=== CONFIG LOADED FROM NVS (AP MODE) ===");
    ESP_LOGI(TAG, "WiFi SSID: %s", wifi_ssid);
    ESP_LOGI(TAG, "WiFi PASS: %s", wifi_pass);
    ESP_LOGI(TAG, "MQTT Broker URL: %s", mqtt_broker_url);
    ESP_LOGI(TAG, "Data Topic: %s", data_topic);
    ESP_LOGI(TAG, "Sample interval (ms): %ld", sample_interval_ms);
    ESP_LOGI(TAG, "Watchdog reset counter: %lu", watchdog_reset_counter);
    ESP_LOGI(TAG, "=== STARTING AP CONFIG MODE ===");
    
    // Note: esp_netif_init() and esp_event_loop_create_default() 
    // are already called in main app initialization
    
    // Start the captive portal
    start_ap_and_captive_portal();
    
    // Wait a moment for the HTTP server to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Start DNS hijack server for captive portal
    TaskHandle_t dns_task_handle = NULL;
    BaseType_t dns_task_result = xTaskCreate(dns_hijack_task, "dns_hijack_task", 4096, NULL, 4, &dns_task_handle);
    if (dns_task_result == pdPASS) {
        ESP_LOGI(TAG, "DNS hijack task started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create DNS hijack task");
    }
    
    ESP_LOGI(TAG, "AP configuration mode fully initialized");
    ESP_LOGI(TAG, "Connect to ESP32-CONFIG-XXXX network and visit any website to configure");
    
    vTaskDelete(NULL);
}

// Blink task for onboard LED
#define ONBOARD_LED_GPIO GPIO_NUM_2

// Blink the onboard LED for a short time (heartbeat)
void blink_heartbeat() {
    gpio_set_direction(ONBOARD_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ONBOARD_LED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(ONBOARD_LED_GPIO, 0);
}

// Main application entry point.
void app_main(void) {
    // ============================================================
    // REDUCE LOGGING OUTPUT - Uncomment one of these lines:
    // ============================================================
    esp_log_level_set("*", ESP_LOG_WARN);   // Show only warnings and errors
    // esp_log_level_set("*", ESP_LOG_INFO);   // Show info, warnings and errors (for debugging)
    // esp_log_level_set("*", ESP_LOG_ERROR);  // Show only errors
    // esp_log_level_set("*", ESP_LOG_NONE);   // Turn off all logging
    
    // Initialize NVS first for all code paths
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Debug: Show initial watchdog counter value at app start
    ESP_LOGI(TAG, "*** DEBUG: Initial watchdog_reset_counter at app start: %lu ***", watchdog_reset_counter);

    // Configure Task Watchdog Timer
    ESP_LOGI(TAG, "Configuring Task Watchdog Timer...");
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 30000, // 30 second timeout
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Monitor all cores
        .trigger_panic = true // Trigger panic on timeout
    };
    esp_err_t twdt_err = esp_task_wdt_init(&twdt_config);
    ESP_LOGI(TAG, "esp_task_wdt_init() returned: %d", twdt_err);
    // No esp_task_wdt_get_timeout() in ESP-IDF; cannot log actual timeout at runtime
    if (twdt_err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Task Watchdog Timer already initialized");
    } else {
        ESP_ERROR_CHECK(twdt_err);
        ESP_LOGI(TAG, "Task Watchdog Timer initialized successfully");
    }
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL)); // Add current task to watchdog
    ESP_LOGI(TAG, "Task Watchdog Timer configured successfully");

    // Initialize network stack early for both AP and STA modes
    ESP_LOGI(TAG, "Initializing network stack...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "Network stack initialized successfully");

    ESP_LOGI(TAG, "Waiting 10 seconds for boot button press...");
    gpio_set_direction(BOOT_BTN_GPIO, GPIO_MODE_INPUT);
    bool boot_btn_pressed = false;
    for (int i = 0; i < 1000; ++i) { // 10 seconds, check every 10ms
        if (gpio_get_level(BOOT_BTN_GPIO) == 0) {
            boot_btn_pressed = true;
            ESP_LOGI(TAG, "Boot button press detected during wait!");
            break;
        }
        // Feed watchdog every 100 iterations (1 second)
        if (i % 100 == 0) {
            esp_task_wdt_reset();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "Boot button wait finished, continuing startup...");
    if (boot_btn_pressed) {
        ESP_LOGI(TAG, "Entering AP config mode due to boot button press.");
        xTaskCreate(ap_config_task, "ap_config_task", 8192, NULL, 5, NULL);
        while (1) {
            esp_task_wdt_reset(); // Feed watchdog in config mode
            vTaskDelay(1000/portTICK_PERIOD_MS); // Block forever
        }
    }

    // TEMPLATE: Initialize sensor communication (modify for your sensor type)
    // Remove I2C initialization if not using I2C sensors
    ESP_LOGI(TAG, "Initializing sensor communication...");
    esp_err_t comm_err = init_i2c_master();  // Change to your communication init function
    if (comm_err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor communication initialization failed: %s", esp_err_to_name(comm_err));
    } else {
        ESP_LOGI(TAG, "Sensor communication initialized successfully");
        
        // TEMPLATE: Initialize your specific sensor(s)
        esp_err_t sensor_err = sensor_init();  // Replace with your sensor init function
        if (sensor_err != ESP_OK) {
            ESP_LOGE(TAG, "Sensor initialization failed: %s", esp_err_to_name(sensor_err));
        } else {
            ESP_LOGI(TAG, "Sensor initialized successfully");
        }
    }

    // Debug: Show watchdog counter before loading from NVS
    ESP_LOGI(TAG, "*** DEBUG: watchdog_reset_counter before NVS load (normal mode): %lu ***", watchdog_reset_counter);

    load_wifi_config_from_nvs();
    load_mqtt_config_from_nvs();
    load_sample_interval_from_nvs();
    load_data_topic_from_nvs();
    load_watchdog_counter_from_nvs();
    
    // Debug: Show watchdog counter after loading from NVS
    ESP_LOGI(TAG, "*** DEBUG: watchdog_reset_counter after NVS load (normal mode): %lu ***", watchdog_reset_counter);
    
    ESP_LOGI(TAG, "=== CONFIG LOADED FROM NVS ===");
    ESP_LOGI(TAG, "WiFi SSID: %s", wifi_ssid);
    ESP_LOGI(TAG, "WiFi PASS: %s", wifi_pass);
    ESP_LOGI(TAG, "MQTT Broker URL: %s", mqtt_broker_url);
    ESP_LOGI(TAG, "Data Topic: %s", data_topic);
    ESP_LOGI(TAG, "Sample interval (ms): %ld", sample_interval_ms);
    ESP_LOGI(TAG, "Watchdog reset counter: %lu", watchdog_reset_counter);
    
    // Initialize SPIFFS for normal operation mode
    ESP_LOGI(TAG, "About to call init_spiffs() in normal mode...");
    init_spiffs();
    ESP_LOGI(TAG, "init_spiffs() completed successfully in normal mode");
    
    ESP_LOGI(TAG, "=== STARTING WIFI INIT ===");
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta(); // Initialize Wi-Fi
    
    // Initialize software watchdog
    // Watchdog timeout is 10× sample interval (in ms), inactive if timeout ≤10,000 ms
    watchdog_timeout_ms = sample_interval_ms * 10;
    if (watchdog_timeout_ms > 10000) {
        watchdog_enabled = true;
        last_successful_publish = xTaskGetTickCount();
        ESP_LOGI(TAG, "Software watchdog enabled with timeout: %lu ms (10× sample interval)", watchdog_timeout_ms);
        ESP_LOGI(TAG, "WDT will reset ESP32 only if no MQTT data is published for %lu ms", watchdog_timeout_ms);
    } else {
        watchdog_enabled = false;
        ESP_LOGI(TAG, "Software watchdog inactive (timeout ≤10,000 ms, sample interval: %lu ms)", sample_interval_ms);
    }

    // Main loop to periodically read sensor data and publish to MQTT.
    while (1) {
        ESP_LOGD(TAG, "[MAIN LOOP] Start iteration");
        esp_task_wdt_reset();
        
        // Check software watchdog timeout - only triggers if no MQTT data published
        ESP_LOGD(TAG, "[MAIN LOOP] Check software watchdog");
        if (watchdog_enabled) {
            TickType_t current_time = xTaskGetTickCount();
            TickType_t elapsed_ms = pdTICKS_TO_MS(current_time - last_successful_publish);
            if (elapsed_ms >= watchdog_timeout_ms) {
                ESP_LOGE(TAG, "Software watchdog timeout! No successful MQTT publish in %lu ms (timeout: %lu ms)", elapsed_ms, watchdog_timeout_ms);
                
                // Debug: Show counter before increment
                ESP_LOGE(TAG, "*** DEBUG: watchdog_reset_counter BEFORE increment: %lu ***", watchdog_reset_counter);
                
                // Increment watchdog reset counter and save to NVS
                watchdog_reset_counter++;
                ESP_LOGE(TAG, "Incrementing watchdog reset counter to: %lu", watchdog_reset_counter);
                
                // Debug: Show counter after increment
                ESP_LOGE(TAG, "*** DEBUG: watchdog_reset_counter AFTER increment: %lu ***", watchdog_reset_counter);
                
                save_watchdog_counter_to_nvs();
                
                // Debug: Verify save by reloading from NVS
                uint32_t verify_counter = 0;
                nvs_handle_t verify_handle;
                esp_err_t verify_err = nvs_open(WIFI_NVS_NAMESPACE, NVS_READONLY, &verify_handle);
                if (verify_err == ESP_OK) {
                    esp_err_t get_err = nvs_get_u32(verify_handle, NVS_KEY_WATCHDOG_COUNTER, &verify_counter);
                    ESP_LOGE(TAG, "*** DEBUG: Verification read from NVS result: %s, value: %lu ***", 
                             esp_err_to_name(get_err), verify_counter);
                    nvs_close(verify_handle);
                } else {
                    ESP_LOGE(TAG, "*** DEBUG: Failed to open NVS for verification: %s ***", esp_err_to_name(verify_err));
                }
                
                ESP_LOGE(TAG, "Forcing system restart...");
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            }
        }
        ESP_LOGD(TAG, "[MAIN LOOP] After software watchdog check");

        mqtt_publish_success = false;
        ESP_LOGD(TAG, "[MAIN LOOP] After reset mqtt_publish_success");

        // Read sensor data and attempt to publish if successful
        bool sensor_read_success = read_sensor_data();
        
        if (sensor_read_success) {
            ESP_LOGD(TAG, "[MAIN LOOP] Sensor read successful, data will be published to MQTT");
        } else {
            ESP_LOGW(TAG, "[MAIN LOOP] Sensor read failed, no MQTT data will be published this cycle");
        }
        ESP_LOGD(TAG, "[MAIN LOOP] After read_sensor_data");

        if (debug_logging) {
            ESP_LOGI(TAG, "Waiting %ld ms before next sensor read cycle...", sample_interval_ms);
        }
        if (sample_interval_ms > 5000) {
            uint32_t remaining_ms = sample_interval_ms;
            while (remaining_ms > 0) {
                uint32_t chunk_ms = (remaining_ms > 5000) ? 5000 : remaining_ms;
                vTaskDelay(pdMS_TO_TICKS(chunk_ms));
                esp_task_wdt_reset();
                remaining_ms -= chunk_ms;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(sample_interval_ms));
        }
        ESP_LOGD(TAG, "[MAIN LOOP] End of iteration, feeding TWDT");
        esp_task_wdt_reset();
    }
}

// Wi-Fi Event Handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    printf("[WIFI] Event handler called: base=%s, event_id=%ld\n", event_base, event_id);
    
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        printf("[WIFI] WIFI_EVENT_STA_START - calling esp_wifi_connect()\n");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        printf("[WIFI] WIFI_EVENT_STA_DISCONNECTED - retry_num=%d\n", wifi_retry_num);
        if (wifi_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            wifi_retry_num++;
            printf("[WIFI] retry to connect to the AP\n");
        } else {
            printf("[WIFI] connect to the AP fail\n");
            printf("[WIFI] WiFi SSID used: %s\n", wifi_ssid);
            printf("[WIFI] WiFi Password used: %s\n", wifi_pass);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        printf("[WIFI] IP_EVENT_STA_GOT_IP received!\n");
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("[WIFI] got ip:" IPSTR "\n", IP2STR(&event->ip_info.ip));
        printf("Obtained IP address: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        wifi_retry_num = 0;
        // Start MQTT client once IP is obtained
        printf("[WIFI] About to start MQTT client...\n");
        mqtt_app_start();
        printf("[WIFI] MQTT client start function completed\n");
    } else {
        printf("[WIFI] Unhandled event: base=%s, event_id=%ld\n", event_base, event_id);
    }
    printf("[WIFI] Event handler completed\n");
}

// Wi-Fi Initialization
void wifi_init_sta(void)
{
    // Note: esp_netif_init() and esp_event_loop_create_default() 
    // are already called in main app initialization
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Or other appropriate authmode
        },
    };
    strncpy((char*)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid)-1);
    strncpy((char*)wifi_config.sta.password, wifi_pass, sizeof(wifi_config.sta.password)-1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

// MQTT Event Handler
static void mqtt_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ld", event_base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    printf("[MQTT] Event received: %ld\n", event_id);
    // esp_mqtt_client_handle_t client = event->client; // Not used in this basic handler
    // int msg_id; // Not used here
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        printf("[MQTT] Connected to broker successfully!\n");
        // Example: Subscribe to a topic (optional)
        // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        printf("[MQTT] Disconnected from broker\n");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        printf("[MQTT] Message published successfully!\n");
        
        // Flash the LED to indicate successful MQTT publish (heartbeat)
        blink_heartbeat();
        
        // Reset the software watchdog timer (if enabled)
        if (watchdog_enabled) {
            last_successful_publish = xTaskGetTickCount(); // Update last publish time
        }
        break;
    case MQTT_EVENT_DATA: // Incoming data event (if subscribed)
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        printf("[MQTT] Connection error occurred!\n");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGE(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGE(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGE(TAG, "MQTT Broker URL: %s", mqtt_broker_url);
            printf("[MQTT] TCP Transport error - check broker URL and network connectivity\n");
            printf("[MQTT] Current broker: %s\n", mqtt_broker_url);
            printf("[MQTT] Possible issues:\n");
            printf("  1. Broker IP address not reachable\n");
            printf("  2. Broker not running on specified port (default 1883)\n");
            printf("  3. Network connectivity issues\n");
            printf("  4. Firewall blocking connection\n");
        } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            ESP_LOGE(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            printf("[MQTT] Broker refused connection (code: 0x%x)\n", event->error_handle->connect_return_code);
            printf("[MQTT] Check broker configuration and authentication\n");
        } else {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
            printf("[MQTT] Unknown error type: 0x%x\n", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// Check WiFi connection status
static bool check_wifi_connection(void) {
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "✓ WiFi connected to: %s (RSSI: %d dBm)", ap_info.ssid, ap_info.rssi);
        
        // Get IP address
        esp_netif_ip_info_t ip_info;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            ESP_LOGI(TAG, "✓ IP Address: " IPSTR, IP2STR(&ip_info.ip));
            ESP_LOGI(TAG, "✓ Gateway: " IPSTR, IP2STR(&ip_info.gw));
            printf("[WiFi] Connected - IP: " IPSTR "\n", IP2STR(&ip_info.ip));
            return true;
        }
    }
    
    ESP_LOGE(TAG, "✗ WiFi not connected");
    printf("[WiFi] Not connected - check WiFi credentials\n");
    return false;
}

// Test MQTT broker connectivity before attempting to connect
static bool test_mqtt_broker_connectivity(const char* broker_url) {
    // Extract IP and port from broker URL
    char ip_str[32];
    int port = 1883;  // Default MQTT port
    
    // Simple URL parsing - expect format mqtt://ip:port or mqtt://ip
    const char* ip_start = strstr(broker_url, "://");
    if (ip_start == NULL) {
        ESP_LOGE(TAG, "Invalid broker URL format: %s", broker_url);
        return false;
    }
    ip_start += 3;  // Skip past "://"
    
    // Find port separator
    const char* port_start = strchr(ip_start, ':');
    if (port_start != NULL) {
        // Extract IP
        size_t ip_len = port_start - ip_start;
        if (ip_len >= sizeof(ip_str)) ip_len = sizeof(ip_str) - 1;
        strncpy(ip_str, ip_start, ip_len);
        ip_str[ip_len] = '\0';
        
        // Extract port
        port = atoi(port_start + 1);
    } else {
        // No port specified, copy IP and use default port
        strncpy(ip_str, ip_start, sizeof(ip_str) - 1);
        ip_str[sizeof(ip_str) - 1] = '\0';
        
        // Remove any trailing slash or path
        char* slash = strchr(ip_str, '/');
        if (slash) *slash = '\0';
    }
    
    ESP_LOGI(TAG, "Testing connectivity to MQTT broker: %s:%d", ip_str, port);
    
    // Create a socket for testing
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket for connectivity test");
        return false;
    }
    
    // Set socket timeout
    struct timeval timeout;
    timeout.tv_sec = 5;  // 5 second timeout
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    
    // Setup address
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(ip_str);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    
    // Test connection
    int connect_result = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    close(sock);
    
    if (connect_result == 0) {
        ESP_LOGI(TAG, "✓ Connectivity test successful - broker is reachable");
        return true;
    } else {
        ESP_LOGE(TAG, "✗ Connectivity test failed - broker not reachable (error: %d, errno: %d)", connect_result, errno);
        printf("[MQTT] Broker connectivity test failed!\n");
        printf("[MQTT] Broker: %s:%d\n", ip_str, port);
        printf("[MQTT] Check your broker configuration:\n");
        printf("  1. Verify broker IP address is correct\n");
        printf("  2. Ensure broker is running and accessible\n");
        printf("  3. Check network connectivity\n");
        printf("  4. Verify port number (default MQTT: 1883)\n");
        return false;
    }
}

// MQTT Application Start
static void mqtt_app_start(void) {
    ESP_LOGI(TAG, "Starting MQTT client initialization...");
    
    // Check WiFi connection first
    if (!check_wifi_connection()) {
        ESP_LOGE(TAG, "WiFi not connected - cannot start MQTT client");
        printf("[MQTT] WiFi connection required before MQTT\n");
        return;
    }
    
    // Test broker connectivity
    if (!test_mqtt_broker_connectivity(mqtt_broker_url)) {
        ESP_LOGE(TAG, "MQTT broker connectivity test failed - skipping MQTT client start");
        printf("[MQTT] Broker not reachable - check configuration\n");
        return;
    }
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = mqtt_broker_url,
        .network.timeout_ms = 10000,           // 10 second timeout
        .network.refresh_connection_after_ms = 20000,  // Refresh connection every 20s
        .session.keepalive = 60,               // 60 second keepalive
        .network.reconnect_timeout_ms = 10000, // 10 second reconnect timeout
        .buffer.size = 1024,                   // 1KB buffer
        .buffer.out_size = 1024,               // 1KB output buffer
    };
    // The event_handle field was removed from esp_mqtt_client_config_t in ESP-IDF v5.0.
    // Events are now registered globally for the client instance.
    // For older IDF: .event_handle = mqtt_event_handler,

    ESP_LOGI(TAG, "Initializing MQTT client with broker: %s", mqtt_broker_url);
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }
    
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    esp_err_t start_result = esp_mqtt_client_start(mqtt_client);
    if (start_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(start_result));
        return;
    }
    
    ESP_LOGI(TAG, "MQTT client started successfully.");
}

// Helper functions for processor metrics
static int32_t get_wifi_rssi_percent(void) {
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret == ESP_OK) {
        int32_t rssi_dbm = ap_info.rssi;
        // Convert RSSI from dBm to percentage
        // Typical range: -100 dBm (0%) to -50 dBm (100%)
        int32_t rssi_percent;
        if (rssi_dbm >= -50) {
            rssi_percent = 100;
        } else if (rssi_dbm <= -100) {
            rssi_percent = 0;
        } else {
            // Linear mapping: -100 dBm = 0%, -50 dBm = 100%
            rssi_percent = 2 * (rssi_dbm + 100);
        }
        return rssi_percent;
    }
    return 0; // Error value - 0% signal
}

static char* get_ip_address(void) {
    static char ip_str[16];
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) {
        esp_netif_ip_info_t ip_info;
        esp_err_t ret = esp_netif_get_ip_info(netif, &ip_info);
        if (ret == ESP_OK) {
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
            return ip_str;
        }
    }
    return "0.0.0.0";
}

static float get_cpu_temperature(void) {
    // ESP32 temperature reading is complex and requires additional configuration
    // For now, return a placeholder that could be enhanced later
    // Alternative: Use ADC reading from internal temperature sensor or external sensor
    return 25.0 + (esp_random() % 20); // Simulated temperature 25-45°C for demonstration
}

static char* get_software_version(void) {
    static char version_str[32];
    static bool version_loaded = false;
    
    if (!version_loaded) {
        // Use the CMake-generated version from project_version.h
        ESP_LOGI(TAG, "Using CMake-generated version: %s", PROJECT_VERSION);
        strncpy(version_str, PROJECT_VERSION, sizeof(version_str) - 1);
        version_str[sizeof(version_str) - 1] = '\0';
        version_loaded = true;
    }
    return version_str;
}

static char* get_chip_id(void) {
    static char chip_id_str[13]; // 6 bytes * 2 hex chars + null terminator
    static bool chip_id_loaded = false;
    
    if (!chip_id_loaded) {
        uint8_t mac[6];
        esp_err_t ret = esp_efuse_mac_get_default(mac);
        if (ret == ESP_OK) {
            // Convert MAC address (last 6 bytes) to hex string
            snprintf(chip_id_str, sizeof(chip_id_str), "%02X%02X%02X%02X%02X%02X", 
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        } else {
            snprintf(chip_id_str, sizeof(chip_id_str), "UNKNOWN");
            ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(ret));
        }
        chip_id_loaded = true;
        ESP_LOGI(TAG, "ESP32 Chip ID: %s", chip_id_str);
    }
    return chip_id_str;
}

// Function to publish sensor data to MQTT broker
// This function will be expanded to create and send the two JSON messages
void publish_sensor_data_mqtt(const sensor_data_t *sensor_data_ptr) {
    if (debug_logging) printf("[DEBUG] publish_sensor_data_mqtt() called\n");
    
    if (!mqtt_client) {
        ESP_LOGE(TAG, "MQTT client not initialized!");
        if (debug_logging) printf("[DEBUG] MQTT client not initialized - returning\n");
        return;
    }

    ESP_LOGI(TAG, "Preparing to publish sensor data via MQTT...");

    // TEMPLATE: Create JSON with your specific data structure
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE(TAG, "Failed to create cJSON root object.");
        return;
    }

    // Add processor metrics data first (system information)
    cJSON *processor_root = cJSON_CreateObject();
    if (processor_root) {
        // WiFi RSSI as percentage
        int32_t wifi_rssi_percent = get_wifi_rssi_percent();
        cJSON_AddNumberToObject(processor_root, "WiFiRSSI", wifi_rssi_percent);
        
        // IP Address
        char *ip_address = get_ip_address();
        cJSON_AddStringToObject(processor_root, "IPAddress", ip_address);
        
        // CPU Temperature
        float cpu_temp = get_cpu_temperature();
        cJSON_AddNumberToObject(processor_root, "CPUTemperature", cpu_temp);
        
        // Software Version
        char *software_version = get_software_version();
        cJSON_AddStringToObject(processor_root, "SoftwareVersion", software_version);
        
        // ESP32 Chip ID
        char *chip_id = get_chip_id();
        cJSON_AddStringToObject(processor_root, "ChipID", chip_id);
        
        // Watchdog Restart Count
        cJSON_AddNumberToObject(processor_root, "WDTRestartCount", watchdog_reset_counter);
        
        cJSON_AddItemToObject(root, "processor", processor_root);
    }

    // TEMPLATE: Add your specific sensor data formatting here
    // Replace with your sensor-specific data fields
    cJSON *data_root = cJSON_CreateObject();
    if (data_root) {
        // TEMPLATE: Add your sensor data fields here
        // Examples (customize for your sensors):
        cJSON_AddNumberToObject(data_root, "sensor_value_1", sensor_data_ptr->sensor_value_1);
        cJSON_AddNumberToObject(data_root, "sensor_value_2", sensor_data_ptr->sensor_value_2);
        cJSON_AddBoolToObject(data_root, "sensor_ok", sensor_data_ptr->sensor_ok);
        
        // TEMPLATE: Add more fields as needed for your specific sensors
        // Examples:
        // cJSON_AddNumberToObject(data_root, "temperature", sensor_data_ptr->temperature);
        // cJSON_AddNumberToObject(data_root, "humidity", sensor_data_ptr->humidity);
        // cJSON_AddNumberToObject(data_root, "pressure", sensor_data_ptr->pressure);
        // cJSON_AddBoolToObject(data_root, "sensor_1_ok", sensor_data_ptr->sensor_1_ok);
        
        cJSON_AddItemToObject(root, "data", data_root);
    }

    // Publish as a single topic
    char *json_string = cJSON_PrintUnformatted(root);
    if (json_string == NULL) {
        ESP_LOGE(TAG, "Failed to print combined cJSON to string.");
    } else {
        char topic[64];
        // Use data_topic directly (you may want to rename this variable for your use case)
        snprintf(topic, sizeof(topic), "%s", data_topic);
        if (debug_logging) printf("[DEBUG] About to publish to topic: %s\n", topic);
        if (debug_logging) printf("[DEBUG] JSON payload length: %d\n", strlen(json_string));
        esp_mqtt_client_publish(mqtt_client, topic, json_string, 0, 1, 0);
        ESP_LOGI(TAG, "Published to %s (length: %d)", topic, strlen(json_string));
        free(json_string);
    }
    cJSON_Delete(root);
}

// Helper to convert hex char to int
int hex2int(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

// URL decode utility (in-place)
void url_decode(char *dst, const char *src, size_t dstsize) {
    char a, b;
    size_t i = 0, j = 0;
    while (src[i] && j + 1 < dstsize) {
        if (src[i] == '%' && src[i+1] && src[i+2]) {
            if ((a = src[i+1]) && (b = src[i+2])) {
                dst[j++] = hex2int(a) * 16 + hex2int(b);
                i += 3;
            } else {
                dst[j++] = src[i++];
            }
        } else if (src[i] == '+') {
            dst[j++] = ' ';
            i++;
        } else {
            dst[j++] = src[i++];
        }
    }
    dst[j] = '\0';
}

// DNS hijack task for captive portal
void dns_hijack_task(void *pvParameter) {
    // Remove from watchdog to prevent conflicts during configuration mode
    // esp_task_wdt_add(NULL);
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create DNS socket: errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }
    
    // Set socket options for better reliability
    int reuse = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        ESP_LOGW(TAG, "Failed to set SO_REUSEADDR: errno=%d", errno);
    }
    
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(53);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind DNS socket: errno=%d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "DNS hijack server started on UDP port 53");
    
    uint8_t buf[512];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    uint32_t dns_requests_handled = 0;
    
    // Set a more reasonable timeout
    struct timeval timeout;
    timeout.tv_sec = 5;  // Increased to 5 seconds
    timeout.tv_usec = 0;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        ESP_LOGW(TAG, "Failed to set socket timeout: errno=%d", errno);
    }
    
    while (!system_shutting_down) {  // Check shutdown flag
        int len = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&client_addr, &addr_len);
        
        if (len > 0 && len >= 12) {  // Valid DNS query minimum size
            dns_requests_handled++;
            ESP_LOGD(TAG, "DNS request #%lu received (%d bytes)", dns_requests_handled, len);
            
            // Minimal DNS response: copy ID, set response flags, answer count = 1
            uint8_t response[512];
            memcpy(response, buf, len);
            response[2] = 0x81; // QR=1, Opcode=0, AA=1, TC=0, RD=0
            response[3] = 0x80; // RA=1, Z=0, RCODE=0
            response[7] = 1;    // ANCOUNT = 1
            
            // Copy question section after header (12 bytes)
            int qlen = len - 12;
            if (qlen > 0 && qlen < 400) {  // Sanity check
                memcpy(response + 12, buf + 12, qlen);
                int pos = 12 + qlen;
                
                // Answer: pointer to name (0xC00C), type A, class IN, TTL, RDLENGTH, RDATA
                response[pos++] = 0xC0; response[pos++] = 0x0C;
                response[pos++] = 0x00; response[pos++] = 0x01; // Type A
                response[pos++] = 0x00; response[pos++] = 0x01; // Class IN
                response[pos++] = 0x00; response[pos++] = 0x00; response[pos++] = 0x00; response[pos++] = 0x3C; // TTL 60s
                response[pos++] = 0x00; response[pos++] = 0x04; // RDLENGTH 4
                response[pos++] = 192;  response[pos++] = 168;  response[pos++] = 4; response[pos++] = 1; // 192.168.4.1
                
                int sent = sendto(sock, response, pos, 0, (struct sockaddr *)&client_addr, addr_len);
                if (sent < 0) {
                    ESP_LOGW(TAG, "Failed to send DNS response: errno=%d", errno);
                } else {
                    ESP_LOGD(TAG, "DNS response sent (%d bytes)", sent);
                }
            } else {
                ESP_LOGW(TAG, "Invalid DNS query length: %d", qlen);
            }
        } else if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Timeout - this is expected, continue
                ESP_LOGV(TAG, "DNS socket timeout (expected)");
            } else {
                ESP_LOGE(TAG, "DNS socket error: errno=%d", errno);
                break;
            }
        }
        
        // Yield to other tasks periodically
        if (dns_requests_handled % 10 == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    ESP_LOGI(TAG, "DNS hijack task shutting down gracefully (%lu requests handled)", dns_requests_handled);
    close(sock);
    vTaskDelete(NULL);
}

// Test function to manually trigger watchdog reset (for testing only)
void test_watchdog_reset() {
    ESP_LOGI(TAG, "=== TESTING WATCHDOG RESET ===");
    ESP_LOGI(TAG, "Current watchdog_reset_counter: %lu", watchdog_reset_counter);
    
    // Increment watchdog reset counter and save to NVS
    watchdog_reset_counter++;
    ESP_LOGI(TAG, "Incrementing watchdog reset counter to: %lu", watchdog_reset_counter);
    save_watchdog_counter_to_nvs();
    
    ESP_LOGI(TAG, "Test watchdog reset complete. Counter saved to NVS.");
}