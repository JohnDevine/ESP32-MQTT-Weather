/*
 * SN-3000-FSJT-N01 Wind Speed Sensor Driver
 * RS485/Modbus RTU Implementation for ESP32
 * 
 * This driver implements communication with the SN-3000-FSJT-N01 wind speed sensor
 * using Modbus RTU protocol over RS485 interface. It provides rolling average
 * and gust calculations over configurable time periods.
 */

#include "sensor_config.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>
#include <inttypes.h>

static const char *TAG = "WIND_SPEED";

// Hardware configuration
#define WIND_UART_NUM           UART_NUM_2
#define WIND_UART_BAUD_RATE     4800
#define WIND_UART_DATA_BITS     UART_DATA_8_BITS
#define WIND_UART_PARITY        UART_PARITY_DISABLE
#define WIND_UART_STOP_BITS     UART_STOP_BITS_1
#define WIND_UART_FLOW_CTRL     UART_HW_FLOWCTRL_DISABLE

#define WIND_GPIO_TX            GPIO_NUM_17
#define WIND_GPIO_RX            GPIO_NUM_16
#define WIND_GPIO_DE_RE         GPIO_NUM_4

#define WIND_UART_BUF_SIZE      256
#define WIND_RESPONSE_TIMEOUT   1000  // ms

// Modbus configuration
#define WIND_SLAVE_ADDRESS      0x01
#define WIND_FUNCTION_CODE      0x03
#define WIND_REGISTER_ADDRESS   0x0000
#define WIND_REGISTER_COUNT     0x0001

// Rolling buffer configuration
#define MAX_BUFFER_SIZE         300   // Maximum readings (10 minutes at 2-second intervals)

// Default configuration
#define DEFAULT_READING_INTERVAL_MS     2000    // 2 seconds
#define DEFAULT_AVG_PERIOD_MS          60000    // 1 minute
#define DEFAULT_GUST_PERIOD_MS        300000    // 5 minutes

// Data structures
typedef struct {
    float *buffer;
    int size;
    int head;
    int count;
    uint64_t *timestamps;  // Store timestamps for each reading
} rolling_buffer_t;

typedef struct {
    uint32_t reading_interval_ms;
    uint32_t avg_period_ms;
    uint32_t gust_period_ms;
    rolling_buffer_t avg_buffer;
    rolling_buffer_t gust_buffer;
    float current_speed;
    float current_avg;
    float current_gust;
    esp_timer_handle_t reading_timer;
    bool initialized;
} wind_sensor_t;

// Global wind sensor instance
static wind_sensor_t wind_sensor = {0};

// Function prototypes
static esp_err_t wind_uart_init(void);
static esp_err_t wind_modbus_read_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t *value);
static uint16_t wind_calculate_crc(uint8_t *data, int length);
static esp_err_t wind_init_rolling_buffer(rolling_buffer_t *buffer, int max_size);
static void wind_free_rolling_buffer(rolling_buffer_t *buffer);
static void wind_add_to_buffer(rolling_buffer_t *buffer, float value, uint64_t timestamp, uint32_t max_age_ms);
static float wind_calculate_average(rolling_buffer_t *buffer, uint64_t current_time, uint32_t max_age_ms);
static float wind_calculate_maximum(rolling_buffer_t *buffer, uint64_t current_time, uint32_t max_age_ms);
static void wind_reading_timer_callback(void *arg);

static esp_err_t wind_uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = WIND_UART_BAUD_RATE,
        .data_bits = WIND_UART_DATA_BITS,
        .parity = WIND_UART_PARITY,
        .stop_bits = WIND_UART_STOP_BITS,
        .flow_ctrl = WIND_UART_FLOW_CTRL,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(WIND_UART_NUM, WIND_UART_BUF_SIZE, WIND_UART_BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(WIND_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(WIND_UART_NUM, WIND_GPIO_TX, WIND_GPIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure DE/RE pin for RS485 direction control
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << WIND_GPIO_DE_RE);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DE/RE GPIO: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set to receive mode initially
    gpio_set_level(WIND_GPIO_DE_RE, 0);

    ESP_LOGI(TAG, "UART initialized for wind sensor (4800 baud, 8N1)");
    return ESP_OK;
}

static uint16_t wind_calculate_crc(uint8_t *data, int length) {
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static esp_err_t wind_modbus_read_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t *value) {
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }

    // Build Modbus RTU request
    uint8_t request[8];
    request[0] = slave_addr;
    request[1] = WIND_FUNCTION_CODE;
    request[2] = (reg_addr >> 8) & 0xFF;
    request[3] = reg_addr & 0xFF;
    request[4] = (WIND_REGISTER_COUNT >> 8) & 0xFF;
    request[5] = WIND_REGISTER_COUNT & 0xFF;
    
    uint16_t crc = wind_calculate_crc(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    // Clear UART buffer
    uart_flush(WIND_UART_NUM);

    // Switch to transmit mode
    gpio_set_level(WIND_GPIO_DE_RE, 1);
    vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for line settling

    // Send request
    int sent = uart_write_bytes(WIND_UART_NUM, request, sizeof(request));
    if (sent != sizeof(request)) {
        ESP_LOGE(TAG, "Failed to send complete Modbus request");
        gpio_set_level(WIND_GPIO_DE_RE, 0);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Wait for transmission to complete
    uart_wait_tx_done(WIND_UART_NUM, pdMS_TO_TICKS(100));
    
    // Switch to receive mode
    gpio_set_level(WIND_GPIO_DE_RE, 0);

    // Read response
    uint8_t response[7]; // Expected: addr + func + byte_count + data[2] + crc[2]
    int bytes_read = uart_read_bytes(WIND_UART_NUM, response, sizeof(response), pdMS_TO_TICKS(WIND_RESPONSE_TIMEOUT));
    
    if (bytes_read != sizeof(response)) {
        ESP_LOGE(TAG, "Invalid response length: expected %d, got %d", sizeof(response), bytes_read);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Validate response
    if (response[0] != slave_addr || response[1] != WIND_FUNCTION_CODE || response[2] != 2) {
        ESP_LOGE(TAG, "Invalid response header: addr=0x%02X, func=0x%02X, count=%d", 
                 response[0], response[1], response[2]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Verify CRC
    uint16_t received_crc = (response[6] << 8) | response[5];
    uint16_t calculated_crc = wind_calculate_crc(response, 5);
    if (received_crc != calculated_crc) {
        ESP_LOGE(TAG, "CRC mismatch: received=0x%04X, calculated=0x%04X", received_crc, calculated_crc);
        return ESP_ERR_INVALID_CRC;
    }

    // Extract data (big-endian)
    *value = (response[3] << 8) | response[4];
    
    ESP_LOGD(TAG, "Modbus read successful: register=0x%04X, value=0x%04X (%u)", reg_addr, *value, *value);
    return ESP_OK;
}

static esp_err_t wind_init_rolling_buffer(rolling_buffer_t *buffer, int max_size) {
    if (!buffer || max_size <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    buffer->buffer = (float*)calloc(max_size, sizeof(float));
    buffer->timestamps = (uint64_t*)calloc(max_size, sizeof(uint64_t));
    
    if (!buffer->buffer || !buffer->timestamps) {
        if (buffer->buffer) free(buffer->buffer);
        if (buffer->timestamps) free(buffer->timestamps);
        return ESP_ERR_NO_MEM;
    }

    buffer->size = max_size;
    buffer->head = 0;
    buffer->count = 0;

    return ESP_OK;
}

static void wind_free_rolling_buffer(rolling_buffer_t *buffer) {
    if (buffer) {
        if (buffer->buffer) {
            free(buffer->buffer);
            buffer->buffer = NULL;
        }
        if (buffer->timestamps) {
            free(buffer->timestamps);
            buffer->timestamps = NULL;
        }
        buffer->size = 0;
        buffer->head = 0;
        buffer->count = 0;
    }
}

static void wind_add_to_buffer(rolling_buffer_t *buffer, float value, uint64_t timestamp, uint32_t max_age_ms) {
    if (!buffer || !buffer->buffer || !buffer->timestamps) {
        return;
    }

    // Add new value
    buffer->buffer[buffer->head] = value;
    buffer->timestamps[buffer->head] = timestamp;
    buffer->head = (buffer->head + 1) % buffer->size;
    
    if (buffer->count < buffer->size) {
        buffer->count++;
    }
}

static float wind_calculate_average(rolling_buffer_t *buffer, uint64_t current_time, uint32_t max_age_ms) {
    if (!buffer || !buffer->buffer || !buffer->timestamps || buffer->count == 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    int valid_count = 0;
    uint64_t min_time = current_time - (max_age_ms * 1000ULL); // Convert to microseconds

    for (int i = 0; i < buffer->count; i++) {
        if (buffer->timestamps[i] >= min_time) {
            sum += buffer->buffer[i];
            valid_count++;
        }
    }

    return valid_count > 0 ? sum / valid_count : 0.0f;
}

static float wind_calculate_maximum(rolling_buffer_t *buffer, uint64_t current_time, uint32_t max_age_ms) {
    if (!buffer || !buffer->buffer || !buffer->timestamps || buffer->count == 0) {
        return 0.0f;
    }

    float max_value = 0.0f;
    bool found_valid = false;
    uint64_t min_time = current_time - (max_age_ms * 1000ULL); // Convert to microseconds

    for (int i = 0; i < buffer->count; i++) {
        if (buffer->timestamps[i] >= min_time) {
            if (!found_valid || buffer->buffer[i] > max_value) {
                max_value = buffer->buffer[i];
                found_valid = true;
            }
        }
    }

    return max_value;
}

static void wind_reading_timer_callback(void *arg) {
    uint16_t raw_value;
    esp_err_t ret = wind_modbus_read_register(WIND_SLAVE_ADDRESS, WIND_REGISTER_ADDRESS, &raw_value);
    
    if (ret == ESP_OK) {
        // Convert raw value to wind speed (divide by 10 as per datasheet)
        float wind_speed = raw_value / 10.0f;
        uint64_t timestamp = esp_timer_get_time();
        
        // Update current speed
        wind_sensor.current_speed = wind_speed;
        
        // Add to rolling buffers
        wind_add_to_buffer(&wind_sensor.avg_buffer, wind_speed, timestamp, wind_sensor.avg_period_ms);
        wind_add_to_buffer(&wind_sensor.gust_buffer, wind_speed, timestamp, wind_sensor.gust_period_ms);
        
        // Calculate rolling statistics
        wind_sensor.current_avg = wind_calculate_average(&wind_sensor.avg_buffer, timestamp, wind_sensor.avg_period_ms);
        wind_sensor.current_gust = wind_calculate_maximum(&wind_sensor.gust_buffer, timestamp, wind_sensor.gust_period_ms);
        
        ESP_LOGD(TAG, "Wind reading: raw=%u, speed=%.1f m/s, avg=%.1f m/s, gust=%.1f m/s", 
                 raw_value, wind_speed, wind_sensor.current_avg, wind_sensor.current_gust);
    } else {
        ESP_LOGW(TAG, "Failed to read wind sensor: %s", esp_err_to_name(ret));
    }
}

esp_err_t sensor_wind_speed_init(void) {
    if (wind_sensor.initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing SN-3000-FSJT-N01 wind speed sensor");

    // Initialize UART
    esp_err_t ret = wind_uart_init();
    if (ret != ESP_OK) {
        return ret;
    }

    // Set default configuration
    wind_sensor.reading_interval_ms = DEFAULT_READING_INTERVAL_MS;
    wind_sensor.avg_period_ms = DEFAULT_AVG_PERIOD_MS;
    wind_sensor.gust_period_ms = DEFAULT_GUST_PERIOD_MS;

    // Calculate buffer sizes
    int avg_buffer_size = (wind_sensor.avg_period_ms / wind_sensor.reading_interval_ms) + 10; // +10 for safety
    int gust_buffer_size = (wind_sensor.gust_period_ms / wind_sensor.reading_interval_ms) + 10;

    // Limit buffer sizes
    avg_buffer_size = avg_buffer_size > MAX_BUFFER_SIZE ? MAX_BUFFER_SIZE : avg_buffer_size;
    gust_buffer_size = gust_buffer_size > MAX_BUFFER_SIZE ? MAX_BUFFER_SIZE : gust_buffer_size;

    // Initialize rolling buffers
    ret = wind_init_rolling_buffer(&wind_sensor.avg_buffer, avg_buffer_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize average buffer");
        return ret;
    }

    ret = wind_init_rolling_buffer(&wind_sensor.gust_buffer, gust_buffer_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize gust buffer");
        wind_free_rolling_buffer(&wind_sensor.avg_buffer);
        return ret;
    }

    // Create and start periodic timer
    esp_timer_create_args_t timer_args = {
        .callback = wind_reading_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "wind_reading_timer",
        .skip_unhandled_events = true
    };

    ret = esp_timer_create(&timer_args, &wind_sensor.reading_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create reading timer");
        wind_free_rolling_buffer(&wind_sensor.avg_buffer);
        wind_free_rolling_buffer(&wind_sensor.gust_buffer);
        return ret;
    }

    ret = esp_timer_start_periodic(wind_sensor.reading_timer, wind_sensor.reading_interval_ms * 1000ULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start reading timer");
        esp_timer_delete(wind_sensor.reading_timer);
        wind_free_rolling_buffer(&wind_sensor.avg_buffer);
        wind_free_rolling_buffer(&wind_sensor.gust_buffer);
        return ret;
    }

    wind_sensor.initialized = true;
    ESP_LOGI(TAG, "Wind speed sensor initialized successfully");
    ESP_LOGI(TAG, "Configuration: reading_interval=%" PRIu32 "ms, avg_period=%" PRIu32 "ms, gust_period=%" PRIu32 "ms",
             wind_sensor.reading_interval_ms, wind_sensor.avg_period_ms, wind_sensor.gust_period_ms);
    ESP_LOGI(TAG, "Buffer sizes: avg=%d, gust=%d", avg_buffer_size, gust_buffer_size);

    return ESP_OK;
}

esp_err_t sensor_wind_speed_read(sensor_data_t* data) {
    if (!wind_sensor.initialized) {
        ESP_LOGE(TAG, "Wind speed sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize data structure
    memset(data, 0, sizeof(sensor_data_t));
    data->type = SENSOR_TYPE_WIND_SPEED;
    data->valid = false;

    // Test sensor communication with a single read
    uint16_t raw_value;
    esp_err_t ret = wind_modbus_read_register(WIND_SLAVE_ADDRESS, WIND_REGISTER_ADDRESS, &raw_value);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read wind sensor for validation: %s", esp_err_to_name(ret));
        return ret;
    }

    // Fill data structure with current values
    data->data.wind_speed.wind_speed = wind_sensor.current_speed;
    data->data.wind_speed.wind_speed_avg = wind_sensor.current_avg;
    data->data.wind_speed.wind_gust = wind_sensor.current_gust;
    data->data.wind_speed.raw_reading = raw_value;
    data->valid = true;

    ESP_LOGI(TAG, "Wind speed reading: speed=%.1f m/s, avg=%.1f m/s, gust=%.1f m/s (raw=%u)",
             data->data.wind_speed.wind_speed, data->data.wind_speed.wind_speed_avg, 
             data->data.wind_speed.wind_gust, raw_value);

    return ESP_OK;
}

bool wind_speed_is_initialized(void) {
    return wind_sensor.initialized;
}

esp_err_t sensor_wind_speed_set_config(uint32_t reading_interval_ms, uint32_t avg_period_ms, uint32_t gust_period_ms) {
    if (reading_interval_ms < 100 || reading_interval_ms > 60000) {
        ESP_LOGE(TAG, "Invalid reading interval: %" PRIu32 " ms (valid range: 100-60000)", reading_interval_ms);
        return ESP_ERR_INVALID_ARG;
    }

    if (avg_period_ms < reading_interval_ms || avg_period_ms > 600000) {
        ESP_LOGE(TAG, "Invalid average period: %" PRIu32 " ms (valid range: %" PRIu32 "-600000)", avg_period_ms, reading_interval_ms);
        return ESP_ERR_INVALID_ARG;
    }

    if (gust_period_ms < reading_interval_ms || gust_period_ms > 1800000) {
        ESP_LOGE(TAG, "Invalid gust period: %" PRIu32 " ms (valid range: %" PRIu32 "-1800000)", gust_period_ms, reading_interval_ms);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Updating wind sensor configuration: reading=%" PRIu32 "ms, avg=%" PRIu32 "ms, gust=%" PRIu32 "ms",
             reading_interval_ms, avg_period_ms, gust_period_ms);

    wind_sensor.reading_interval_ms = reading_interval_ms;
    wind_sensor.avg_period_ms = avg_period_ms;
    wind_sensor.gust_period_ms = gust_period_ms;

    // If initialized, restart the timer with new interval
    if (wind_sensor.initialized && wind_sensor.reading_timer) {
        esp_timer_stop(wind_sensor.reading_timer);
        esp_err_t ret = esp_timer_start_periodic(wind_sensor.reading_timer, reading_interval_ms * 1000ULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restart timer with new interval");
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t sensor_wind_speed_get_config(uint32_t* reading_interval_ms, uint32_t* avg_period_ms, uint32_t* gust_period_ms) {
    if (!reading_interval_ms || !avg_period_ms || !gust_period_ms) {
        return ESP_ERR_INVALID_ARG;
    }

    *reading_interval_ms = wind_sensor.reading_interval_ms;
    *avg_period_ms = wind_sensor.avg_period_ms;
    *gust_period_ms = wind_sensor.gust_period_ms;

    return ESP_OK;
}
