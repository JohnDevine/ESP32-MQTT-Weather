 * I2C Bus Scanner - Header File
 */
/**
 * @file i2c_scanner.h
 * @brief I2C Bus Scanner header for ESP32-MQTT-Weather
 * @uml{component: I2C Scanner}
 *
 * Declares functions for scanning I2C bus and identifying devices.
 *
 * @section ToDo
 * - Add support for more detailed device identification
 */

#ifndef I2C_SCANNER_H
#define I2C_SCANNER_H

#include <stdint.h>

/**
 * Scan the I2C bus and report all responding devices
 */
void i2c_scanner_scan_bus(void);

/**
 * Perform detailed analysis of a specific I2C device
 * @param address I2C address to analyze (7-bit)
 */
void i2c_scanner_identify_chip_id(uint8_t address);

#endif // I2C_SCANNER_H
