/*
 * I2C Bus Scanner - Header File
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
