#ifndef I2C_SYNC_H
#define I2C_SYNC_H

#include <stdint.h>
#include "Driver_I2C.h" // CMSIS I2C Driver API
#include "FreeRTOS.h"
#include "semphr.h" // For SemaphoreHandle_t

// Define a maximum timeout for I2C operations in FreeRTOS ticks
#define I2C_TIMEOUT_TICKS pdMS_TO_TICKS(100) // 100ms timeout

/**
 * @brief Generic I2C Synchronization Handle Structure.
 * This structure holds the necessary components for synchronizing I2C transfers
 * using FreeRTOS semaphores with CMSIS I2C drivers.
 */
typedef struct {
    ARM_DRIVER_I2C *i2c_driver;         // Pointer to the CMSIS I2C driver instance
    uint8_t         i2c_addr;           // I2C address of the sensor
    SemaphoreHandle_t i2c_semaphore;    // Semaphore for I2C transfer completion (FreeRTOS)
    volatile int32_t  i2c_transfer_status; // Status of the last I2C transfer
} i2c_sync_handle_t;

// The common CMSIS I2C event callback function.
// This callback MUST be able to identify which i2c_sync_handle_t it belongs to.
// With the current CMSIS driver callback signature, this is achieved by
// using a global pointer that is set by the task before initiating the transfer,
// and this global pointer is protected by a mutex for reentrancy.
void i2c_sync_event_callback(uint32_t event);

/**
 * @brief Initializes the global I2C synchronization mutex.
 * This function MUST be called once at system startup, before any I2C operations.
 */
void i2c_sync_global_init(void);

/**
 * @brief Writes a single byte to an I2C device register.
 * @param[in] handle Pointer to the generic I2C synchronization handle.
 * @param[in] reg_addr Register address to write to.
 * @param[in] data Byte to write.
 * @return 0 on success, -1 on I2C error or timeout.
 */
int32_t i2c_sync_write_byte(i2c_sync_handle_t *handle, uint8_t reg_addr, uint8_t data);

/**
 * @brief Reads multiple bytes from an I2C device registers.
 * @param[in] handle Pointer to the generic I2C synchronization handle.
 * @param[in] reg_addr Starting register address.
 * @param[out] data Pointer to buffer to store read data.
 * @param[in] len Number of bytes to read.
 * @return 0 on success, -1 on I2C error or timeout.
 */
int32_t i2c_sync_read_bytes(i2c_sync_handle_t *handle, uint8_t reg_addr, uint8_t *data, uint32_t len);

/**
 * @brief Reads a single byte from an I2C device register.
 * @param[in] handle Pointer to the generic I2C synchronization handle.
 * @param[in] reg_addr Register address to read from.
 * @param[out] data Pointer to store the read byte.
 * @return 0 on success, -1 on I2C error or timeout.
 */
int32_t i2c_sync_read_byte(i2c_sync_handle_t *handle, uint8_t reg_addr, uint8_t *data);

#endif // I2C_SYNC_H
