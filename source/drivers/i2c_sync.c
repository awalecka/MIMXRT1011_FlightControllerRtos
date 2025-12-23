#include "i2c_sync.h"

// Statically allocated buffer for the mutex's control block
static StaticSemaphore_t xI2cMutexBuffer;

// Define a mutex for protecting I2C bus access
static SemaphoreHandle_t i2c_bus_mutex = NULL;

// Global variable to store the handle for the *currently active* transaction.
// This is still global but will be protected by the mutex.
// This is a common pattern when the underlying driver callback doesn't support user_data.
static i2c_sync_handle_t *g_active_i2c_sync_handle = NULL;


/**
 * @brief Common CMSIS I2C Event Callback Function.
 * This function is called by the CMSIS I2C driver when a transfer completes or an error occurs.
 * It uses the g_active_i2c_sync_handle to release the semaphore.
 * @param[in] event Event flags indicating the reason for the callback.
 */
void i2c_sync_event_callback(uint32_t event) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Use g_active_i2c_sync_handle which was set by the task before initiating the transfer.
    // We expect it to be non-NULL here because the mutex should prevent race conditions.
    if (g_active_i2c_sync_handle == NULL || g_active_i2c_sync_handle->i2c_semaphore == NULL) {
        // This case should ideally not happen with the mutex protection.
        // Log an error if possible, or assert.
        return;
    }

    // Set the transfer status based on the event
    g_active_i2c_sync_handle->i2c_transfer_status = event;

    // Release the semaphore to unblock the task waiting for I2C completion
    xSemaphoreGiveFromISR(g_active_i2c_sync_handle->i2c_semaphore, &xHigherPriorityTaskWoken);

    // Clear the active handle now that the transaction is complete and semaphore given.
    // This is crucial for reentrancy with the mutex.
    g_active_i2c_sync_handle = NULL;

    // If a higher priority task was unblocked, yield from ISR
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Initializes the I2C synchronization mechanism (creates mutex).
 * This function should be called once at system startup.
 */
void i2c_sync_global_init(void) {
    if (i2c_bus_mutex == NULL) {
        i2c_bus_mutex = xSemaphoreCreateMutexStatic(&xI2cMutexBuffer);
        // Handle error if mutex creation fails (e.g., out of heap)
    }
}


/**
 * @brief Writes a single byte to an I2C device register.
 * @param[in] handle Pointer to the generic I2C synchronization handle.
 * @param[in] reg_addr Register address to write to.
 * @param[in] data Byte to write.
 * @return 0 on success, -1 on I2C error or timeout.
 */
int32_t i2c_sync_write_byte(i2c_sync_handle_t *handle, uint8_t reg_addr, uint8_t data) {
    uint8_t tx_buffer[2];
    tx_buffer[0] = reg_addr;
    tx_buffer[1] = data;
    int32_t ret = -1;

    if (handle->i2c_driver == NULL || handle->i2c_semaphore == NULL || i2c_bus_mutex == NULL) {
        return -1; // Driver, semaphore, or mutex not ready
    }

    // Acquire the mutex to ensure exclusive access to the I2C bus
    if (xSemaphoreTake(i2c_bus_mutex, portMAX_DELAY) == pdFALSE) { // Use a timeout for mutex acquisition
        return -1; // Failed to acquire mutex
    }

    // Set the global active handle *after* acquiring the mutex and *before* starting the transfer
    g_active_i2c_sync_handle = handle;

    // Clear any pending semaphore tokens before starting a new transfer
    xSemaphoreTake(handle->i2c_semaphore, 0);

    // Master transmit, 2 bytes (register address + data), no restart
    // This function returns immediately in asynchronous mode (DMA-driven)
    int32_t status = handle->i2c_driver->MasterTransmit(handle->i2c_addr, tx_buffer, 2, false);
    if (status != ARM_DRIVER_OK) {
        goto exit_error;
    }

    // Wait for the transfer to complete (semaphore will be given in the callback)
    if (xSemaphoreTake(handle->i2c_semaphore, I2C_TIMEOUT_TICKS) == pdFALSE) {
        // Timeout occurred, or semaphore not given
        goto exit_error;
    }

    // Check status from the callback
    if (handle->i2c_transfer_status & (ARM_I2C_EVENT_ARBITRATION_LOST | ARM_I2C_EVENT_BUS_ERROR | ARM_I2C_EVENT_ADDRESS_NACK)) {
        goto exit_error; // Indicate I2C bus error or NACK
    }

    ret = 0; // Success

exit_error:
    // Important: Release the mutex after the transaction, whether it succeeded or failed.
    // If g_active_i2c_sync_handle was cleared by the callback, we don't need to clear it again.
    // If we exited due to a driver error *before* the callback, it must be cleared here.
    if (g_active_i2c_sync_handle == handle) { // Check if it's still our handle
        g_active_i2c_sync_handle = NULL;
    }
    xSemaphoreGive(i2c_bus_mutex);
    return ret;
}

/**
 * @brief Reads multiple bytes from an I2C device registers.
 * @param[in] handle Pointer to the generic I2C synchronization handle.
 * @param[in] reg_addr Starting register address.
 * @param[out] data Pointer to buffer to store read data.
 * @param[in] len Number of bytes to read.
 * @return 0 on success, -1 on I2C error or timeout.
 */
int32_t i2c_sync_read_bytes(i2c_sync_handle_t *handle, uint8_t reg_addr, uint8_t *data, uint32_t len) {
    int32_t ret = -1;

    if (handle->i2c_driver == NULL || handle->i2c_semaphore == NULL || i2c_bus_mutex == NULL) {
        return -1; // Driver, asemaphore, or mutex not ready
    }

    // Acquire the mutex
    if (xSemaphoreTake(i2c_bus_mutex, I2C_TIMEOUT_TICKS) == pdFALSE) {
        return -1; // Failed to acquire mutex
    }

    g_active_i2c_sync_handle = handle; // Set the global active handle

    // Clear any pending semaphore tokens
    xSemaphoreTake(handle->i2c_semaphore, 0);

    // Send register address (Master Transmit with restart)
    int32_t status = handle->i2c_driver->MasterTransmit(handle->i2c_addr, &reg_addr, 1, true); // True for restart
    if (status != ARM_DRIVER_OK) {
        goto exit_error;
    }

    if (xSemaphoreTake(handle->i2c_semaphore, I2C_TIMEOUT_TICKS) == pdFALSE) {
        goto exit_error;
    }
    if (handle->i2c_transfer_status & (ARM_I2C_EVENT_ARBITRATION_LOST | ARM_I2C_EVENT_BUS_ERROR | ARM_I2C_EVENT_ADDRESS_NACK)) {
        goto exit_error;
    }

    // Clear any pending semaphore tokens again for the next transaction
    xSemaphoreTake(handle->i2c_semaphore, 0);

    // Read data (Master Receive)
    // Set the global active handle *after* acquiring the mutex and *before* starting the transfer
    g_active_i2c_sync_handle = handle; // Set the global active handle

    status = handle->i2c_driver->MasterReceive(handle->i2c_addr, data, len, false); // False for no restart (last byte)
    if (status != ARM_DRIVER_OK) {
        goto exit_error;
    }

    if (xSemaphoreTake(handle->i2c_semaphore, I2C_TIMEOUT_TICKS) == pdFALSE) {
        goto exit_error;
    }
    if (handle->i2c_transfer_status & (ARM_I2C_EVENT_ARBITRATION_LOST | ARM_I2C_EVENT_BUS_ERROR)) { // No NACK expected on receive
        goto exit_error;
    }

    ret = 0; // Success

exit_error:
    if (g_active_i2c_sync_handle == handle) {
        g_active_i2c_sync_handle = NULL;
    }
    xSemaphoreGive(i2c_bus_mutex); // Release the mutex
    return ret;
}

/**
 * @brief Reads a single byte from an I2C device register.
 * This function reuses i2c_sync_read_bytes for simplicity.
 * @param[in] handle Pointer to the generic I2C synchronization handle.
 * @param[in] reg_addr Register address to read from.
 * @param[out] data Pointer to store the read byte.
 * @return 0 on success, -1 on I2C error or timeout.
 */
int32_t i2c_sync_read_byte(i2c_sync_handle_t *handle, uint8_t reg_addr, uint8_t *data) {
    return i2c_sync_read_bytes(handle, reg_addr, data, 1);
}
