#include "i2c_sync.h"

// Statically allocated buffer for the mutex's control block
static StaticSemaphore_t xI2cMutexBuffer;

// Define a mutex for protecting I2C bus access
static SemaphoreHandle_t i2c_bus_mutex = NULL;

// Global variable to store the handle for the *currently active* transaction.
static i2c_sync_handle_t *g_active_i2c_sync_handle = NULL;

void i2c_sync_event_callback(uint32_t event) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (g_active_i2c_sync_handle != NULL && g_active_i2c_sync_handle->i2c_semaphore != NULL) {
        g_active_i2c_sync_handle->i2c_transfer_status = event;
        xSemaphoreGiveFromISR(g_active_i2c_sync_handle->i2c_semaphore, &xHigherPriorityTaskWoken);
        g_active_i2c_sync_handle = NULL;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void i2c_sync_global_init(void) {
    if (i2c_bus_mutex == NULL) {
        i2c_bus_mutex = xSemaphoreCreateMutexStatic(&xI2cMutexBuffer);
    }
}

int32_t i2c_sync_write_byte(i2c_sync_handle_t *handle, uint8_t reg_addr, uint8_t data) {
    uint8_t tx_buffer[2];
    tx_buffer[0] = reg_addr;
    tx_buffer[1] = data;
    int32_t ret = -1;

    if (handle->i2c_driver == NULL || handle->i2c_semaphore == NULL || i2c_bus_mutex == NULL) {
        return -1;
    }

    if (xSemaphoreTake(i2c_bus_mutex, portMAX_DELAY) == pdFALSE) {
        return -1;
    }

    g_active_i2c_sync_handle = handle;
    xSemaphoreTake(handle->i2c_semaphore, 0);

    // Perform Transfer
    int32_t status = handle->i2c_driver->MasterTransmit(handle->i2c_addr, tx_buffer, 2, false);

    // Evaluate Result
    if (status == ARM_DRIVER_OK) {
        if (xSemaphoreTake(handle->i2c_semaphore, I2C_TIMEOUT_TICKS) == pdTRUE) {
            // Check for bus errors in the status reported by callback
            if ((handle->i2c_transfer_status & (ARM_I2C_EVENT_ARBITRATION_LOST | ARM_I2C_EVENT_BUS_ERROR | ARM_I2C_EVENT_ADDRESS_NACK)) == 0) {
                ret = 0; // Success
            }
        }
    }

    // Cleanup
    if (g_active_i2c_sync_handle == handle) {
        g_active_i2c_sync_handle = NULL;
    }
    xSemaphoreGive(i2c_bus_mutex);

    return ret;
}

int32_t i2c_sync_read_bytes(i2c_sync_handle_t *handle, uint8_t reg_addr, uint8_t *data, uint32_t len) {
    int32_t ret = -1;

    if (handle->i2c_driver == NULL || handle->i2c_semaphore == NULL || i2c_bus_mutex == NULL) {
        return -1;
    }

    if (xSemaphoreTake(i2c_bus_mutex, I2C_TIMEOUT_TICKS) == pdFALSE) {
        return -1;
    }

    // --- Step 1: Write Register Address ---
    g_active_i2c_sync_handle = handle;
    xSemaphoreTake(handle->i2c_semaphore, 0);

    int32_t status = handle->i2c_driver->MasterTransmit(handle->i2c_addr, &reg_addr, 1, true);
    bool step1Success = false;

    if (status == ARM_DRIVER_OK) {
        if (xSemaphoreTake(handle->i2c_semaphore, I2C_TIMEOUT_TICKS) == pdTRUE) {
            if ((handle->i2c_transfer_status & (ARM_I2C_EVENT_ARBITRATION_LOST | ARM_I2C_EVENT_BUS_ERROR | ARM_I2C_EVENT_ADDRESS_NACK)) == 0) {
                step1Success = true;
            }
        }
    }

    // --- Step 2: Read Data ---
    if (step1Success) {
        // Ensure handle is set again (safety, though mutex holds it)
        g_active_i2c_sync_handle = handle;
        xSemaphoreTake(handle->i2c_semaphore, 0);

        status = handle->i2c_driver->MasterReceive(handle->i2c_addr, data, len, false);

        if (status == ARM_DRIVER_OK) {
             if (xSemaphoreTake(handle->i2c_semaphore, I2C_TIMEOUT_TICKS) == pdTRUE) {
                if ((handle->i2c_transfer_status & (ARM_I2C_EVENT_ARBITRATION_LOST | ARM_I2C_EVENT_BUS_ERROR)) == 0) {
                    ret = 0; // Success
                }
             }
        }
    }

    // Cleanup
    if (g_active_i2c_sync_handle == handle) {
        g_active_i2c_sync_handle = NULL;
    }
    xSemaphoreGive(i2c_bus_mutex);

    return ret;
}

int32_t i2c_sync_read_byte(i2c_sync_handle_t *handle, uint8_t reg_addr, uint8_t *data) {
    return i2c_sync_read_bytes(handle, reg_addr, data, 1);
}
