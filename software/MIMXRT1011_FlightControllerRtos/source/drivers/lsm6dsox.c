#include <lsm6dsox.h>
#include "i2c_sync.h" // Include common I2C synchronization functions

// Global pointer to the handle
static lsm6dsox_handle_t *g_lsm6dsox_handle = NULL;

// Statically allocated buffer for the semaphore's control block
static StaticSemaphore_t xLsm6dsoxSemaphoreBuffer;

// --- Public API Functions ---

int32_t LSM6DSOX_Init(lsm6dsox_handle_t *handle, ARM_DRIVER_I2C *i2c_driver, uint8_t i2c_addr) {
    if (handle == NULL || i2c_driver == NULL) {
        return -1; // Invalid parameters
    }

    // Initialize the embedded i2c_sync_handle_t
    handle->i2c_sync.i2c_driver = i2c_driver;
    handle->i2c_sync.i2c_addr   = i2c_addr;
    handle->i2c_sync.i2c_transfer_status = 0; // Initialize status

    // Assign the global sensor-specific handle for potential future use (e.g., if needed by external modules)
    g_lsm6dsox_handle = handle;

    handle->acc_fs     = LSM6DSOX_ACC_FS_2G;    // Default to 2g
    handle->gyro_fs    = LSM6DSOX_GYRO_FS_250DPS; // Default to 250 dps

    // Create the binary semaphore for I2C transfer completion
    handle->i2c_sync.i2c_semaphore = xSemaphoreCreateBinaryStatic(&xLsm6dsoxSemaphoreBuffer);
    if (handle->i2c_sync.i2c_semaphore == NULL) {
        return -1; // Failed to create semaphore
    }
    // Ensure the semaphore is initially empty
    xSemaphoreTake(handle->i2c_sync.i2c_semaphore, 0);

    // Initialize the I2C driver with the common event callback
    // The underlying CMSIS driver implementation for RT1011 should
    // be configured to use eDMA. This call enables the driver to
    // use its configured asynchronous mechanisms.
    if (handle->i2c_sync.i2c_driver->Initialize(i2c_sync_event_callback) != ARM_DRIVER_OK) {
        return -1;
    }
    if (handle->i2c_sync.i2c_driver->PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK) {
        return -1;
    }
    // Set bus speed (example: 100 kHz)
    if (handle->i2c_sync.i2c_driver->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST) != ARM_DRIVER_OK) {
        return -1;
    }

    // Verify sensor communication by reading WHO_AM_I
    int32_t who_am_i = LSM6DSOX_ReadID(handle);
    if (who_am_i != LSM6DSOX_WHO_AM_I_VAL) {
        // Sensor not found or incorrect ID
        return -1;
    }

    // Perform a soft reset (optional, good for robust initialization)
    // Read CTRL3_C, set BOOT bit, write back.
    // Datasheet recommends waiting for 200ms after soft-reset for boot up.
    uint8_t reg_val;
    if (i2c_sync_read_byte(&handle->i2c_sync, LSM6DSOX_REG_CTRL3_C, &reg_val) != 0) return -1;
    reg_val |= (1 << 7); // Set BOOT bit
    if (i2c_sync_write_byte(&handle->i2c_sync, LSM6DSOX_REG_CTRL3_C, reg_val) != 0) return -1;

    // A small delay might be needed here to ensure the sensor finishes booting after soft reset.
    vTaskDelay(pdMS_TO_TICKS(200)); // Delay for 200ms for sensor boot-up after soft reset

    // After soft reset, re-read WHO_AM_I if necessary, or proceed with default configuration.
    // For this example, we'll assume the sensor is ready.

    // Set default ODR and FS for both accel and gyro to power-down them initially
    // This is a good practice to start from a known state.
    if (LSM6DSOX_SetAccConfig(handle, LSM6DSOX_ACC_ODR_OFF, LSM6DSOX_ACC_FS_2G) != 0) return -1;
    if (LSM6DSOX_SetGyroConfig(handle, LSM6DSOX_GYRO_ODR_OFF, LSM6DSOX_GYRO_FS_250DPS) != 0) return -1;

    return 0; // Success
}

int32_t LSM6DSOX_ReadID(lsm6dsox_handle_t *handle) {
    uint8_t who_am_i_val;
    if (i2c_sync_read_byte(&handle->i2c_sync, LSM6DSOX_REG_WHO_AM_I, &who_am_i_val) != 0) {
        return -1; // I2C error
    }
    return (int32_t)who_am_i_val;
}

int32_t LSM6DSOX_SetAccConfig(lsm6dsox_handle_t *handle, lsm6dsox_acc_odr_t odr, lsm6dsox_acc_fs_t fs) {
    uint8_t ctrl1_xl_val = 0;

    // ODR_XL[3:0] bits 7:4
    ctrl1_xl_val |= ((uint8_t)odr << 4);

    // FS_XL[1:0] bits 3:2
    ctrl1_xl_val |= ((uint8_t)fs << 2);

    handle->acc_fs = fs; // Store the current full scale for later conversion

    return i2c_sync_write_byte(&handle->i2c_sync, LSM6DSOX_REG_CTRL1_XL, ctrl1_xl_val);
}

int32_t LSM6DSOX_SetGyroConfig(lsm6dsox_handle_t *handle, lsm6dsox_gyro_odr_t odr, lsm6dsox_gyro_fs_t fs) {
    uint8_t ctrl2_g_val = 0;

    // ODR_G[3:0] bits 7:4
    ctrl2_g_val |= ((uint8_t)odr << 4);

    // FS_G[1:0] bits 3:2
    ctrl2_g_val |= ((uint8_t)fs << 2);

    handle->gyro_fs = fs; // Store the current full scale for later conversion

    return i2c_sync_write_byte(&handle->i2c_sync, LSM6DSOX_REG_CTRL2_G, ctrl2_g_val);
}

int32_t LSM6DSOX_ReadAccRaw(lsm6dsox_handle_t *handle, lsm6dsox_3axis_raw_t *acc_raw) {
    uint8_t data[6];
    // Read 6 bytes starting from OUTX_L_A (0x28)
    // The datasheet implies that reading multiple bytes from a starting address will auto-increment.
    // Check LSM6DSOX datasheet: Section 6.2 "I2C interface" - Auto-increment of address.
    if (i2c_sync_read_bytes(&handle->i2c_sync, LSM6DSOX_REG_OUTX_L_A, data, 6) != 0) {
        return -1; // I2C error
    }

    // Data is 16-bit signed, LSB first (little endian)
    acc_raw->x = (int16_t)((data[1] << 8) | data[0]);
    acc_raw->y = (int16_t)((data[3] << 8) | data[2]);
    acc_raw->z = (int16_t)((data[5] << 8) | data[4]);

    return 0;
}

int32_t LSM6DSOX_ReadGyroRaw(lsm6dsox_handle_t *handle, lsm6dsox_3axis_raw_t *gyro_raw) {
    uint8_t data[6];
    // Read 6 bytes starting from OUTX_L_G (0x22)
    if (i2c_sync_read_bytes(&handle->i2c_sync, LSM6DSOX_REG_OUTX_L_G, data, 6) != 0) {
        return -1; // I2C error
    }

    // Data is 16-bit signed, LSB first (little endian)
    gyro_raw->x = (int16_t)((data[1] << 8) | data[0]);
    gyro_raw->y = (int16_t)((data[3] << 8) | data[2]);
    gyro_raw->z = (int16_t)((data[5] << 8) | data[4]);

    return 0;
}

int32_t LSM6DSOX_ReadAcc(lsm6dsox_handle_t *handle, lsm6dsox_3axis_data_t *acc_data) {
    lsm6dsox_3axis_raw_t acc_raw;
    if (LSM6DSOX_ReadAccRaw(handle, &acc_raw) != 0) {
        return -1;
    }

    float sensitivity = 0.0f;
    switch (handle->acc_fs) {
        case LSM6DSOX_ACC_FS_2G:    sensitivity = 0.061f; break; // mg/LSB
        case LSM6DSOX_ACC_FS_4G:    sensitivity = 0.122f; break;
        case LSM6DSOX_ACC_FS_8G:    sensitivity = 0.244f; break;
        case LSM6DSOX_ACC_FS_16G:   sensitivity = 0.488f; break;
        default:                    sensitivity = 0.061f; break; // Default to 2G if unknown
    }

    // Convert raw LSB to g (divide by 1000 since sensitivity is mg/LSB)
    acc_data->x = (float)acc_raw.x * (sensitivity / 1000.0f);
    acc_data->y = (float)acc_raw.y * (sensitivity / 1000.0f);
    acc_data->z = (float)acc_raw.z * (sensitivity / 1000.0f);

    return 0;
}

int32_t LSM6DSOX_ReadGyro(lsm6dsox_handle_t *handle, lsm6dsox_3axis_data_t *gyro_data) {
    lsm6dsox_3axis_raw_t gyro_raw;
    if (LSM6DSOX_ReadGyroRaw(handle, &gyro_raw) != 0) {
        return -1;
    }

    float sensitivity = 0.0f;
    switch (handle->gyro_fs) {
        case LSM6DSOX_GYRO_FS_250DPS:   sensitivity = 8.75f; break; // mdps/LSB
        case LSM6DSOX_GYRO_FS_500DPS:   sensitivity = 17.50f; break;
        case LSM6DSOX_GYRO_FS_1000DPS:  sensitivity = 35.0f; break;
        case LSM6DSOX_GYRO_FS_2000DPS:  sensitivity = 70.0f; break;
        default:                        sensitivity = 8.75f; break; // Default to 250dps if unknown
    }

    // Convert raw LSB to dps (divide by 1000 since sensitivity is mdps/LSB)
    gyro_data->x = (float)gyro_raw.x * (sensitivity / 1000.0f);
    gyro_data->y = (float)gyro_raw.y * (sensitivity / 1000.0f);
    gyro_data->z = (float)gyro_raw.z * (sensitivity / 1000.0f);

    return 0;
}
