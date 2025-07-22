#include "LIS3MDL.h"

// --- Public API Functions ---
// Statically allocated buffer for the semaphore's control block
static StaticSemaphore_t xLis3mdlSemaphoreBuffer;

/**
 * @brief Initializes the LIS3MDL sensor.
 */
int32_t LIS3MDL_Init(lis3mdl_handle_t *handle, ARM_DRIVER_I2C *i2c_driver, uint8_t i2c_addr) {
    if (handle == NULL || i2c_driver == NULL) {
        return -1; // Invalid parameters
    }

    // Initialize the embedded i2c_sync_handle_t
    handle->i2c_sync.i2c_driver = i2c_driver;
    handle->i2c_sync.i2c_addr   = i2c_addr;
    handle->i2c_sync.i2c_transfer_status = 0;

    // Default full scale
    handle->mag_fs = LIS3MDL_FS_4_GAUSS;

    // Create the binary semaphore for I2C transfer completion
    handle->i2c_sync.i2c_semaphore = xSemaphoreCreateBinaryStatic(&xLis3mdlSemaphoreBuffer);
    if (handle->i2c_sync.i2c_semaphore == NULL) {
        return -1; // Failed to create semaphore
    }
    // Ensure the semaphore is initially empty
    xSemaphoreTake(handle->i2c_sync.i2c_semaphore, 0);

    // Initialize the I2C driver with the common event callback
    if (handle->i2c_sync.i2c_driver->Initialize(i2c_sync_event_callback) != ARM_DRIVER_OK) {
        return -1;
    }
    if (handle->i2c_sync.i2c_driver->PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK) {
        return -1;
    }
    if (handle->i2c_sync.i2c_driver->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST) != ARM_DRIVER_OK) {
        return -1;
    }

    // Verify sensor communication by reading WHO_AM_I
    int32_t who_am_i = LIS3MDL_ReadID(handle);
    if (who_am_i != LIS3MDL_WHO_AM_I_VAL) {
        return -1; // Sensor not found or incorrect ID
    }

    // Set a default configuration
    if (LIS3MDL_SetConfig(handle, LIS3MDL_ODR_80_HZ_HP, LIS3MDL_FS_4_GAUSS) != 0) {
        return -1;
    }

    // Set to continuous conversion mode
    uint8_t ctrl_reg3_val = 0x00; // Continuous-conversion mode
    if (i2c_sync_write_byte(&handle->i2c_sync, LIS3MDL_REG_CTRL_REG3, ctrl_reg3_val) != 0) {
        return -1;
    }


    return 0; // Success
}

/**
 * @brief Reads the WHO_AM_I register.
 */
int32_t LIS3MDL_ReadID(lis3mdl_handle_t *handle) {
    uint8_t who_am_i_val;
    if (i2c_sync_read_byte(&handle->i2c_sync, LIS3MDL_REG_WHO_AM_I, &who_am_i_val) != 0) {
        return -1; // I2C error
    }
    return (int32_t)who_am_i_val;
}

/**
 * @brief Configures the magnetometer's ODR and Full Scale.
 */
int32_t LIS3MDL_SetConfig(lis3mdl_handle_t *handle, lis3mdl_odr_config_t odr, lis3mdl_fs_t fs) {
    uint8_t ctrl1_val = 0;
    uint8_t ctrl2_val = 0;
    uint8_t ctrl4_val = 0;

    // Configure Full Scale (CTRL_REG2)
    ctrl2_val |= ((uint8_t)fs << 5);
    handle->mag_fs = fs; // Store the current full scale

    // Configure ODR and Performance Mode (CTRL_REG1 and CTRL_REG4)
    switch (odr) {
        case LIS3MDL_ODR_10_HZ_LP:
            ctrl1_val |= (0x00 << 5) | (0x04 << 2); // OM=LP, DO=10Hz
            ctrl4_val |= (0x00 << 2);              // OMZ=LP
            break;
        case LIS3MDL_ODR_40_HZ_MP:
            ctrl1_val |= (0x01 << 5) | (0x06 << 2); // OM=MP, DO=40Hz
            ctrl4_val |= (0x01 << 2);              // OMZ=MP
            break;
        case LIS3MDL_ODR_80_HZ_HP:
            ctrl1_val |= (0x02 << 5) | (0x07 << 2); // OM=HP, DO=80Hz
            ctrl4_val |= (0x02 << 2);              // OMZ=HP
            break;
    }

    // Write the configuration registers
    if (i2c_sync_write_byte(&handle->i2c_sync, LIS3MDL_REG_CTRL_REG1, ctrl1_val) != 0) return -1;
    if (i2c_sync_write_byte(&handle->i2c_sync, LIS3MDL_REG_CTRL_REG2, ctrl2_val) != 0) return -1;
    if (i2c_sync_write_byte(&handle->i2c_sync, LIS3MDL_REG_CTRL_REG4, ctrl4_val) != 0) return -1;

    return 0;
}


/**
 * @brief Reads raw magnetometer data.
 */
int32_t LIS3MDL_ReadMagnetometerRaw(lis3mdl_handle_t *handle, lis3mdl_3axis_raw_t *mag_raw) {
    uint8_t data[6];
    // Read 6 bytes starting from OUT_X_L (0x28).
    // The LIS3MDL auto-increments the register address on multi-byte reads.
    if (i2c_sync_read_bytes(&handle->i2c_sync, LIS3MDL_REG_OUT_X_L, data, 6) != 0) {
        return -1; // I2C error
    }

    // Data is 16-bit signed, LSB first (little endian)
    mag_raw->x = (int16_t)((data[1] << 8) | data[0]);
    mag_raw->y = (int16_t)((data[3] << 8) | data[2]);
    mag_raw->z = (int16_t)((data[5] << 8) | data[4]);

    return 0;
}

/**
 * @brief Reads magnetometer data and converts it to gauss.
 */
int32_t LIS3MDL_ReadMagnetometer(lis3mdl_handle_t *handle, lis3mdl_3axis_data_t *mag_data) {
    lis3mdl_3axis_raw_t mag_raw;
    if (LIS3MDL_ReadMagnetometerRaw(handle, &mag_raw) != 0) {
        return -1;
    }

    float sensitivity = 0.0f;
    switch (handle->mag_fs) {
        case LIS3MDL_FS_4_GAUSS:    sensitivity = 1.0f / 6842.0f; break; // LSB/gauss -> gauss/LSB
        case LIS3MDL_FS_8_GAUSS:    sensitivity = 1.0f / 3421.0f; break;
        case LIS3MDL_FS_12_GAUSS:   sensitivity = 1.0f / 2281.0f; break;
        case LIS3MDL_FS_16_GAUSS:   sensitivity = 1.0f / 1711.0f; break;
        default:                    sensitivity = 1.0f / 6842.0f; break; // Default to 4G
    }

    // Convert raw LSB to gauss
    mag_data->x = (float)mag_raw.x * sensitivity;
    mag_data->y = (float)mag_raw.y * sensitivity;
    mag_data->z = (float)mag_raw.z * sensitivity;

    return 0;
}
