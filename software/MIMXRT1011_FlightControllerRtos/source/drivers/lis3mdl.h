#ifndef LIS3MDL_H
#define LIS3MDL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "Driver_I2C.h" // CMSIS I2C Driver API
#include "i2c_sync.h"   // Include common I2C synchronization functions

// Include FreeRTOS headers for semaphore synchronization
#include "FreeRTOS.h"
#include "semphr.h"

// --- Defines for LIS3MDL ---
#define LIS3MDL_I2C_ADDR_SA1_LOW    0x1C // SA1 pin connected to GND
#define LIS3MDL_I2C_ADDR_SA1_HIGH   0x1E // SA1 pin connected to VDD
#define LIS3MDL_I2C_ADDR            LIS3MDL_I2C_ADDR_SA1_LOW // Default address

// --- LIS3MDL Register Map ---
#define LIS3MDL_REG_WHO_AM_I        0x0F
#define LIS3MDL_WHO_AM_I_VAL        0x3D // Expected value for WHO_AM_I register

#define LIS3MDL_REG_CTRL_REG1       0x20
#define LIS3MDL_REG_CTRL_REG2       0x21
#define LIS3MDL_REG_CTRL_REG3       0x22
#define LIS3MDL_REG_CTRL_REG4       0x23
#define LIS3MDL_REG_CTRL_REG5       0x24

#define LIS3MDL_REG_STATUS_REG      0x27
#define LIS3MDL_REG_OUT_X_L         0x28
#define LIS3MDL_REG_OUT_X_H         0x29
#define LIS3MDL_REG_OUT_Y_L         0x2A
#define LIS3MDL_REG_OUT_Y_H         0x2B
#define LIS3MDL_REG_OUT_Z_L         0x2C
#define LIS3MDL_REG_OUT_Z_H         0x2D

// --- Configuration Enumerations ---

// Full Scale Selection (CTRL_REG2, bits FS[1:0])
typedef enum {
    LIS3MDL_FS_4_GAUSS    = 0x00, // +/- 4 gauss
    LIS3MDL_FS_8_GAUSS    = 0x01, // +/- 8 gauss
    LIS3MDL_FS_12_GAUSS   = 0x02, // +/- 12 gauss
    LIS3MDL_FS_16_GAUSS   = 0x03, // +/- 16 gauss
} lis3mdl_fs_t;

// Operating Mode & ODR (CTRL_REG1, CTRL_REG4)
// Simplified ODR selection for common performance modes
typedef enum {
    LIS3MDL_ODR_10_HZ_LP,   // 10 Hz, Low Power Mode
    LIS3MDL_ODR_40_HZ_MP,   // 40 Hz, Medium Performance Mode
    LIS3MDL_ODR_80_HZ_HP,   // 80 Hz, High Performance Mode
} lis3mdl_odr_config_t;

// --- Data Structures ---
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lis3mdl_3axis_raw_t;

typedef struct {
    float x;
    float y;
    float z;
} lis3mdl_3axis_data_t;

// --- LIS3MDL Handle Structure ---
typedef struct {
    i2c_sync_handle_t i2c_sync; // Embedded generic I2C synchronization handle
    lis3mdl_fs_t mag_fs;        // Current magnetometer full scale setting
} lis3mdl_handle_t;


// --- Function Prototypes ---

/**
 * @brief Initializes the LIS3MDL sensor.
 * @param[in,out] handle Pointer to the LIS3MDL handle structure.
 * @param[in] i2c_driver Pointer to the CMSIS I2C driver instance (e.g., &Driver_I2C0).
 * @param[in] i2c_addr I2C address of the sensor.
 * @return 0 on success, -1 on failure.
 */
int32_t LIS3MDL_Init(lis3mdl_handle_t *handle, ARM_DRIVER_I2C *i2c_driver, uint8_t i2c_addr);

/**
 * @brief Reads the WHO_AM_I register to verify sensor presence.
 * @param[in] handle Pointer to the LIS3MDL handle structure.
 * @return WHO_AM_I value on success, -1 on I2C error.
 */
int32_t LIS3MDL_ReadID(lis3mdl_handle_t *handle);

/**
 * @brief Configures the magnetometer's ODR and Full Scale.
 * @param[in] handle Pointer to the LIS3MDL handle structure.
 * @param[in] odr Desired output data rate and performance mode configuration.
 * @param[in] fs Desired full scale.
 * @return 0 on success, -1 on failure.
 */
int32_t LIS3MDL_SetConfig(lis3mdl_handle_t *handle, lis3mdl_odr_config_t odr, lis3mdl_fs_t fs);

/**
 * @brief Reads raw magnetometer data.
 * @param[in] handle Pointer to the LIS3MDL handle structure.
 * @param[out] mag_raw Pointer to a structure to store raw magnetometer data.
 * @return 0 on success, -1 on failure.
 */
int32_t LIS3MDL_ReadMagnetometerRaw(lis3mdl_handle_t *handle, lis3mdl_3axis_raw_t *mag_raw);

/**
 * @brief Reads magnetometer data and converts it to gauss.
 * @param[in] handle Pointer to the LIS3MDL handle structure.
 * @param[out] mag_data Pointer to a structure to store magnetometer data in gauss.
 * @return 0 on success, -1 on failure.
 */
int32_t LIS3MDL_ReadMagnetometer(lis3mdl_handle_t *handle, lis3mdl_3axis_data_t *mag_data);

#ifdef __cplusplus
}
#endif

#endif // LIS3MDL_H
