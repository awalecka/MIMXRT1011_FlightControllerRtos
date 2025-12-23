#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include <stdint.h>
#include "Driver_I2C.h" // CMSIS I2C Driver API
#include "i2c_sync.h"   // Include common I2C synchronization functions

// Include FreeRTOS headers for semaphore, if using FreeRTOS for asynchronous event handling.
// If not using FreeRTOS, you will need to implement a custom synchronization mechanism (e.g., a flag set by interrupt).
#include "FreeRTOS.h"
#include "semphr.h" // For SemaphoreHandle_t

// --- Defines for LSM6DSOX ---
#define LSM6DSOX_I2C_ADDR_SA0_LOW   0x6A // SDO/SA0 pin connected to GND
#define LSM6DSOX_I2C_ADDR_SA0_HIGH  0x6B // SDO/SA0 pin connected to VDD (if used)
#define LSM6DSOX_I2C_ADDR           LSM6DSOX_I2C_ADDR_SA0_LOW // Default address

// --- LSM6DSOX Register Map ---
#define LSM6DSOX_REG_WHO_AM_I       0x0F
#define LSM6DSOX_WHO_AM_I_VAL       0x6C // Expected value for WHO_AM_I register

#define LSM6DSOX_REG_CTRL1_XL       0x10 // Accelerometer control
#define LSM6DSOX_REG_CTRL2_G        0x11 // Gyroscope control
#define LSM6DSOX_REG_CTRL3_C        0x12 // Control register 3
#define LSM6DSOX_REG_CTRL4_C        0x13
#define LSM6DSOX_REG_CTRL5_C        0x14
#define LSM6DSOX_REG_CTRL6_C        0x15
#define LSM6DSOX_REG_CTRL7_G        0x16
#define LSM6DSOX_REG_CTRL8_XL       0x17
#define LSM6DSOX_REG_CTRL9_XL       0x18
#define LSM6DSOX_REG_CTRL10_C       0x19

#define LSM6DSOX_REG_OUTX_L_G       0x22 // Gyroscope X-axis low byte
#define LSM6DSOX_REG_OUTX_H_G       0x23 // Gyroscope X-axis high byte
#define LSM6DSOX_REG_OUTY_L_G       0x24 // Gyroscope Y-axis low byte
#define LSM6DSOX_REG_OUTY_H_G       0x25 // Gyroscope Z-axis low byte
#define LSM6DSOX_REG_OUTZ_L_G       0x26 // Gyroscope Z-axis high byte
#define LSM6DSOX_REG_OUTZ_H_G       0x27 // Gyroscope Z-axis high byte

#define LSM6DSOX_REG_OUTX_L_A       0x28 // Accelerometer X-axis low byte
#define LSM6DSOX_REG_OUTX_H_A       0x29 // Accelerometer X-axis high byte
#define LSM6DSOX_REG_OUTY_L_A       0x2A // Accelerometer Y-axis low byte
#define LSM6DSOX_REG_OUTY_H_A       0x2B // Accelerometer Y-axis high byte
#define LSM6DSOX_REG_OUTZ_L_A       0x2C // Accelerometer Z-axis low byte
#define LSM6DSOX_REG_OUTZ_H_A       0x2D // Accelerometer Z-axis high byte

// --- ODR (Output Data Rate) values for CTRL1_XL and CTRL2_G ---
// Accelerometer ODR in CTRL1_XL (bits 7:4)
typedef enum {
    LSM6DSOX_ACC_ODR_OFF        = 0x00, // Power-down
    LSM6DSOX_ACC_ODR_12_5Hz     = 0x01,
    LSM6DSOX_ACC_ODR_26Hz       = 0x02,
    LSM6DSOX_ACC_ODR_52Hz       = 0x03,
    LSM6DSOX_ACC_ODR_104Hz      = 0x04,
    LSM6DSOX_ACC_ODR_208Hz      = 0x05,
    LSM6DSOX_ACC_ODR_416Hz      = 0x06,
    LSM6DSOX_ACC_ODR_833Hz      = 0x07,
    LSM6DSOX_ACC_ODR_1_66kHz    = 0x08,
    LSM6DSOX_ACC_ODR_3_33kHz    = 0x09,
    LSM6DSOX_ACC_ODR_6_66kHz    = 0x0A,
} lsm6dsox_acc_odr_t;

// Gyroscope ODR in CTRL2_G (bits 7:4)
typedef enum {
    LSM6DSOX_GYRO_ODR_OFF       = 0x00, // Power-down
    LSM6DSOX_GYRO_ODR_12_5Hz    = 0x01,
    LSM6DSOX_GYRO_ODR_26Hz      = 0x02,
    LSM6DSOX_GYRO_ODR_52Hz      = 0x03,
    LSM6DSOX_GYRO_ODR_104Hz     = 0x04,
    LSM6DSOX_GYRO_ODR_208Hz     = 0x05,
    LSM6DSOX_GYRO_ODR_416Hz     = 0x06,
    LSM6DSOX_GYRO_ODR_833Hz     = 0x07,
    LSM6DSOX_GYRO_ODR_1_66kHz   = 0x08,
    LSM6DSOX_GYRO_ODR_3_33kHz   = 0x09,
    LSM6DSOX_GYRO_ODR_6_66kHz   = 0x0A,
} lsm6dsox_gyro_odr_t;

// FS (Full Scale) values for CTRL1_XL and CTRL2_G
// Accelerometer Full Scale in CTRL1_XL (bits 3:2)
typedef enum {
    LSM6DSOX_ACC_FS_2G      = 0x00, // +/- 2g
    LSM6DSOX_ACC_FS_16G     = 0x01, // +/- 16g (default)
    LSM6DSOX_ACC_FS_4G      = 0x02, // +/- 4g
    LSM6DSOX_ACC_FS_8G      = 0x03, // +/- 8g
} lsm6dsox_acc_fs_t;

// Gyroscope Full Scale in CTRL2_G (bits 3:2)
typedef enum {
    LSM6DSOX_GYRO_FS_250DPS     = 0x00, // +/- 250 dps
    LSM6DSOX_GYRO_FS_500DPS     = 0x01, // +/- 500 dps
    LSM6DSOX_GYRO_FS_1000DPS    = 0x02, // +/- 1000 dps
    LSM6DSOX_GYRO_FS_2000DPS    = 0x03, // +/- 2000 dps
} lsm6dsox_gyro_fs_t;


// --- Data Structures ---
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} lsm6dsox_3axis_raw_t;

typedef struct {
    float x;
    float y;
    float z;
} lsm6dsox_3axis_data_t;

// --- NEW: Axis Remapping Structures ---
/**
 * @brief Defines a mapping from a single sensor axis to an output axis.
 * Used to specify which sensor axis (and its sign) corresponds to the
 * canonical axes of the vehicle/device (e.g., Forward, Right, Down).
 */
typedef enum {
    LSM6DSOX_AXIS_X_POSITIVE,
    LSM6DSOX_AXIS_X_NEGATIVE,
    LSM6DSOX_AXIS_Y_POSITIVE,
    LSM6DSOX_AXIS_Y_NEGATIVE,
    LSM6DSOX_AXIS_Z_POSITIVE,
    LSM6DSOX_AXIS_Z_NEGATIVE,
} lsm6dsox_axis_source_t;

/**
 * @brief Structure to hold the complete axis mapping configuration.
 * The user defines this based on the physical orientation of the sensor.
 */
typedef struct {
    lsm6dsox_axis_source_t map_x; // Defines the source for the output X-axis.
    lsm6dsox_axis_source_t map_y; // Defines the source for the output Y-axis.
    lsm6dsox_axis_source_t map_z; // Defines the source for the output Z-axis.
} lsm6dsox_axis_mapping_t;


// --- LSM6DSOX Handle Structure ---
typedef struct {
    i2c_sync_handle_t i2c_sync;   // Embedded generic I2C synchronization handle
    lsm6dsox_acc_fs_t acc_fs;     // Current accelerometer full scale setting
    lsm6dsox_gyro_fs_t gyro_fs;   // Current gyroscope full scale setting
} lsm6dsox_handle_t;


// --- Function Prototypes ---

/**
 * @brief Initializes the LSM6DSOX sensor.
 * @param[in,out] handle Pointer to the LSM6DSOX handle structure.
 * @param[in] i2c_driver Pointer to the CMSIS I2C driver instance (e.g., &Driver_I2C0).
 * @param[in] i2c_addr I2C address of the sensor (LSM6DSOX_I2C_ADDR_SA0_LOW or LSM6DSOX_I2C_ADDR_SA0_HIGH).
 * @return 0 on success, -1 on failure (e.g., WHO_AM_I mismatch, I2C error).
 */
int32_t LSM6DSOX_Init(lsm6dsox_handle_t *handle, ARM_DRIVER_I2C *i2c_driver, uint8_t i2c_addr);

/**
 * @brief Reads the WHO_AM_I register to verify sensor presence.
 * @param[in] handle Pointer to the LSM6DSOX handle structure.
 * @return WHO_AM_I value on success, -1 on I2C error.
 */
int32_t LSM6DSOX_ReadID(lsm6dsox_handle_t *handle);

/**
 * @brief Sets the Output Data Rate (ODR) and Full Scale (FS) for the accelerometer.
 * @param[in] handle Pointer to the LSM6DSOX handle structure.
 * @param[in] odr Desired accelerometer ODR.
 * @param[in] fs Desired accelerometer Full Scale.
 * @return 0 on success, -1 on failure.
 */
int32_t LSM6DSOX_SetAccConfig(lsm6dsox_handle_t *handle, lsm6dsox_acc_odr_t odr, lsm6dsox_acc_fs_t fs);

/**
 * @brief Sets the Output Data Rate (ODR) and Full Scale (FS) for the gyroscope.
 * @param[in] handle Pointer to the LSM6DSOX handle structure.
 * @param[in] odr Desired gyroscope ODR.
 * @param[in] fs Desired gyroscope Full Scale.
 * @return 0 on success, -1 on failure.
 */
int32_t LSM6DSOX_SetGyroConfig(lsm6dsox_handle_t *handle, lsm6dsox_gyro_odr_t odr, lsm6dsox_gyro_fs_t fs);

/**
 * @brief Reads raw accelerometer data.
 * @param[in] handle Pointer to the LSM6DSOX handle structure.
 * @param[out] acc_raw Pointer to a structure to store raw accelerometer data.
 * @return 0 on success, -1 on failure.
 */
int32_t LSM6DSOX_ReadAccRaw(lsm6dsox_handle_t *handle, lsm6dsox_3axis_raw_t *acc_raw);

/**
 * @brief Reads raw gyroscope data.
 * @param[in] handle Pointer to the LSM6DSOX handle structure.
 * @param[out] gyro_raw Pointer to a structure to store raw gyroscope data.
 * @return 0 on success, -1 on failure.
 */
int32_t LSM6DSOX_ReadGyroRaw(lsm6dsox_handle_t *handle, lsm6dsox_3axis_raw_t *gyro_raw);

/**
 * @brief Reads accelerometer data and converts to g (gravity units).
 * @param[in] handle Pointer to the LSM6DSOX handle structure.
 * @param[out] acc_data Pointer to a structure to store accelerometer data in g.
 * @return 0 on success, -1 on failure.
 */
int32_t LSM6DSOX_ReadAcc(lsm6dsox_handle_t *handle, lsm6dsox_3axis_data_t *acc_data);

/**
 * @brief Reads gyroscope data and converts to dps (degrees per second).
 * @param[in] handle Pointer to the LSM6DSOX handle structure.
 * @param[out] gyro_data Pointer to a structure to store gyroscope data in dps.
 * @return 0 on success, -1 on failure.
 */
int32_t LSM6DSOX_ReadGyro(lsm6dsox_handle_t *handle, lsm6dsox_3axis_data_t *gyro_data);

/**
 * @brief NEW: Remaps sensor data from the sensor's body frame to the device's canonical frame.
 * This is a crucial step before using the data in a sensor fusion algorithm.
 * The mapping must be defined by the user based on the physical mounting of the sensor.
 * @param[in] input_data Pointer to the source data (e.g., from LSM6DSOX_ReadAcc).
 * @param[in] mapping Pointer to the user-defined axis mapping configuration.
 * @param[out] output_data Pointer to a structure where the remapped data will be stored.
 * @return 0 on success, -1 if any parameters are NULL.
 */
int32_t LSM6DSOX_RemapData(const lsm6dsox_3axis_data_t *input_data, const lsm6dsox_axis_mapping_t *mapping, lsm6dsox_3axis_data_t *output_data);


#endif // LSM6DSOX_H
