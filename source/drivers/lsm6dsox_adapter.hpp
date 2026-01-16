/**
 * @file lsm6dsox_adapter.hpp
 * @brief Policy-based adapter for the LSM6DSOX sensor.
 */

#ifndef LSM6DSOX_ADAPTER_HPP
#define LSM6DSOX_ADAPTER_HPP

#include "lsm6dsox.h"
#include "imu_concepts.h"
#include "board.h"
#include "peripherals.h"

// Extern the driver defined in peripherals.c/h or main
extern ARM_DRIVER_I2C LPI2C1_SENSORS_CMSIS_DRIVER;

class Lsm6dsoxAdapter {
public:
    int init() {
    	// Initialize the struct and I2C (Note: This sets ODR to OFF/Power-Down)
		if (LSM6DSOX_Init(&m_handle, &LPI2C1_SENSORS_CMSIS_DRIVER, LSM6DSOX_I2C_ADDR) != 0) {
			return -1;
		}

		// WAKE UP: Configure ODR and Full Scale to match original settings
		// Accel: 52Hz, 2G
		if (LSM6DSOX_SetAccConfig(&m_handle, LSM6DSOX_ACC_ODR_104Hz, LSM6DSOX_ACC_FS_2G) != 0) {
			return -1;
		}

		// Gyro: 104Hz, 250dps
		if (LSM6DSOX_SetGyroConfig(&m_handle, LSM6DSOX_GYRO_ODR_104Hz, LSM6DSOX_GYRO_FS_250DPS) != 0) {
			return -1;
		}

		return 0;
    }

    int readAccel(Axis3f& data) {
        lsm6dsox_3axis_data_t raw;
        int status = LSM6DSOX_ReadAcc(&m_handle, &raw);
        if (status == 0) {
            data.x = raw.x;
            data.y = raw.y;
            data.z = raw.z;
        }
        return status;
    }

    int readGyro(Axis3f& data) {
        lsm6dsox_3axis_data_t raw;
        int status = LSM6DSOX_ReadGyro(&m_handle, &raw);
        if (status == 0) {
            data.x = raw.x;
            data.y = raw.y;
            data.z = raw.z;
        }
        return status;
    }

private:
    lsm6dsox_handle_t m_handle;
};

#endif // LSM6DSOX_ADAPTER_HPP
