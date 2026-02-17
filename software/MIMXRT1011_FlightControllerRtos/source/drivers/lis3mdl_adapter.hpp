/**
 * @file lis3mdl_adapter.hpp
 * @brief Policy-based adapter for the LIS3MDL sensor.
 */

#ifndef LIS3MDL_ADAPTER_HPP
#define LIS3MDL_ADAPTER_HPP

#include "lis3mdl.h"
#include "imu_concepts.h"
#include "board.h"
#include "peripherals.h"

// Extern the driver defined in peripherals.c/h
extern ARM_DRIVER_I2C LPI2C1_SENSORS_CMSIS_DRIVER;

class Lis3mdlAdapter {
public:
    int init() {
    	if (LIS3MDL_Init(&m_handle, &LPI2C1_SENSORS_CMSIS_DRIVER, LIS3MDL_I2C_ADDR) != 0) {
    	            return -1;
		}

		// Explicitly set desired config (80Hz High Performance, 4 Gauss)
		if (LIS3MDL_SetConfig(&m_handle, LIS3MDL_ODR_80_HZ_HP, LIS3MDL_FS_4_GAUSS) != 0) {
			return -1;
		}

		return 0;
    }

    int readMag(Axis3f& data) {
        lis3mdl_3axis_data_t raw;
        int status = LIS3MDL_ReadMagnetometer(&m_handle, &raw);
        if (status == 0) {
            data.x = raw.x;
            data.y = raw.y;
            data.z = raw.z;
        }
        return status;
    }

private:
    lis3mdl_handle_t m_handle;
};

#endif // LIS3MDL_ADAPTER_HPP
