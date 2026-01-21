#ifndef SENSOR_SYSTEM_H
#define SENSOR_SYSTEM_H

#include "sensors/rls_mag_calibration.h"
#include "drivers/imu_concepts.h"
#include "drivers/i2c_sync.h"
#include "drivers/lsm6dsox.h"
#include <FreeRTOS.h>
#include <task.h>

/**
 * @brief Policy-Based Sensor System.
 * Templates resolve at compile time for zero runtime overhead.
 */
template <SixAxisSensor AccelPolicy, MagnetometerSensor MagPolicy>
class SensorSystem {
public:
    struct RawData {
        float gyroXDps, gyroYDps, gyroZDps;
        float accelXG, accelYG, accelZG;
        float magXGauss, magYGauss, magZGauss;
        float airspeedMs;
    };

    SensorSystem() : gyroBiasX(0.0f), gyroBiasY(0.0f), gyroBiasZ(0.0f),
                     magCalibrator(0.98f, 1000.0f, 0.01f) {
        // Vehicle mapping definition
        vehicleMapping.map_x = LSM6DSOX_AXIS_X_POSITIVE;
        vehicleMapping.map_y = LSM6DSOX_AXIS_Y_NEGATIVE;
        vehicleMapping.map_z = LSM6DSOX_AXIS_Z_NEGATIVE;
    }

    int init() {
        i2c_sync_global_init();
        if (accelDriver.init() != 0) return -1;
        if (magDriver.init() != 0) return -1;
        return 0;
    }

    void calibrateGyro() {
        const int sampleCount = 200;
        float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
        Axis3f rawGyro;
        Axis3f mappedGyro;

        for (int i = 0; i < sampleCount; i++) {
            if (accelDriver.readGyro(rawGyro) == 0) {
                remapAxis(rawGyro, mappedGyro);
                sumX += mappedGyro.x;
                sumY += mappedGyro.y;
                sumZ += mappedGyro.z;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        gyroBiasX = sumX / (float)sampleCount;
        gyroBiasY = sumY / (float)sampleCount;
        gyroBiasZ = sumZ / (float)sampleCount;
    }

    /**
     * @brief Reads sensor data and applies CURRENT calibration.
     */
    int readData(RawData& rawData) {
        Axis3f rawAcc, rawGyro, rawMag;
        Axis3f mapAcc, mapGyro;

        if (accelDriver.readAccel(rawAcc) != 0 ||
            accelDriver.readGyro(rawGyro) != 0 ||
            magDriver.readMag(rawMag) != 0) {
            return -1;
        }

        remapAxis(rawAcc, mapAcc);
        remapAxis(rawGyro, mapGyro);

        rawData.gyroXDps = mapGyro.x - gyroBiasX;
        rawData.gyroYDps = mapGyro.y - gyroBiasY;
        rawData.gyroZDps = mapGyro.z - gyroBiasZ;
        rawData.accelXG = mapAcc.x;
        rawData.accelYG = mapAcc.y;
        rawData.accelZG = mapAcc.z;
        rawData.airspeedMs = 0.0f;

        // Apply Magnetometer Calibration
        using RlsMagnetometerCalibratorF = RlsMagnetometerCalibration<float>;
        RlsMagnetometerCalibratorF::Vector3 rawMagVec(rawMag.x, rawMag.y, rawMag.z);
        RlsMagnetometerCalibratorF::Vector3 calMagVec = magCalibrator.getCalibratedData(rawMagVec);

        rawData.magXGauss = calMagVec.x();
        rawData.magYGauss = calMagVec.y();
        rawData.magZGauss = calMagVec.z();

        return 0;
    }

    /**
     * @brief Reads raw mag data for calibration purposes.
     */
    int feedMagCalibration(RawData& rawData) {
        Axis3f rawMag;
        if (magDriver.readMag(rawMag) != 0) return -1;

        // Store Raw for Logging (no cal applied)
        rawData.magXGauss = rawMag.x;
        rawData.magYGauss = rawMag.y;
        rawData.magZGauss = rawMag.z;

        return 0;
    }

    void setCalibration(const float hardIron[3], const float softIron[9]) {
        using RlsMagnetometerCalibratorF = RlsMagnetometerCalibration<float>;
        RlsMagnetometerCalibratorF::Vector3 offset(hardIron[0], hardIron[1], hardIron[2]);
        RlsMagnetometerCalibratorF::Matrix3 matrix;
        matrix << softIron[0], softIron[1], softIron[2],
                  softIron[3], softIron[4], softIron[5],
                  softIron[6], softIron[7], softIron[8];
        magCalibrator.setInitialCalibration(matrix, offset);
    }

    void getCalibration(float hardIron[3], float softIron[9]) {
        auto offset = magCalibrator.getHardIronOffset();
        auto matrix = magCalibrator.getSoftIronCorrection();

        hardIron[0] = offset.x(); hardIron[1] = offset.y(); hardIron[2] = offset.z();
        for(int r=0; r<3; r++) {
            for(int c=0; c<3; c++) {
                softIron[r*3 + c] = matrix(r, c);
            }
        }
    }

private:
    AccelPolicy accelDriver;
    MagPolicy magDriver;
    float gyroBiasX, gyroBiasY, gyroBiasZ;
    RlsMagnetometerCalibration<float> magCalibrator;
    lsm6dsox_axis_mapping_t vehicleMapping;

    void remapAxis(const Axis3f& input, Axis3f& output) {
        output.x = getMappedValue(input, vehicleMapping.map_x);
        output.y = getMappedValue(input, vehicleMapping.map_y);
        output.z = getMappedValue(input, vehicleMapping.map_z);
    }

    float getMappedValue(const Axis3f& input, lsm6dsox_axis_source_t source) {
        switch (source) {
            case LSM6DSOX_AXIS_X_POSITIVE: return input.x;
            case LSM6DSOX_AXIS_X_NEGATIVE: return -input.x;
            case LSM6DSOX_AXIS_Y_POSITIVE: return input.y;
            case LSM6DSOX_AXIS_Y_NEGATIVE: return -input.y;
            case LSM6DSOX_AXIS_Z_POSITIVE: return input.z;
            case LSM6DSOX_AXIS_Z_NEGATIVE: return -input.z;
            default: return 0.0f;
        }
    }
};

#endif // SENSOR_SYSTEM_H
