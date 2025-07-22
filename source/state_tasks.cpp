/**
 * @file tasks.cpp
 * @brief Implements the state-specific tasks for the flight controller.
 */
#include "flight_controller.h"
#include "fsl_debug_console.h"
#include "fusion.h"

/**
 * @brief Task for the FLIGHT state.
 */

void flight_task(void *pvParameters) {

    sensor_log_data_raw_t current_sensor_data_raw;
    sensor_log_data_vehicle_t current_sensor_data_vehicle;
    FusionAhrs ahrs;
    //RLS_MagnetometerCalibration<float> calibrator;
    //Eigen::Vector3f calMagnetometer;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFlightLoopFrequency = pdMS_TO_TICKS(10); // 100Hz

    PRINTF("Flight State \r\n");

	// Define the axis mapping based on your hardware setup
	lsm6dsox_axis_mapping_t vehicle_mapping = {
	    .map_x = LSM6DSOX_AXIS_X_POSITIVE,
	    .map_y = LSM6DSOX_AXIS_Y_NEGATIVE,
	    .map_z = LSM6DSOX_AXIS_Z_NEGATIVE
	};

    FusionAhrsInitialise(&ahrs);

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFlightLoopFrequency);

        // Read Sensors
        if (LSM6DSOX_ReadAcc(&g_sensor_handle, &current_sensor_data_raw.acc_data) != 0 ||
            LSM6DSOX_ReadGyro(&g_sensor_handle, &current_sensor_data_raw.gyro_data) != 0 ||
			LIS3MDL_ReadMagnetometerRaw(&g_mag_handle, &current_sensor_data_raw.mag_data) != 0) {
            PRINTF("Error: Sensor read failure in flight! Entering FAILSAFE.\r\n");
            g_flight_state = STATE_FAILSAFE;
            continue;
        }

        // Remap the data to the vehicle's coordinate frame
        LSM6DSOX_RemapData(&current_sensor_data_raw.acc_data, &vehicle_mapping, &current_sensor_data_vehicle.acc_data);
        LSM6DSOX_RemapData(&current_sensor_data_raw.gyro_data, &vehicle_mapping, &current_sensor_data_vehicle.gyro_data);

        // Run Flight Control Algorithms
    	//gyroscope data in degrees/s, accelerometer data in g
        const FusionVector gyroscope     = {current_sensor_data_vehicle.gyro_data.x, current_sensor_data_vehicle.gyro_data.y, current_sensor_data_vehicle.gyro_data.z};
        const FusionVector accelerometer = {current_sensor_data_vehicle.acc_data.x, current_sensor_data_vehicle.acc_data.y, current_sensor_data_vehicle.acc_data.z};
        const FusionVector magnetometer  = {current_sensor_data_vehicle.mag_data.x, current_sensor_data_vehicle.mag_data.y, current_sensor_data_vehicle.mag_data.z};
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, 0.01f);
        volatile FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        //PRINTF("%f \t %f \t %f \r\n", current_sensor_data_vehicle.acc_data.x, current_sensor_data_vehicle.acc_data.y, current_sensor_data_vehicle.acc_data.z);
        //PRINTF("%f \t %f \t %f \r\n", current_sensor_data_vehicle.gyro_data.x, current_sensor_data_vehicle.gyro_data.y, current_sensor_data_vehicle.gyro_data.z);
        //PRINTF("%f \t %f \t %f \r\n", current_sensor_data_vehicle.mag_data.x, current_sensor_data_vehicle.mag_data.y, current_sensor_data_vehicle.mag_data.z);
        //PRINTF("%d \t %d \t %d \r\n", current_sensor_data_raw.mag_data.x, current_sensor_data_raw.mag_data.y, current_sensor_data_raw.mag_data.z);
        PRINTF("%f \t %f \t %f \r\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);


        // Update Motor Outputs


        // Add timestamp and send data to the logging queue
        //current_sensor_data.timestamp = xTaskGetTickCount();
        //xQueueSend(g_sensor_data_queue, &current_sensor_data, (TickType_t)0);

    }
}




/**
 * @brief Task for the IDLE state.State transition: 0 -> 1
 */
void idle_task(void *pvParameters) {
    while (1) {
        PRINTF("State: IDLE - System ready. Waiting for command...\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
        g_flight_state = STATE_FLIGHT;

    }
}
