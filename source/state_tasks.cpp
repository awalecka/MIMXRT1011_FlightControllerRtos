/**
 * @file tasks.cpp
 * @brief Implements the state-specific tasks for the flight controller.
 */
#include "flight_controller.h"
#include "fsl_debug_console.h"
#include "fusion.h"
#include "board.h"
#include "peripherals.h"
#include "utils.h"
#include <vector>

/**
 * @brief Task for the FLIGHT state.
 */
__attribute__((section(".text.$SRAM_ITC")))
void flight_task(void *pvParameters) {

	constexpr TickType_t xFlightLoopFrequency = pdMS_TO_TICKS(10); // 100Hz
	TickType_t xLastWakeTime = xTaskGetTickCount();
	constexpr float dt = 0.01f;

    sensor_log_data_raw_t current_sensor_data_raw;
    sensor_log_data_vehicle_t current_sensor_data_vehicle;
    FusionAhrs ahrs;
    AttitudeController controller;
    FullSensorData sensor_data;
    ActuatorOutput controls;

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

        USER_TIMING_ON();

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
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, dt);
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Update sensors
        sensor_data.pitch_deg = euler.angle.pitch;
		sensor_data.roll_deg = euler.angle.roll;
		sensor_data.yaw_deg = euler.angle.yaw;
		sensor_data.pitch_rate_dps = current_sensor_data_vehicle.gyro_data.y;
		sensor_data.roll_rate_dps = current_sensor_data_vehicle.gyro_data.x;
		sensor_data.yaw_rate_dps = current_sensor_data_vehicle.gyro_data.z;
		sensor_data.true_airspeed_ms = 0;

        controller.setSetpoints( 0, 0);
        controls = controller.update(sensor_data, dt);

        const std::vector<unsigned short>* channelsPtr = nullptr;
        if (xQueueReceive(g_command_data_queue, &channelsPtr, 0) == pdPASS) {
            // Check that the pointer is not null before using it.
            if (channelsPtr != nullptr) {

            	// Create a reference to the vector (more efficient, no copy)
            	const std::vector<unsigned short>& channels = *channelsPtr;
                PWM_UpdatePwmDutycycle(PWM1_PERIPHERAL, kPWM_Module_0, kPWM_PwmA, kPWM_EdgeAligned, map_ushort(channels[0], 1000, 2000, 50, 100));
                PWM_UpdatePwmDutycycle(PWM1_PERIPHERAL, kPWM_Module_0, kPWM_PwmB, kPWM_EdgeAligned, map_ushort(channels[1], 1000, 2000, 50, 100));
                PWM_UpdatePwmDutycycle(PWM1_PERIPHERAL, kPWM_Module_2, kPWM_PwmA, kPWM_EdgeAligned, map_ushort(channels[2], 1000, 2000, 50, 100));
                PWM_UpdatePwmDutycycle(PWM1_PERIPHERAL, kPWM_Module_2, kPWM_PwmB, kPWM_EdgeAligned, map_ushort(channels[3], 1000, 2000, 50, 100));
                PWM_UpdatePwmDutycycle(PWM1_PERIPHERAL, kPWM_Module_3, kPWM_PwmA, kPWM_EdgeAligned, map_ushort(channels[4], 1000, 2000, 50, 100));
                PWM_UpdatePwmDutycycle(PWM1_PERIPHERAL, kPWM_Module_3, kPWM_PwmB, kPWM_EdgeAligned, map_ushort(channels[5], 1000, 2000, 50, 100));
                PWM_SetPwmLdok(PWM1_PERIPHERAL, (kPWM_Control_Module_0 | kPWM_Control_Module_2 | kPWM_Control_Module_3), true);

                // Delete the vector to free the memory ---
                // This is the most critical step. The receiver has accepted
                // ownership and MUST free the memory to prevent leaks.
                delete channelsPtr;

                // Set pointer to null to prevent accidental reuse.
                channelsPtr = nullptr;
            }
        }

        controls.timestamp = xTaskGetTickCount();
        xQueueSend(g_controls_data_queue, &controls, (TickType_t)0);

        // Update Motor Outputs
        current_sensor_data_vehicle.timestamp = xTaskGetTickCount();
        xQueueSend(g_sensor_data_queue, &current_sensor_data_vehicle, (TickType_t)0);

        USER_TIMING_OFF();

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
