/**
 * @file state_manager.cpp
 * @brief Implements the main state machine controller task.
 */

#include "flight_controller.h"
#include "fsl_debug_console.h"
#include "Driver_I2C.h"
#include "peripherals.h"
#include "i2c_sync.h"

// CMSIS I2C Driver Instance
extern ARM_DRIVER_I2C LPI2C1_CMSIS_DRIVER;


/**
 * @brief Main state machine controller task.
 * Manages the lifecycle of state-specific tasks.
 */
void state_manager_task(void *pvParameters) {
    volatile FlightState_t last_known_state = STATE_BOOT;
    g_flight_state = STATE_BOOT;

    i2c_sync_global_init();

    // --- BOOT SEQUENCE ---
    PRINTF("State: BOOT\r\n");

    if (LSM6DSOX_Init(&g_sensor_handle, &LPI2C1_CMSIS_DRIVER, LSM6DSOX_I2C_ADDR) != 0 ||
        LSM6DSOX_SetAccConfig(&g_sensor_handle, LSM6DSOX_ACC_ODR_52Hz, LSM6DSOX_ACC_FS_2G) != 0 ||
        LSM6DSOX_SetGyroConfig(&g_sensor_handle, LSM6DSOX_GYRO_ODR_52Hz, LSM6DSOX_GYRO_FS_250DPS) != 0 ||
		LIS3MDL_Init(&g_mag_handle, &LPI2C1_CMSIS_DRIVER, LIS3MDL_I2C_ADDR) != 0 ||
		LIS3MDL_SetConfig(&g_mag_handle, LIS3MDL_ODR_80_HZ_HP, LIS3MDL_FS_4_GAUSS) != 0)
    {

        PRINTF("Error: Sensor initialization failed! Entering FAILSAFE.\r\n");
        g_flight_state = STATE_FAILSAFE;
    } else {
        PRINTF("Sensor initialized successfully.\r\n");
        g_flight_state = STATE_IDLE;
    }


    while (1) {

        // Check if the state has changed
        if (last_known_state != g_flight_state) {
            PRINTF("State transition: %d -> %d\r\n", last_known_state, g_flight_state);

            // Suspend the task of the previous state
            switch (last_known_state) {
                case STATE_IDLE:      vTaskSuspend(g_idle_task_handle); break;
                case STATE_FLIGHT:    vTaskSuspend(g_flight_task_handle); break;
                case STATE_CALIBRATE: vTaskSuspend(g_calibrate_task_handle); break;
                default: break; // No task to suspend for BOOT or FAILSAFE
            }

            // Resume the task of the new state
            switch (g_flight_state) {
                case STATE_IDLE:      vTaskResume(g_idle_task_handle); break;
                case STATE_FLIGHT:    vTaskResume(g_flight_task_handle); break;
                case STATE_CALIBRATE: vTaskResume(g_calibrate_task_handle); break;
                case STATE_FAILSAFE:
                    PRINTF("State: FAILSAFE - System halted. Please reset.\r\n");
                    vTaskSuspend(g_command_handler_task_handle);
                    vTaskSuspend(g_logging_task_handle);
                    vTaskSuspend(NULL); // Suspend self
                    break;
                default: break;
            }
            last_known_state = g_flight_state;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Check for state changes every 50ms

    }
}
