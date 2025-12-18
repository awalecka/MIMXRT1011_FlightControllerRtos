/**
 * @file tasks.cpp
 * @brief Implements the state-specific tasks for the flight controller.
 */
#include <flight_controller.h>
#include <fsl_debug_console.h>
#include <board.h>
#include <utils.h>

/**
 * @brief Task for the FLIGHT state.
 */
void flightTask(void *pvParameters) {
    const TickType_t xFlightLoopFrequency = pdMS_TO_TICKS(10); // 100Hz
	TickType_t xLastWakeTime = xTaskGetTickCount();

    PRINTF("Flight State \r\n");
	g_heartbeat_frequency = pdMS_TO_TICKS(250); // 2Hz

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFlightLoopFrequency);
        // Use the global controller instance
        USER_TIMING_ON();
        g_flightController.update();
        USER_TIMING_OFF();
    }
}

/**
 * @brief Task for the IDLE state.
 */
void idleTask(void *pvParameters) {
	g_heartbeat_frequency = pdMS_TO_TICKS(500); // 1Hz

	while (true) {
        PRINTF("State: IDLE - System ready. Waiting for command...\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- Request transition to flight ---
        // Instead of writing to g_flight_state, send a request to the state manager.
        FlightState_t new_state = STATE_CALIBRATE;
        xQueueSend(g_state_change_request_queue, &new_state, 0);

        // This task will now be suspended by the state manager once it
        // processes the request. The vTaskDelay ensures this loop
        // doesn't spin, repeatedly sending requests.
    }
}

/**
 * @brief Task for the CALIBRATE state.
 */
void calibrateTask(void *pvParameters) {
	g_heartbeat_frequency = pdMS_TO_TICKS(100); // Fast blink for calibration

	while (true) {
        PRINTF("State: CALIBRATE - Starting Sensor Calibration...\r\n");

        // Blocking call to perform calibration
        g_flightController.calibrateSensors();

        PRINTF("State: CALIBRATE - Done. Returning to IDLE.\r\n");

        // Automatically transition back to IDLE
        FlightState_t new_state = STATE_FLIGHT;
        xQueueSend(g_state_change_request_queue, &new_state, 0);

        // Wait to be suspended by state manager
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
