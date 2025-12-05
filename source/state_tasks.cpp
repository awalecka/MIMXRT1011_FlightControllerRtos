/**
 * @file tasks.cpp
 * @brief Implements the state-specific tasks for the flight controller.
 */
#include <flight_controller.h>
#include <fsl_debug_console.h>
#include <fusion.h>
#include <board.h>
#include <peripherals.h>
#include <utils.h>
#include <vector>

/**
 * @brief Task for the FLIGHT state.
 */
void flight_task(void *pvParameters) {
    // ... (function unchanged) ...
    const TickType_t xFlightLoopFrequency = pdMS_TO_TICKS(10); // 100Hz
	TickType_t xLastWakeTime = xTaskGetTickCount();
	static FlightController controller(0.01);

    PRINTF("Flight State \r\n");
	g_heartbeat_frequency = pdMS_TO_TICKS(250); // 2Hz

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFlightLoopFrequency);
        USER_TIMING_ON();
        controller.update();
        USER_TIMING_OFF();
    }
}

/**
 * @brief Task for the IDLE state.
 */
void idle_task(void *pvParameters) {

	g_heartbeat_frequency = pdMS_TO_TICKS(500); // 1Hz

	while (1) {
        PRINTF("State: IDLE - System ready. Waiting for command...\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- Request transition to flight ---
        // Instead of writing to g_flight_state, send a request to the state manager.
        FlightState_t new_state = STATE_FLIGHT;
        xQueueSend(g_state_change_request_queue, &new_state, 0);

        // This task will now be suspended by the state manager once it
        // processes the request. The vTaskDelay ensures this loop
        // doesn't spin, repeatedly sending requests.
    }
}

/**
 * @brief Task for the CALIBRATE state.
 */
void calibrate_task(void *pvParameters) {

	g_heartbeat_frequency = pdMS_TO_TICKS(1000); // 0.5Hz

	while (1) {
        PRINTF("State: CALIBRATE - System ready. Waiting for command...\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // This delay is just for demo purposes

        // --- Request transition to flight ---
        FlightState_t new_state = STATE_FLIGHT;
        xQueueSend(g_state_change_request_queue, &new_state, 0);

        // This task will also be suspended by the state manager.
    }
}


