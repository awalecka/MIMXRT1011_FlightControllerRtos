/**
 * @file heartbeat_task.cpp
 * @brief Implements the heartbeat task.
 */
#include "flight_controller.h"
#include "fsl_debug_console.h"
#include "board.h"

/**
 * @brief Task that generates a heartbeat.
 */
void heartbeat_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xHeartbeatFrequency = pdMS_TO_TICKS(500); // 1Hz

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xHeartbeatFrequency);
        USER_LED_TOGGLE();
    }
}
