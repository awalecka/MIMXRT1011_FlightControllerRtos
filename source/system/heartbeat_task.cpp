/**
 * @file heartbeat_task.cpp
 * @brief Implements the heartbeat task.
 */
#include "system/heartbeat_task.h"
#include "flight_controller.h"
#include "board.h"

/**
 * @brief Task that generates a heartbeat.
 */
void heartbeatTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, g_heartbeat_frequency);
        USER_LED_TOGGLE();
    }
}
