/**
 * @file logging_task.cpp
 * @brief Implements the data logging task.
 */
#include "flight_controller.h"
#include "fsl_debug_console.h"

/**
 * @brief Task that waits for sensor data and writes it to a log file.
 */
void loggingTask(void *pvParameters) {
    ActuatorOutput surface_commands;

    while (true) {
        // Wait for telemetry data (runs at a decimated rate from the flight task)
        if (xQueueReceive(g_controls_data_queue, &surface_commands, portMAX_DELAY) == pdPASS) {

#if 0
            // Print Controls
            PRINTF("[Controls] A:%.2f, E:%.2f, R:%.2f ",
                    surface_commands.aileron,
                    surface_commands.elevator,
                    surface_commands.rudder);


            RC_Channels_t rcData = g_flightController.getRcData();

            PRINTF("| [RC] 1:%u 2:%u 3:%u 4:%u\r\n",
                    rcData.channels[0], // Roll
                    rcData.channels[1], // Pitch
                    rcData.channels[2], // Throttle
                    rcData.channels[3]); // Yaw
#endif
        }
    }
}
