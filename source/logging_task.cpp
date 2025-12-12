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
	sensor_data_vehicle_t received_sensor_data;
	ActuatorOutput surface_commands;

    while (1) {

#if 0
    	// Wait indefinitely for an item to appear in the queue.
    	if (xQueueReceive(g_sensor_data_queue, &received_sensor_data, 0) == pdPASS) {
    	    PRINTF("[Sensors] T:%lu, AX:%.2f, AY:%.2f, AZ:%.2f, GX:%.2f, GY:%.2f, GZ:%.2f\r\n",
    	    		received_sensor_data.timestamp,
					received_sensor_data.acc_data.x, received_sensor_data.acc_data.y, received_sensor_data.acc_data.z,
					received_sensor_data.gyro_data.x, received_sensor_data.gyro_data.y, received_sensor_data.gyro_data.z);
        }
#endif
        if (xQueueReceive(g_controls_data_queue, &surface_commands, portMAX_DELAY) == pdPASS) {
            PRINTF("[Controls] A:%.2f, E:%.2f, R:%.2f \r\n",
            		surface_commands.aileron,
					surface_commands.elevator, surface_commands.rudder);
        }

        RC_Channels_t rcData;
        if (xQueueReceive(g_command_data_queue, &rcData, 0) == pdPASS) {
            PRINTF("[Commands] %i \r\n",
            		rcData.channels[1]);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
}





