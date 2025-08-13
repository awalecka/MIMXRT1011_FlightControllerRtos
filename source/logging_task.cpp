/**
 * @file logging_task.cpp
 * @brief Implements the data logging task.
 */
#include "flight_controller.h"
#include "fsl_debug_console.h"

/**
 * @brief Task that waits for sensor data and writes it to a log file.
 */
void logging_task(void *pvParameters) {
	sensor_log_data_vehicle_t received_sensor_data;
	ActuatorOutput received_controls_data;


    while (1) {

    	// Wait indefinitely for an item to appear in the queue.
    	if (xQueueReceive(g_sensor_data_queue, &received_sensor_data, 0) == pdPASS) {
    	    PRINTF("[Sensors] T:%lu, AX:%.2f, AY:%.2f, AZ:%.2f, GX:%.2f, GY:%.2f, GZ:%.2f\r\n",
    	    		received_sensor_data.timestamp,
					received_sensor_data.acc_data.x, received_sensor_data.acc_data.y, received_sensor_data.acc_data.z,
					received_sensor_data.gyro_data.x, received_sensor_data.gyro_data.y, received_sensor_data.gyro_data.z);
        }

        if (xQueueReceive(g_controls_data_queue, &received_controls_data, 0) == pdPASS) {
            PRINTF("[Controls] T:%lu, A:%.2f, E:%.2f, R:%.2f \r\n",
            		received_controls_data.timestamp, received_controls_data.aileron,
					received_controls_data.elevator, received_controls_data.rudder);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}





