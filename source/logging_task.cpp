/**
 * @file logging_task.cpp
 * @brief Implements the data logging task.
 */
#include "flight_controller.h"
#include "fsl_debug_console.h"

// --- SD Card Placeholders ---
// In a real implementation, you would include headers for FatFs, etc.
// #include "ff.h"
// #include "fsl_sd.h"

// static FATFS g_file_system;
// static FIL g_log_file;

/**
 * @brief Initializes the SD card and opens a new log file.
 * @return True on success, false on failure.
 */
static bool init_sd_card_and_logfile() {
    PRINTF("[Logger] Initializing SD card...\r\n");
    // Placeholder for mounting the filesystem
    // f_mount(&g_file_system, "0:", 1);

    PRINTF("[Logger] Opening log file...\r\n");
    // Placeholder for opening a file
    // f_open(&g_log_file, "log.csv", FA_WRITE | FA_CREATE_ALWAYS);

    // Placeholder for writing a CSV header
    // f_printf(&g_log_file, "Timestamp,AccX,AccY,AccZ,GyroX,GyroY,GyroZ\n");

    PRINTF("[Logger] SD card and log file ready.\r\n");
    return true; // Placeholder
}

/**
 * @brief Task that waits for sensor data and writes it to a log file.
 */
void logging_task(void *pvParameters) {
	sensor_log_data_vehicle_t received_sensor_data;
	ActuatorOutput received_controls_data;

    // Initialize the SD card. If it fails, suspend the task.
    if (!init_sd_card_and_logfile()) {
        PRINTF("[Logger] Error: Failed to initialize SD card. Halting logger.\r\n");
        vTaskSuspend(NULL);
    }

    while (1) {
        // Wait indefinitely for an item to appear in the queue.

    	if (xQueueReceive(g_sensor_data_queue, &received_sensor_data, portMAX_DELAY) == pdPASS) {
    	    PRINTF("[Sensors] T:%lu, AX:%.2f, AY:%.2f, AZ:%.2f, GX:%.2f, GY:%.2f, GZ:%.2f\r\n",
    	    		received_sensor_data.timestamp,
					received_sensor_data.acc_data.x, received_sensor_data.acc_data.y, received_sensor_data.acc_data.z,
					received_sensor_data.gyro_data.x, received_sensor_data.gyro_data.y, received_sensor_data.gyro_data.z);
        }

        if (xQueueReceive(g_controls_data_queue, &received_controls_data, portMAX_DELAY) == pdPASS) {
            PRINTF("[Controls] T:%lu, A:%.2f, E:%.2f, R:%.2f \r\n",
            		received_controls_data.timestamp, received_controls_data.aileron,
					received_controls_data.elevator, received_controls_data.rudder);
        }
    }
}





