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
 * @brief Writes a line of data to the log file.
 * @param data The sensor data to write.
 */
static void write_log_data(const sensor_log_data_vehicle_t* data) {
    // In a real implementation, you would format and write the data
    // f_printf(&g_log_file, "%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
    //          data->timestamp,
    //          data->acc_data.x, data->acc_data.y, data->acc_data.z,
    //          data->gyro_data.x, data->gyro_data.y, data->gyro_data.z);

    // For now, we just print to the console to show it's working
    PRINTF("[Logger] T:%u, AX:%.2f, AY:%.2f, AZ:%.2f, GX:%.2f, GY:%.2f, GZ:%.2f\r\n",
             data->timestamp,
             data->acc_data.x, data->acc_data.y, data->acc_data.z,
             data->gyro_data.x, data->gyro_data.y, data->gyro_data.z);
}


/**
 * @brief Task that waits for sensor data and writes it to a log file.
 */
void logging_task(void *pvParameters) {
	sensor_log_data_vehicle_t received_data;

    // Initialize the SD card. If it fails, suspend the task.
    if (!init_sd_card_and_logfile()) {
        PRINTF("[Logger] Error: Failed to initialize SD card. Halting logger.\r\n");
        vTaskSuspend(NULL);
    }

    while (1) {
        // Wait indefinitely for an item to appear in the queue.
        if (xQueueReceive(g_sensor_data_queue, &received_data, portMAX_DELAY) == pdPASS) {
            write_log_data(&received_data);
        }
    }
}
