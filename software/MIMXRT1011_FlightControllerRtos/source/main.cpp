/**
 * @file    main.cpp
 * @brief   Application entry point.
 */

#include "pin_mux.h"
#include "command_handler.h"
#include "logging_task.h"
#include "heartbeat_task.h"
#include "flight_controller.h"
#include "gps_handler.h"
#include "global.h"

/**
 * @brief Main Application Entry Point.
 * Initializes the board hardware, sets up FreeRTOS queues and tasks, and starts the scheduler.
 * @return Does not return.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    // Initialize custom hardware for IBUS via BSP
    BOARD_InitIBUS(ibus_idle_interrupt_callback);
    
    // Initialize GPS UART via BSP
    BOARD_InitGPS(gps_rx_callback);

    // Create Queues
    g_controls_data_queue = xQueueCreateStatic(CONTROLS_QUEUE_LENGTH, CONTROLS_QUEUE_ITEM_SIZE, ucControlsQueueStorageArea, &xControlsQueueControlBlock);
    g_command_data_queue = xQueueCreateStatic(COMMAND_QUEUE_LENGTH, COMMAND_QUEUE_ITEM_SIZE, ucCommandQueueStorageArea, &xCommandQueueControlBlock);
    g_state_change_request_queue = xQueueCreateStatic(STATE_CHANGE_QUEUE_LENGTH, STATE_CHANGE_QUEUE_ITEM_SIZE, ucStateChangeQueueStorageArea, &xStateChangeQueueControlBlock);
    g_imu_data_queue = xQueueCreateStatic(IMU_QUEUE_LENGTH, IMU_QUEUE_ITEM_SIZE, ucImuQueueStorageArea, &xImuQueueControlBlock);
    g_mag_data_queue = xQueueCreateStatic(MAG_QUEUE_LENGTH, MAG_QUEUE_ITEM_SIZE, ucMagQueueStorageArea, &xMagQueueControlBlock);
    g_gps_rx_queue = xQueueCreateStatic(GPS_RX_QUEUE_LENGTH, GPS_RX_QUEUE_ITEM_SIZE, ucGpsRxQueueStorageArea, &xGpsRxQueueControlBlock);
    g_gps_data_queue = xQueueCreateStatic(GPS_DATA_QUEUE_LENGTH, GPS_DATA_QUEUE_ITEM_SIZE, ucGpsDataQueueStorageArea, &xGpsDataQueueControlBlock);

    if (g_controls_data_queue == NULL || g_command_data_queue == NULL || g_state_change_request_queue == NULL || g_imu_data_queue == NULL || g_mag_data_queue == NULL || g_gps_rx_queue == NULL || g_gps_data_queue == NULL) {
        while(1);
    }

    // Create the State Manager Task
    g_state_manager_task_handle = xTaskCreateStatic(stateManagerTask, "StateMgrTask", STATE_MGR_STACK_SIZE, NULL, STATE_MANAGER_TASK_PRIORITY, xStateMgrStack, &xStateMgrTaskControlBlock);
    if (g_state_manager_task_handle == NULL) {
        while(1);
    }

    // Create the Other Tasks
    g_heartbeat_task_handle = xTaskCreateStatic(heartbeatTask, "HeartbeatTask", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIORITY, s_heartbeatStack, &s_heartbeatTaskControlBlock);
    g_command_handler_task_handle = xTaskCreateStatic(commandHandlerTask, "CommandTask", CMD_HANDLER_STACK_SIZE, NULL, COMMAND_HANDLER_TASK_PRIORITY, s_cmdHandlerStack, &s_cmdHandlerTaskControlBlock);
    g_logging_task_handle = xTaskCreateStatic(loggingTask, "LoggingTask", LOGGING_STACK_SIZE, NULL, LOGGING_TASK_PRIORITY, s_loggingStack, &s_loggingTaskControlBlock);
    s_imuTaskHandle = xTaskCreateStatic(imuTask, "ImuTask", IMU_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, s_imuStack, &s_imuTaskTCB);
    s_magTaskHandle = xTaskCreateStatic(magTask, "MagTask", MAG_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, s_magStack, &s_magTaskTCB);
    g_gps_task_handle = xTaskCreateStatic(gpsTask, "GpsTask", GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, s_gpsStack, &s_gpsTaskControlBlock);

    // Start Scheduler
    vTaskStartScheduler();

    // Should not reach here
    while(1);

    return 0;
}
