/**
 * @file gps_handler.cpp
 * @brief FreeRTOS task handling the NMEA stream from the GPS module.
 */

#include "gps_handler.h"
#include "nmea.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Bring in global queues defined in main.cpp
extern QueueHandle_t g_gps_rx_queue;
extern QueueHandle_t g_gps_data_queue;

using namespace firmware::sensors;

extern "C" void gps_rx_callback(uint8_t byte) {
    if (g_gps_rx_queue != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(g_gps_rx_queue, &byte, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

extern "C" void gpsTask(void *pvParameters) {
    NmeaParser parser;
    uint8_t rxByte;

    while (1) {
        // Wait indefinitely for a byte from the UART ISR
        if (xQueueReceive(g_gps_rx_queue, &rxByte, portMAX_DELAY) == pdPASS) {
            
            // Feed the state machine
            if (parser.processByte(rxByte)) {
                // A complete packet was parsed successfully
                GpsData data = parser.getLatestData();
                
                if (data.valid) {
                    // Update system tick when data was received
                    data.lastUpdateTick = xTaskGetTickCount();
                    // Non-blocking overwrite of the latest data queue
                    xQueueOverwrite(g_gps_data_queue, &data);
                }
            }
        }
    }
}
