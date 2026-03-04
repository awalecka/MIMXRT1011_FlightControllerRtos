/**
 * @file gps_handler.cpp
 * @brief FreeRTOS task handling the NMEA stream from the GPS module via DMA.
 */

#include "gps_handler.h"
#include "nmea.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "board.h"
#include "fsl_cache.h"

extern QueueHandle_t g_gps_data_queue;
extern TaskHandle_t g_gps_task_handle;
extern uint8_t g_gpsDmaRxBuffer[];

using namespace firmware::sensors;

static size_t s_readIndex = 0;

extern "C" void gps_idle_interrupt_callback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (g_gps_task_handle != NULL) {
        vTaskNotifyGiveFromISR(g_gps_task_handle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

extern "C" void gpsTask(void *pvParameters) {
    (void)pvParameters;
    NmeaParser parser;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        DCACHE_InvalidateByRange(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(g_gpsDmaRxBuffer)), GPS_DMA_BUFFER_SIZE);

        uintptr_t dmaAddr = static_cast<uintptr_t>(GPS_DMA_BASE->TCD[GPS_DMA_CHANNEL].DADDR);
        uintptr_t baseAddr = reinterpret_cast<uintptr_t>(g_gpsDmaRxBuffer);
        size_t writeIndex = (static_cast<size_t>(dmaAddr - baseAddr)) % GPS_DMA_BUFFER_SIZE;

        while (s_readIndex != writeIndex) {
            uint8_t byte = g_gpsDmaRxBuffer[s_readIndex];

            if (parser.processByte(byte)) {
                GpsData data = parser.getLatestData();
                
                if (data.valid) {
                    data.lastUpdateTick = xTaskGetTickCount();
                    xQueueOverwrite(g_gps_data_queue, &data);
                }
            }

            s_readIndex++;
            if (s_readIndex >= GPS_DMA_BUFFER_SIZE) {
                s_readIndex = 0;
            }
        }
    }
}
