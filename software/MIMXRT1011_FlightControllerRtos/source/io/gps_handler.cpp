#include "io/gps_handler.h"
#include "board.h"
#include "fsl_cache.h"

static TaskHandle_t s_gpsNotifiedTaskHandle = nullptr;

void gpsHandlerRegisterIsrTask(TaskHandle_t taskHandle) {
    s_gpsNotifiedTaskHandle = taskHandle;
}

extern "C" void gps_idle_interrupt_callback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (s_gpsNotifiedTaskHandle != nullptr) {
        vTaskNotifyGiveFromISR(s_gpsNotifiedTaskHandle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

GpsHandler::GpsHandler(QueueHandle_t gpsDataQueue)
    : m_gpsDataQueue(gpsDataQueue), m_taskHandle(nullptr) {
}

void GpsHandler::start() {
    m_taskHandle = xTaskCreateStatic(
        taskEntry,
        "GpsTask",
        STACK_SIZE,
        this,
        tskIDLE_PRIORITY + 2,
        m_taskStack,
        &m_taskControlBlock
    );
}

TaskHandle_t GpsHandler::getTaskHandle() const {
    return m_taskHandle;
}

void GpsHandler::taskEntry(void* pvParameters) {
    static_cast<GpsHandler*>(pvParameters)->run();
}

void GpsHandler::run() {
    size_t readIndex = 0;
    firmware::sensors::NmeaParser parser;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        DCACHE_InvalidateByRange(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(g_gpsDmaRxBuffer)), GPS_DMA_BUFFER_SIZE);

        uintptr_t dmaAddr = static_cast<uintptr_t>(GPS_DMA_BASE->TCD[GPS_DMA_CHANNEL].DADDR);
        uintptr_t baseAddr = reinterpret_cast<uintptr_t>(g_gpsDmaRxBuffer);
        size_t writeIndex = (static_cast<size_t>(dmaAddr - baseAddr)) % GPS_DMA_BUFFER_SIZE;

        while (readIndex != writeIndex) {
            uint8_t byte = g_gpsDmaRxBuffer[readIndex];

            if (parser.processByte(byte)) {
                firmware::sensors::GpsData data = parser.getLatestData();

                if (data.valid && m_gpsDataQueue != nullptr) {
                    data.lastUpdateTick = xTaskGetTickCount();
                    xQueueOverwrite(m_gpsDataQueue, &data);
                }
            }

            readIndex++;
            if (readIndex >= GPS_DMA_BUFFER_SIZE) {
                readIndex = 0;
            }
        }
    }
}
