#ifndef GPS_HANDLER_H
#define GPS_HANDLER_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "io/nmea.h"

/**
 * @brief Encapsulates the GPS processing task and handles DMA data.
 */
class GpsHandler {
public:
    /**
     * @brief Constructs the GPS handler with the required data queue.
     * @param gpsDataQueue Handle to the queue where parsed GPS data is sent.
     */
    GpsHandler(QueueHandle_t gpsDataQueue);

    /**
     * @brief Creates and starts the internal FreeRTOS task.
     */
    void start();

    /**
     * @brief Retrieves the underlying FreeRTOS task handle for ISR registration.
     * @return The task handle.
     */
    TaskHandle_t getTaskHandle() const;

private:
    QueueHandle_t m_gpsDataQueue;

    static constexpr uint32_t STACK_SIZE = configMINIMAL_STACK_SIZE + 512;
    StackType_t m_taskStack[STACK_SIZE];
    StaticTask_t m_taskControlBlock;
    TaskHandle_t m_taskHandle;

    static void taskEntry(void* pvParameters);
    void run();
};

/**
 * @brief Registers the task handle to be notified by the GPS UART idle line ISR.
 * @param taskHandle The handle of the GPS handler task.
 */
void gpsHandlerRegisterIsrTask(TaskHandle_t taskHandle);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ISR callback for the GPS UART idle line detection.
 */
void gps_idle_interrupt_callback(void);

#ifdef __cplusplus
}
#endif

#endif // GPS_HANDLER_H
