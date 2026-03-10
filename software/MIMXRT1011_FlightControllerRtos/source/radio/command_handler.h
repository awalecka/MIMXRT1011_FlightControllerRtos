#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <cstdint>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "radio/ibus_handler.hpp"
#include "utils/common_types.h"

/**
 * @brief Handles incoming IBUS RC packets via DMA.
 */
class CommandHandler {
public:
    /**
     * @brief Constructs the CommandHandler with injected queue dependency.
     * @param commandQueue Handle to the command data queue.
     */
    CommandHandler(QueueHandle_t commandQueue);

    /**
     * @brief Creates and starts the internal FreeRTOS task.
     */
    void start();

    /**
     * @brief Retrieves the underlying FreeRTOS task handle for ISR registration.
     * @return TaskHandle_t The task handle.
     */
    TaskHandle_t getTaskHandle() const;

private:
    QueueHandle_t m_commandQueue;
    firmware::drivers::IbusHandler m_ibusHandler;

    // Protocol Assembly State
    uint8_t m_assemblyBuffer[firmware::protocols::ibus::PACKET_LENGTH];
    size_t m_assemblyIndex;
    bool m_synced;

    static constexpr uint32_t STACK_SIZE = configMINIMAL_STACK_SIZE + 256;
    StackType_t m_taskStack[STACK_SIZE];
    StaticTask_t m_taskControlBlock;
    TaskHandle_t m_taskHandle;

    static void taskEntry(void* pvParameters);
    void run();
};

/**
 * @brief Registers the task handle to be notified by the idle line ISR.
 * @param taskHandle The handle of the command handler task.
 */
void commandHandlerRegisterIsrTask(TaskHandle_t taskHandle);

#ifdef __cplusplus
extern "C" {
#endif

void ibus_idle_interrupt_callback(void);

#ifdef __cplusplus
}
#endif

#endif // COMMAND_HANDLER_H
