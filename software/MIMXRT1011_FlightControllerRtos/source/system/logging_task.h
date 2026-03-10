#ifndef LOGGING_TASK_H
#define LOGGING_TASK_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/**
 * @brief Processes system log messages from a dedicated queue.
 */
class LoggingTask {
public:
    /**
     * @brief Constructs the LoggingTask.
     * @param controlsQueue Handle to the queue providing log messages.
     */
    LoggingTask(QueueHandle_t controlsQueue);

    /**
     * @brief Creates and starts the internal FreeRTOS task.
     */
    void start();

private:
    QueueHandle_t m_controlsQueue;

    static constexpr uint32_t STACK_SIZE = configMINIMAL_STACK_SIZE + 256;
    StackType_t m_taskStack[STACK_SIZE];
    StaticTask_t m_taskControlBlock;
    TaskHandle_t m_taskHandle;

    static void taskEntry(void* pvParameters);
    void run();
};

#endif // LOGGING_TASK_H
