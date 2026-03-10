#ifndef HEARTBEAT_TASK_H
#define HEARTBEAT_TASK_H

#include <FreeRTOS.h>
#include <task.h>

/**
 * @brief Manages the system heartbeat LED based on the current state frequency.
 */
class HeartbeatTask {
public:
    /**
     * @brief Constructs the HeartbeatTask.
     * @param heartbeatFrequency Reference to the volatile frequency variable.
     */
    HeartbeatTask(volatile TickType_t& heartbeatFrequency);

    /**
     * @brief Creates and starts the internal FreeRTOS task.
     */
    void start();

private:
    volatile TickType_t& m_heartbeatFrequency;

    static constexpr uint32_t STACK_SIZE = configMINIMAL_STACK_SIZE;
    StackType_t m_taskStack[STACK_SIZE];
    StaticTask_t m_taskControlBlock;
    TaskHandle_t m_taskHandle;

    static void taskEntry(void* pvParameters);
    void run();
};

#endif // HEARTBEAT_TASK_H
