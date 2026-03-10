#include "system/heartbeat_task.h"
#include "board.h"

HeartbeatTask::HeartbeatTask(volatile TickType_t& heartbeatFrequency)
    : m_heartbeatFrequency(heartbeatFrequency), m_taskHandle(nullptr) {
}

void HeartbeatTask::start() {
    m_taskHandle = xTaskCreateStatic(
        taskEntry,
        "HeartbeatTask",
        STACK_SIZE,
        this,
        tskIDLE_PRIORITY,
        m_taskStack,
        &m_taskControlBlock
    );
}

void HeartbeatTask::taskEntry(void* pvParameters) {
    static_cast<HeartbeatTask*>(pvParameters)->run();
}

void HeartbeatTask::run() {
    for (;;) {
        USER_LED_TOGGLE();
        vTaskDelay(m_heartbeatFrequency);
    }
}
