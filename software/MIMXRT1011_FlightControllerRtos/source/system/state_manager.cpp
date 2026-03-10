#include "system/state_manager.h"

StateManager::StateManager(FlightController& flightController,
                           QueueHandle_t stateQueue,
                           QueueHandle_t imuQueue,
                           QueueHandle_t magQueue,
                           volatile FlightState_t& flightState,
                           volatile TickType_t& heartbeatFrequency)
    : m_flightController(flightController),
      m_stateQueue(stateQueue),
      m_imuQueue(imuQueue),
      m_magQueue(magQueue),
      m_flightState(flightState),
      m_heartbeatFrequency(heartbeatFrequency),
      m_stateTaskHandle(nullptr),
      m_imuTaskHandle(nullptr),
      m_magTaskHandle(nullptr) {
}

void StateManager::start() {
    m_stateTaskHandle = xTaskCreateStatic(stateTaskEntry, "StateMgrTask", STATE_STACK_SIZE, this, tskIDLE_PRIORITY + 4, m_stateStack, &m_stateTaskControlBlock);
    m_imuTaskHandle = xTaskCreateStatic(imuTaskEntry, "ImuTask", SENSOR_STACK_SIZE, this, tskIDLE_PRIORITY + 3, m_imuStack, &m_imuTaskControlBlock);
    m_magTaskHandle = xTaskCreateStatic(magTaskEntry, "MagTask", SENSOR_STACK_SIZE, this, tskIDLE_PRIORITY + 3, m_magStack, &m_magTaskControlBlock);
}

void StateManager::stateTaskEntry(void* pvParameters) {
    static_cast<StateManager*>(pvParameters)->runState();
}

void StateManager::imuTaskEntry(void* pvParameters) {
    static_cast<StateManager*>(pvParameters)->runImu();
}

void StateManager::magTaskEntry(void* pvParameters) {
    static_cast<StateManager*>(pvParameters)->runMag();
}

void StateManager::runImu() {
    FlightController::ImuData data;
    while (true) {
        if (m_flightController.readImu(data) == 0) {
            xQueueOverwrite(m_imuQueue, &data);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void StateManager::runMag() {
    FlightController::MagData data;
    while (true) {
        if (m_flightController.readMag(data) == 0) {
            xQueueOverwrite(m_magQueue, &data);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void StateManager::runState() {
    m_flightState = STATE_BOOT;

    if (m_flightController.init() != 0) {
        m_flightState = STATE_FAILSAFE;
    } else {
        m_flightState = STATE_IDLE;
    }

    TickType_t lastWakeTime = xTaskGetTickCount();
    FlightState_t previousState = m_flightState;
    uint32_t calEntryTime = 0;

    while (true) {
        FlightState_t requestedState;
        if (xQueueReceive(m_stateQueue, &requestedState, 0) == pdPASS) {
             m_flightState = requestedState;
        }

        if (m_flightState != previousState) {
            lastWakeTime = xTaskGetTickCount();
            switch (m_flightState) {
                case STATE_FLIGHT: m_heartbeatFrequency = pdMS_TO_TICKS(250); break;
                case STATE_IDLE: m_heartbeatFrequency = pdMS_TO_TICKS(500); break;
                case STATE_CALIBRATE:
                    m_heartbeatFrequency = pdMS_TO_TICKS(50);
                    calEntryTime = xTaskGetTickCount();
                    break;
                case STATE_FAILSAFE: m_heartbeatFrequency = pdMS_TO_TICKS(125); break;
                default: break;
            }
            previousState = m_flightState;
        }

        FlightState_t nextState;
        switch (m_flightState) {
            case STATE_IDLE: nextState = runIdleState(lastWakeTime); break;
            case STATE_FLIGHT: nextState = runFlightState(lastWakeTime); break;
            case STATE_CALIBRATE: nextState = runCalibrateState(lastWakeTime, calEntryTime); break;
            case STATE_FAILSAFE: nextState = runFailsafeState(); break;
            default: nextState = runIdleState(lastWakeTime); break;
        }

        if (nextState != m_flightState) {
            m_flightState = nextState;
        }
    }
}

FlightState_t StateManager::runFlightState(TickType_t& lastWakeTime) {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(FlightController::LOOP_DT_MS));

    if (m_flightController.getActiveGesture() == Receiver::CommandGesture::Disarm) {
        return STATE_IDLE;
    }

    m_flightController.update();
    return STATE_FLIGHT;
}

FlightState_t StateManager::runIdleState(TickType_t& lastWakeTime) {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(FlightController::LOOP_DT_MS));
    m_flightController.update();

    Receiver::CommandGesture gesture = m_flightController.getActiveGesture();

    if (gesture == Receiver::CommandGesture::Arm) {
        vTaskDelay(pdMS_TO_TICKS(500));
        return STATE_FLIGHT;
    } else if (gesture == Receiver::CommandGesture::Calibrate) {
        vTaskDelay(pdMS_TO_TICKS(500));
        return STATE_CALIBRATE;
    }

    return STATE_IDLE;
}

FlightState_t StateManager::runCalibrateState(TickType_t& lastWakeTime, uint32_t& entryTime) {
    const uint32_t CALIBRATE_TIMEOUT_TICKS = pdMS_TO_TICKS(60000);
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));

    m_flightController.update();
    m_flightController.calibrateMagnetometerStep();

    if ((xTaskGetTickCount() - entryTime) > CALIBRATE_TIMEOUT_TICKS) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return STATE_IDLE;
    }
    return STATE_CALIBRATE;
}

FlightState_t StateManager::runFailsafeState() {
    vTaskDelay(pdMS_TO_TICKS(100));
    return STATE_FAILSAFE;
}
