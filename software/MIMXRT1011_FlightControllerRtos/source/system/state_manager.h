#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "controllers/flight_controller.h"
#include "utils/common_types.h"

/**
 * @brief Manages the main flight state machine.
 */
class StateManager {
public:
    /**
     * @brief Constructs the StateManager with explicit dependencies.
     * @param flightController Reference to the main flight controller.
     * @param stateQueue Handle to the state change request queue.
     * @param imuQueue Handle to the IMU data queue.
     * @param magQueue Handle to the Mag data queue.
     * @param flightState Reference to the shared flight state.
     * @param heartbeatFrequency Reference to the shared heartbeat frequency.
     */
    StateManager(FlightController& flightController,
                 QueueHandle_t stateQueue,
                 QueueHandle_t imuQueue,
                 QueueHandle_t magQueue,
                 volatile FlightState_t& flightState,
                 volatile TickType_t& heartbeatFrequency);

    /**
     * @brief Creates and starts the internal FreeRTOS tasks.
     */
    void start();

private:
    FlightController& m_flightController;
    QueueHandle_t m_stateQueue;
    QueueHandle_t m_imuQueue;
    QueueHandle_t m_magQueue;
    volatile FlightState_t& m_flightState;
    volatile TickType_t& m_heartbeatFrequency;

    static constexpr uint32_t STATE_STACK_SIZE = configMINIMAL_STACK_SIZE + 1024;
    static constexpr uint32_t SENSOR_STACK_SIZE = configMINIMAL_STACK_SIZE + 768;

    StackType_t m_stateStack[STATE_STACK_SIZE];
    StaticTask_t m_stateTaskControlBlock;
    TaskHandle_t m_stateTaskHandle;

    StackType_t m_imuStack[SENSOR_STACK_SIZE];
    StaticTask_t m_imuTaskControlBlock;
    TaskHandle_t m_imuTaskHandle;

    StackType_t m_magStack[SENSOR_STACK_SIZE];
    StaticTask_t m_magTaskControlBlock;
    TaskHandle_t m_magTaskHandle;

    static void stateTaskEntry(void* pvParameters);
    static void imuTaskEntry(void* pvParameters);
    static void magTaskEntry(void* pvParameters);

    void runState();
    void runImu();
    void runMag();

    FlightState_t runFlightState(TickType_t& lastWakeTime);
	FlightState_t runIdleState(TickType_t& lastWakeTime);
	FlightState_t runCalibrateState(TickType_t& lastWakeTime, uint32_t& entryTime);
	FlightState_t runFailsafeState();
};

#endif // STATE_MANAGER_H
