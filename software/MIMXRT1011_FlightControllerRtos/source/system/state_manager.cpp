/**
 * @file state_manager.cpp
 * @brief Implements the main state machine controller task.
 */
#include "flight_controller.h"

// Include Task Definitions
#include "system/state_manager.h"
#include "system/heartbeat_task.h"
#include "system/logging_task.h"
#include "radio/command_handler.h"

static FlightState_t checkGestures(const Receiver::StickInput& s);

void sensorTask(void *pvParameters) {
    FlightController::SensorData data;
    while (true) {
        if (g_flightController.readSensors(data) == 0) {
            xQueueOverwrite(g_sensor_data_queue, &data);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// --- State Loop Helpers ---

/**
 * @brief Main execution iteration for the flight state.
 */
static FlightState_t runFlightState(TickType_t& lastWakeTime) {
    static uint32_t s_disarmStartTime = 0;

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(FlightController::LOOP_DT_MS));

    // --- Disarm Check (Throttle Low + Yaw Left) ---
    Receiver::StickInput s;
    g_flightController.getStickInput(s);
    if (s.throttle < 5.0f && s.yaw < -0.9f) {
         if (s_disarmStartTime == 0) {
             s_disarmStartTime = xTaskGetTickCount();
         } else if ((xTaskGetTickCount() - s_disarmStartTime) > pdMS_TO_TICKS(GESTURE_TIME_MS)) {
             s_disarmStartTime = 0;
             return STATE_IDLE;
         }
    } else {
        s_disarmStartTime = 0;
    }

    // Use the global controller instance
    USER_TIMING_ON();
    g_flightController.update();
    USER_TIMING_OFF();
    
    return STATE_FLIGHT;
}

/**
 * @brief Idle state execution iteration.
 */
static FlightState_t runIdleState(TickType_t& lastWakeTime) {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(FlightController::LOOP_DT_MS));

    // Run Controller (Keeps AHRS updated, sensors read, telemetry sending)
    g_flightController.update();

    // Check Gestures
    Receiver::StickInput sticks;
    g_flightController.getStickInput(sticks);
    FlightState_t nextState = checkGestures(sticks);

    if (nextState != STATE_IDLE) {
        vTaskDelay(pdMS_TO_TICKS(500)); // Wait for switch duration
        return nextState;
    }
    
    return STATE_IDLE;
}

/**
 * @brief Calibrate state execution iteration.
 */
static FlightState_t runCalibrateState(TickType_t& lastWakeTime, uint32_t& entryTime) {
    const uint32_t CALIBRATE_TIMEOUT_TICKS = pdMS_TO_TICKS(60000); // 60s
    
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));

    g_flightController.update();

    // Interactive Calibration Step
    g_flightController.calibrateMagnetometerStep();

    // Timeout Check
    if ((xTaskGetTickCount() - entryTime) > CALIBRATE_TIMEOUT_TICKS) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return STATE_IDLE;
    }
    
    return STATE_CALIBRATE;
}

/**
 * @brief Failsafe state iteration.
 */
static FlightState_t runFailsafeState() {
    // Blink fast, minimal activity
    vTaskDelay(pdMS_TO_TICKS(100));
    return STATE_FAILSAFE;
}

/**
 * @brief Main state machine task.
 * Handles state transitions and dispatches to specific state loops.
 */
void stateManagerTask(void *pvParameters) {

    // --- BOOT SEQUENCE ---
    g_flight_state = STATE_BOOT;

    // Use the FlightController to initialize all hardware
    if (g_flightController.init() != 0) {
        g_flight_state = STATE_FAILSAFE;
    } else {
        g_flight_state = STATE_IDLE;
    }

    TickType_t lastWakeTime = xTaskGetTickCount();
    FlightState_t previousState = g_flight_state;
    uint32_t calEntryTime = 0;

    // --- MAIN STATE LOOP ---
    while (true) {
        // Process any pending state changes from other tasks via Queue
        FlightState_t requestedState;
        if (xQueueReceive(g_state_change_request_queue, &requestedState, 0) == pdPASS) {
             g_flight_state = requestedState;
        }

        // On state change, initialize state specific variables
        if (g_flight_state != previousState) {
            lastWakeTime = xTaskGetTickCount(); // Reset timing

            // Set heartbeat frequency based on state
            switch (g_flight_state) {
                case STATE_FLIGHT: g_heartbeat_frequency = pdMS_TO_TICKS(250); break; // 2Hz
                case STATE_IDLE: g_heartbeat_frequency = pdMS_TO_TICKS(500); break; // 1Hz
                case STATE_CALIBRATE: 
                    g_heartbeat_frequency = pdMS_TO_TICKS(50); // Fast blink
                    calEntryTime = xTaskGetTickCount();
                    break; 
                case STATE_FAILSAFE: g_heartbeat_frequency = pdMS_TO_TICKS(125); break;
                default: break;
            }
            previousState = g_flight_state;
        }

        // Execute current state logic
        FlightState_t nextState;
        switch (g_flight_state) {
            case STATE_IDLE:
                nextState = runIdleState(lastWakeTime);
                break;
            case STATE_FLIGHT:
                nextState = runFlightState(lastWakeTime);
                break;
            case STATE_CALIBRATE:
                nextState = runCalibrateState(lastWakeTime, calEntryTime);
                break;
            case STATE_FAILSAFE:
                nextState = runFailsafeState();
                break;
            default:
                nextState = runIdleState(lastWakeTime);
                break;
        }

        // Apply internal state transition if requested by state function
        if (nextState != g_flight_state) {
            g_flight_state = nextState;
        }
    }
}

// --- Gesture Helper Implementation ---
static uint32_t s_gestureStartTime = 0;
static bool s_gestureActive = false;

/**
 * @brief Checks stick inputs for arming or calibration gestures.
 *        Stick Patterns (Mode 2):
 *         ARM: Throttle Low + Yaw Right
 *         CALIBRATE: Throttle Low + Yaw Left + Pitch Up
 * 
 * @param s Stick input data to evaluate.
 * @return The requested flight state or STATE_IDLE if no complete gesture is detected.
 */
static FlightState_t checkGestures(const Receiver::StickInput& s) {
    bool thrLow  = (s.throttle < 5.0f);
    bool yawRight = (s.yaw > 0.9f);
    bool yawLeft  = (s.yaw < -0.9f);
    bool pitchUp  = (s.pitch > 0.9f);

    bool isArm = thrLow && yawRight;
    bool isCal = thrLow && yawLeft && pitchUp;

    if (isArm || isCal) {
        uint32_t now = xTaskGetTickCount();
        if (!s_gestureActive) {
            s_gestureStartTime = now;
            s_gestureActive = true;
        } else if ((now - s_gestureStartTime) > pdMS_TO_TICKS(GESTURE_TIME_MS)) {
            s_gestureActive = false; // Reset
            return isArm ? STATE_FLIGHT : STATE_CALIBRATE;
        }
    } else {
        s_gestureActive = false;
    }

    return STATE_IDLE; 
}
