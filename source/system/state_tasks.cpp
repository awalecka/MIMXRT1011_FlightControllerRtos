/**
 * @file tasks.cpp
 * @brief Implements the state-specific tasks for the flight controller.
 */
#include "system/state_tasks.h"
#include "flight_controller.h"
#include <board.h>
#include <utils.h>

// --- Gesture Detection Helper ---
static uint32_t s_gestureStartTime = 0;
static bool s_gestureActive = false;

// Stick Patterns (Mode 2):
// ARM: Throttle Low + Yaw Right
// CALIBRATE: Throttle Low + Yaw Left + Pitch Up
FlightState_t checkGestures(const Receiver::StickInput& s) {
    // Map stick inputs (approx -1.0 to 1.0, Throttle 0.0 to 100.0)
    // Thresholds: Low ~ 1050us (5%), High/Right ~ 1900us (>0.9)

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
            s_gestureActive = false; // Reset to prevent re-trigger
            return isArm ? STATE_FLIGHT : STATE_CALIBRATE;
        }
    } else {
        s_gestureActive = false;
    }

    return STATE_IDLE; // No change
}


/**
 * @brief Task for the FLIGHT state.
 */
void flightTask(void *pvParameters) {
    const TickType_t xFlightLoopFrequency = pdMS_TO_TICKS(10); // 100Hz
	TickType_t xLastWakeTime = xTaskGetTickCount();
    static uint32_t disarmStart = 0;

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFlightLoopFrequency);

        // --- Disarm Check (Throttle Low + Yaw Left) ---
        Receiver::StickInput s;
        g_flightController.getStickInput(s);
        if (s.throttle < 5.0f && s.yaw < -0.9f) {
             if (disarmStart == 0) disarmStart = xTaskGetTickCount();
             else if ((xTaskGetTickCount() - disarmStart) > pdMS_TO_TICKS(GESTURE_TIME_MS)) {
                 FlightState_t next = STATE_IDLE;
                 xQueueSend(g_state_change_request_queue, &next, 0);
                 disarmStart = 0;
             }
        } else {
            disarmStart = 0;
        }

        // Use the global controller instance
        USER_TIMING_ON();
        g_flightController.update();
        USER_TIMING_OFF();
    }
}

/**
 * @brief Task for the IDLE state.
 */
void idleTask(void *pvParameters) {

	TickType_t xLastWakeTime = xTaskGetTickCount();

	while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 100Hz update

        // 1. Run Controller (Keeps AHRS updated, sensors read, telemetry sending)
        // Note: Actuators are typically disabled or neutral in IDLE, handled by update() state logic if needed
        g_flightController.update();

        // 2. Check Gestures
        Receiver::StickInput sticks;
        g_flightController.getStickInput(sticks);

        FlightState_t next = checkGestures(sticks);

        if (next != STATE_IDLE) {
            xQueueSend(g_state_change_request_queue, &next, 0);
            vTaskDelay(pdMS_TO_TICKS(500)); // Wait for switch
        }
    }
}

/**
 * @brief Task for the CALIBRATE state.
 */
void calibrateTask(void *pvParameters) {

    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Auto-exit timeout
    uint32_t entryTime = xTaskGetTickCount();
    const uint32_t TIMEOUT = pdMS_TO_TICKS(60000); // 60s

	while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));

        g_flightController.update();

        // 1. Interactive Calibration Step
        g_flightController.calibrateMagnetometerStep();

        // 2. Check Exit (Throttle Low + Yaw Left) or Timeout
        Receiver::StickInput s;
        g_flightController.getStickInput(s);

        //bool exitGesture = (s.throttle < 5.0f && s.yaw < -0.9f);
        bool timedOut = (xTaskGetTickCount() - entryTime) > TIMEOUT;

        //if (exitGesture || timedOut) {
        if (timedOut) {

        	// Save
            //g_flightController.saveCalibration();

            // Return to IDLE
            FlightState_t new_state = STATE_IDLE;
            xQueueSend(g_state_change_request_queue, &new_state, 0);

            // Reset
            entryTime = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
