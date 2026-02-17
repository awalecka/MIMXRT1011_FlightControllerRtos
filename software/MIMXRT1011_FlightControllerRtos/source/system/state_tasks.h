#ifndef STATE_TASKS_H
#define STATE_TASKS_H

/**
 * @brief Task for the FLIGHT state (High frequency control loop).
 */
void flightTask(void *pvParameters);

/**
 * @brief Task for the IDLE state (Standby and gesture detection).
 */
void idleTask(void *pvParameters);

/**
 * @brief Task for the CALIBRATE state (Sensor calibration logic).
 */
void calibrateTask(void *pvParameters);

#endif // STATE_TASKS_H
