#ifndef STATE_MANAGER_TASK_H
#define STATE_MANAGER_TASK_H

/**
 * @brief Task that handles state machine controller task.
 * * @param pvParameters Pointer to task parameters (unused).
 */
void stateManagerTask(void *pvParameters);

/**
 * @brief Task that handles background IMU reading.
 * * @param pvParameters Pointer to task parameters (unused).
 */
void imuTask(void *pvParameters);

/**
 * @brief Task that handles background Magnetometer reading.
 * * @param pvParameters Pointer to task parameters (unused).
 */
void magTask(void *pvParameters);

#endif // STATE_MANAGER_TASK_H
