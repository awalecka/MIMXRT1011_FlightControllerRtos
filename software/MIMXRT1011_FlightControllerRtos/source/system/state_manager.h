#ifndef STATE_MANAGER_TASK_H
#define STATE_MANAGER_TASK_H

/**
 * @brief Task that handles state machine controller task.
 */
void stateManagerTask(void *pvParameters);

/**
 * @brief Task that handles background sensor reading.
 */
void sensorTask(void *pvParameters);

#endif // STATE_MANAGER_TASK_H
