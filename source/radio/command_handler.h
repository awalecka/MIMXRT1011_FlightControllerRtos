/**
 * @file command_handler.h
 * @brief Header for the IBUS FreeRTOS task with DMA/Idle Line support.
 */

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

// --- Public Function Prototypes ---

/**
 * @brief The FreeRTOS task that handles IBUS processing.
 * Defined with C++ linkage.
 * @param pvParameters Task parameters (unused).
 */
void command_handler_task(void *pvParameters);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LPUART4 Interrupt Service Routine.
 * Handles the Idle Line detection to trigger processing.
 * Must be extern "C" to link correctly with the startup vector table.
 */
void LPUART4_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif // COMMAND_HANDLER_H
