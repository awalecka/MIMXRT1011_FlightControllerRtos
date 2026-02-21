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
void commandHandlerTask(void *pvParameters);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Idle line callback from the LPUART ISR.
 * Triggers processing in the command handler task.
 */
void ibus_idle_interrupt_callback(void);

#ifdef __cplusplus
}
#endif

#endif // COMMAND_HANDLER_H
