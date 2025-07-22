/**
 * @file command_handler.h
 * @brief Header for the IBUS FreeRTOS task.
 *
 * This header file provides the function prototype for the IBUS command
 * handler task, allowing it to be created and started from other parts of
 * the application, such as the main application setup.
 */

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <stdint.h> // For uint32_t type

#ifdef __cplusplus
extern "C" {
#endif

// --- Public Function Prototypes ---

/**
 * @brief Asynchronous UART event handler, called from an ISR.
 * @param event UART event flags from the CMSIS driver.
 *
 * This function is called by the UART driver when a DMA-powered async
 * operation completes or an error occurs.
 */
void ibus_uart_callback(uint32_t event);


#ifdef __cplusplus
}
#endif

#endif // COMMAND_HANDLER_H
