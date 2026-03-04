/**
 * @file gps_handler.h
 * @brief Header for the GPS UART parsing task using DMA.
 */

#ifndef GPS_HANDLER_H
#define GPS_HANDLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Idle line callback from the GPS LPUART ISR.
 * Triggers processing in the GPS handler task.
 */
void gps_idle_interrupt_callback(void);

/**
 * @brief FreeRTOS task that processes the incoming NMEA stream
 * from the DMA buffer and updates the global GPS data queue.
 * @param pvParameters Task parameters (unused).
 */
void gpsTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // GPS_HANDLER_H
