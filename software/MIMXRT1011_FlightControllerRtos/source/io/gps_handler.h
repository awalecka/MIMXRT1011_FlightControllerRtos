/**
 * @file gps_handler.h
 * @brief Header for the GPS UART parsing task.
 */

#ifndef GPS_HANDLER_H
#define GPS_HANDLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback function to be registered with the board's UART1 RX ISR.
 *        Called every time a new byte is received from the GPS.
 * @param byte the received character.
 */
void gps_rx_callback(uint8_t byte);

/**
 * @brief FreeRTOS task that processes the incoming NMEA stream
 *        and updates the global GPS data queue.
 * @param pvParameters Task parameters (unused).
 */
void gpsTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // GPS_HANDLER_H
