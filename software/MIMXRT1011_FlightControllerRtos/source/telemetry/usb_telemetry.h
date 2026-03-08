// source/telemetry/usb_telemetry.h
#ifndef USB_TELEMETRY_H
#define USB_TELEMETRY_H

#include <cstdint>
#include <cstddef>

/**
 * @class UsbTelemetry
 * @brief Manages the USB CDC ACM virtual COM port for telemetry data.
 *
 * Provides initialization, connection state tracking, and data transmission
 * capabilities over the USB port as an alternative to the hardware UART.
 */
class UsbTelemetry {
public:
    /**
     * @brief Initializes the USB device stack and virtual COM port.
     *
     * Configures the USB PHY, sets up the CDC ACM class, configures interrupts,
     * and starts the USB device controller.
     */
    static void init();

    /**
     * @brief Checks if the USB virtual COM port is connected and DTE is active.
     * @return true if connected and ready to transmit, false otherwise.
     */
    static bool isConnected();

    /**
     * @brief Checks if a USB transmission is currently in progress.
     * @return true if busy, false if ready for a new transmission.
     */
    static bool isTxBusy();

    /**
     * @brief Sends data over the USB virtual COM port.
     * * The data is copied to an internal static buffer before transmission
     * to ensure memory stability during the DMA transfer.
     *
     * @param data Pointer to the buffer containing data to send.
     * @param length Number of bytes to send.
     */
    static void send(const uint8_t* data, std::size_t length);
};

#endif // USB_TELEMETRY_H
