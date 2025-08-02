/**
 * @file command_handler.cpp
 * @brief Example FreeRTOS task for receiving and parsing IBUS data using a
 * binary semaphore for synchronization.
 *
 * This file provides a complete example of a FreeRTOS task that
 * initializes a UART for asynchronous reception using eDMA. The task is
 * signaled by the UART callback via a binary semaphore when a complete
 * IBUS message is received. This version is integrated with MCUXpresso
 * Config Tools by using the LPUART1_PERIPHERAL definition.
 */

#include "command_handler.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"       // Required for FreeRTOS semaphores
#include "Driver_USART.h"
#include "IbusProtocol.h" // Assumes IbusProtocol.h is in the include path
#include "peripherals.h"  // For LPUART1_PERIPHERAL definition
#include "fsl_debug_console.h"
#include "board.h"
#include "utils.h"
#include <vector>

// --- Global Variables ---
extern QueueHandle_t g_command_data_queue;

// Use the peripheral definition from the MCUXpresso configuration tools.
// This ensures that we are using the correctly configured LPUART instance.
static ARM_DRIVER_USART& ibusUart = LPUART1_IBUS_PERIPHERAL;

// IBUS protocol parser instance
static IbusProtocol ibusParser;

// Statically allocated buffer for the semaphore's control block
static StaticSemaphore_t xIbusSemaphoreBuffer;

// Binary semaphore to signal task from the ISR
static SemaphoreHandle_t ibus_rx_semaphore = NULL;

// Receive buffer for DMA
static uint8_t ibus_rx_buffer[IbusProtocol::MESSAGE_LENGTH];

// --- UART Callback Handler ---

/**
 * @brief Asynchronous UART event handler, called from an ISR.
 * @param event UART event flags from the CMSIS driver.
 */
void ibus_uart_callback(uint32_t event) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
        // A full buffer has been received. Give the semaphore to unblock the task.
        xSemaphoreGiveFromISR(ibus_rx_semaphore, &xHigherPriorityTaskWoken);
    }

    // Handle receive errors by aborting and restarting the reception
    if (event & ARM_USART_EVENT_RX_OVERFLOW ||
        event & ARM_USART_EVENT_RX_BREAK ||
        event & ARM_USART_EVENT_RX_PARITY_ERROR ||
        event & ARM_USART_EVENT_RX_FRAMING_ERROR) {
        ibusUart.Control(ARM_USART_ABORT_RECEIVE, 0);
        ibusUart.Receive(ibus_rx_buffer, IbusProtocol::MESSAGE_LENGTH);
    }

    // If xSemaphoreGiveFromISR unblocked a task, and that task has a higher
    // priority than the currently running task, yield the processor.
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// --- RTOS Task ---

/**
 * @brief The main function for the IBUS processing task.
 * @param pvParameters Task argument (not used in this example).
 */
void command_handler_task(void *pvParameters) {

    // Create the binary semaphore before it's used.
    ibus_rx_semaphore = xSemaphoreCreateBinaryStatic(&xIbusSemaphoreBuffer);
    if (ibus_rx_semaphore == NULL) {
        // Semaphore creation failed. This is a critical error.
        // Halt execution or handle the error appropriately.
        for(;;);
    }

    // Initialize the UART driver with our asynchronous callback
    // Note: The peripheral itself is initialized in BOARD_InitPeripherals().
    // This call just registers the callback.
    ibusUart.Initialize(ibus_uart_callback);

    // Power on the UART peripheral
    ibusUart.PowerControl(ARM_POWER_FULL);

    // Configure the UART for 115200 baud, 8-N-1
    ibusUart.Control(ARM_USART_MODE_ASYNCHRONOUS |
                     ARM_USART_DATA_BITS_8   |
                     ARM_USART_PARITY_NONE   |
                     ARM_USART_STOP_BITS_1   |
                     ARM_USART_FLOW_CONTROL_NONE, 115200);

    // Enable the UART receiver
    ibusUart.Control(ARM_USART_CONTROL_RX, 1);

    // Start the first asynchronous reception.
    ibusUart.Receive(ibus_rx_buffer, IbusProtocol::MESSAGE_LENGTH);

    PRINTF("Command handler configuration complete.\r\n");

    // --- Main Task Loop ---
    for (;;) {
        // Wait indefinitely to take the semaphore. The task will block here
        // with no CPU usage until the semaphore is given by the ISR.
        PRINTF("Command handler waiting for message.\r\n");

        if (xSemaphoreTake(ibus_rx_semaphore, portMAX_DELAY) == pdPASS) {
            // Semaphore received. Process the data in the buffer.
            for (uint32_t i = 0; i < IbusProtocol::MESSAGE_LENGTH; i++) {
                ibusParser.processByte(ibus_rx_buffer[i]);
            }

            // Check if a complete and valid message has been parsed.
            if (ibusParser.isLastMessageValid()) {

				const std::vector<unsigned short>& channels = ibusParser.getChannels();
				xQueueSend(g_command_data_queue, &channels, portMAX_DELAY);

				// Re-trigger the asynchronous reception for the next message.
				ibusUart.Receive(ibus_rx_buffer, IbusProtocol::MESSAGE_LENGTH);
			}
		}
	}
}
