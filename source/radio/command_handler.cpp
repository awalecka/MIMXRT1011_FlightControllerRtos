/**
 * @file command_handler.cpp
 * @brief Implementation of the IBUS command handler using eDMA and Idle Line Interrupts.
 */
#include "command_handler.h"
#include "ibus_handler.hpp"
#include "flight_controller.h"
#include "fsl_lpuart.h"
#include <cstdint> // Required for uintptr_t

using namespace firmware::drivers;
using namespace firmware::protocols::ibus;

// Buffer must be aligned for DMA access
extern uint8_t g_dmaRxBuffer[IBUS_DMA_BUFFER_SIZE] __attribute__((aligned(4)));

// --- Static Data ---
static IbusHandler s_ibusHandler;
static size_t s_readIndex = 0;

// --- Private Function Prototypes ---
static void processReceivedData(void);

// --- Interrupt Service Routine (extern "C") ---
extern "C" void LPUART4_IRQHandler(void) {
    uint32_t statusFlags = LPUART_GetStatusFlags(IBUS_LPUART_INSTANCE);

    // Check for Idle Line Flag (Line high for >1 character time)
    if ((statusFlags & kLPUART_IdleLineFlag) != 0U) {
        LPUART_ClearStatusFlags(IBUS_LPUART_INSTANCE, kLPUART_IdleLineFlag);

        // Notify task
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (g_command_handler_task_handle != NULL) {
            vTaskNotifyGiveFromISR(g_command_handler_task_handle, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    // Check for Overrun (Buffer full in hardware FIFO)
    if ((statusFlags & kLPUART_RxOverrunFlag) != 0U) {
        LPUART_ClearStatusFlags(IBUS_LPUART_INSTANCE, kLPUART_RxOverrunFlag);
    }

    // Check if DMA Major Loop finished (Circular wrap-around event)
    __DSB();
}

// --- Main Task (C++ Linkage) ---

void commandHandlerTask(void* pvParameters) {
    (void)pvParameters;

    for (;;) {
        // Wait for Idle Line (End of Packet)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        processReceivedData();
    }
}

static void processReceivedData(void) {
    // Robust Index Calculation (AV Rule 96 Compliance):
    // We strictly avoid pointer arithmetic (ptrA - ptrB) to calculate the index.
    // Instead, we cast addresses to `uintptr_t` and perform integer arithmetic.

    // DADDR is a uint32_t register value, so we use static_cast (integer to integer)
    uintptr_t dmaAddr = static_cast<uintptr_t>(IBUS_DMA_BASE->TCD[IBUS_DMA_CHANNEL].DADDR);

    // g_dmaRxBuffer is a pointer, so we use reinterpret_cast (pointer to integer)
    uintptr_t baseAddr = reinterpret_cast<uintptr_t>(g_dmaRxBuffer);

    // Calculate index based on integer offset
    size_t writeIndex = (static_cast<size_t>(dmaAddr - baseAddr)) % IBUS_DMA_BUFFER_SIZE;

    while (s_readIndex != writeIndex) {

        // Array Access via Subscript (Compliant with AV Rule 96)
        uint8_t byte = g_dmaRxBuffer[s_readIndex];

        // --- Protocol State Machine ---
        static uint8_t s_assemblyBuffer[PACKET_LENGTH];
        static size_t s_assemblyIndex = 0;
        static bool s_synced = false;

        // Sync Logic: Find [0x20, 0x40]
        if (!s_synced) {
            if (s_assemblyIndex == 0) {
                if (byte == PACKET_LENGTH) { // 0x20
                    s_assemblyBuffer[s_assemblyIndex++] = byte;
                }
            } else if (s_assemblyIndex == 1) {
                if (byte == COMMAND_SERVO) { // 0x40
                    s_assemblyBuffer[s_assemblyIndex++] = byte;
                    s_synced = true;
                } else {
                     // Sync failed. Was this byte 0x20?
                     if (byte == PACKET_LENGTH) {
                         s_assemblyBuffer[0] = byte;
                         s_assemblyIndex = 1;
                     } else {
                         s_assemblyIndex = 0;
                     }
                }
            }
        } else {
            // Payload Collection
            s_assemblyBuffer[s_assemblyIndex++] = byte;

            if (s_assemblyIndex == PACKET_LENGTH) {
                // Parse Packet
                std::span<const uint8_t> packetSpan(s_assemblyBuffer, PACKET_LENGTH);

                if (s_ibusHandler.processBuffer(packetSpan)) {
                    const auto& channels = s_ibusHandler.getAllChannels();
                    RC_Channels_t rcData;
                    for(size_t i = 0; i < std::min((size_t)IBUS_MAX_CHANNELS, channels.size()); i++) {
                        rcData.channels[i] = channels[i];
                    }
                    xQueueOverwrite(g_command_data_queue, &rcData);
                }

                s_synced = false;
                s_assemblyIndex = 0;
            }
        }

        // Advance and Wrap Read Index
        s_readIndex++;
        if (s_readIndex >= IBUS_DMA_BUFFER_SIZE) {
            s_readIndex = 0;
        }
    }
}
