/**
 * @file command_handler.cpp
 * @brief Implementation of the IBUS command handler using eDMA and Idle Line Interrupts.
 */
#include "command_handler.h"
#include "ibus_handler.hpp"
#include "flight_controller.h"

// NXP SDK Drivers
#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "peripherals.h"

using namespace firmware::drivers;
using namespace firmware::protocols::ibus;

// --- Configuration Constants ---
// Increased to 128 to prevent "Full vs Empty" ambiguity (0==0)
// if exactly 64 bytes were received.
#define IBUS_DMA_BUFFER_SIZE    128U
#define IBUS_LPUART_INSTANCE    LPUART1
#define IBUS_LPUART_IRQn        LPUART1_IRQn
#define IBUS_DMA_BASE           DMA0
#define IBUS_DMAMUX_BASE        DMAMUX
#define IBUS_DMA_CHANNEL        0U
#define IBUS_DMA_SOURCE         kDmaRequestMuxLPUART1Rx

// --- Static Data ---
static IbusHandler s_ibusHandler;
// Buffer must be aligned for DMA access
static uint8_t s_dmaRxBuffer[IBUS_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
static size_t s_readIndex = 0;

// EDMA Handle
static edma_handle_t s_edmaHandle;

// --- Private Function Prototypes ---
static void processReceivedData(void);

// --- Interrupt Service Routine (extern "C") ---

extern "C" void LPUART1_IRQHandler(void) {
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
        // Optional: Add error logging here
    }

    // Check if DMA Major Loop finished (Circular wrap-around event)
    // In some configs, we might want to manually re-trigger here if not using Scatter-Gather
    // But we are relying on DLAST_SGA wrapping.

    __DSB();
}

// --- Main Task (C++ Linkage) ---

void commandHandlerTask(void* pvParameters) {
    (void)pvParameters;

    // Initialize DMAMUX
    DMAMUX_Init(IBUS_DMAMUX_BASE);
    DMAMUX_SetSource(IBUS_DMAMUX_BASE, IBUS_DMA_CHANNEL, IBUS_DMA_SOURCE);
    DMAMUX_EnableChannel(IBUS_DMAMUX_BASE, IBUS_DMA_CHANNEL);

    // Initialize EDMA
    edma_config_t edmaConfig;
    EDMA_GetDefaultConfig(&edmaConfig);
    EDMA_Init(IBUS_DMA_BASE, &edmaConfig);
    EDMA_CreateHandle(&s_edmaHandle, IBUS_DMA_BASE, IBUS_DMA_CHANNEL);

    // Configure EDMA TCD for Circular Buffer
    edma_transfer_config_t transferConfig;
    EDMA_PrepareTransfer(&transferConfig,
                         (void *)(uint32_t)LPUART_GetDataRegisterAddress(IBUS_LPUART_INSTANCE), // Source: UART Data Reg
                         1,                                                                     // Source Width: 1 byte
                         s_dmaRxBuffer,                                                         // Dest: RAM Buffer
                         1,                                                                     // Dest Width: 1 byte
                         1,                                                                     // Bytes per request (UART RX)
                         IBUS_DMA_BUFFER_SIZE,                                                  // Total bytes in Major Loop
                         kEDMA_PeripheralToMemory);

    EDMA_SubmitTransfer(&s_edmaHandle, &transferConfig);

    // Configure "Hardware Loop": When the buffer fills, subtract BufferSize from the address
    // to wrap back to the start.
    IBUS_DMA_BASE->TCD[IBUS_DMA_CHANNEL].DLAST_SGA = -((int32_t)IBUS_DMA_BUFFER_SIZE);

    // IMPORTANT: Prevent DMA from disabling itself after the major loop completes
    // This effectively keeps the ring buffer running forever.
    IBUS_DMA_BASE->TCD[IBUS_DMA_CHANNEL].CSR &= ~(DMA_CSR_DREQ_MASK);

    EDMA_StartTransfer(&s_edmaHandle);

    // Initialize LPUART
    lpuart_config_t lpuartConfig;
    LPUART_GetDefaultConfig(&lpuartConfig);
    lpuartConfig.baudRate_Bps = BAUD_RATE;
    lpuartConfig.enableTx = false;
    lpuartConfig.enableRx = true;
    lpuartConfig.rxFifoWatermark = 0; // DMA request on every byte

    LPUART_Init(IBUS_LPUART_INSTANCE, &lpuartConfig, BOARD_BOOTCLOCKRUN_UART_CLK_ROOT);

    // Enable Interrupts
    LPUART_EnableInterrupts(IBUS_LPUART_INSTANCE, kLPUART_IdleLineInterruptEnable);
    NVIC_SetPriority(IBUS_LPUART_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ(IBUS_LPUART_IRQn);

    // Enable Rx DMA
    LPUART_EnableRxDMA(IBUS_LPUART_INSTANCE, true);

    PRINTF("[IBUS] DMA Circular Buffer Task Started.\r\n");

    for (;;) {
        // Wait for Idle Line (End of Packet)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        processReceivedData();
    }
}

static void processReceivedData(void) {
    // Robust Index Calculation:
    // Read the current Destination Address directly from hardware TCD.
    uint32_t dmaAddr = (uint32_t)IBUS_DMA_BASE->TCD[IBUS_DMA_CHANNEL].DADDR;
    uint32_t baseAddr = (uint32_t)s_dmaRxBuffer;

    // Calculate index based on pointer offset
    size_t writeIndex = (dmaAddr - baseAddr) % IBUS_DMA_BUFFER_SIZE;

    // If s_readIndex == writeIndex, the buffer is EMPTY (or we processed everything).
    // Because we increased buffer size to 128, the "Full Wrap" collision is highly unlikely.

    while (s_readIndex != writeIndex) {

        uint8_t byte = s_dmaRxBuffer[s_readIndex];

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
