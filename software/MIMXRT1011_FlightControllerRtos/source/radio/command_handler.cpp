#include "radio/command_handler.h"
#include "board.h"
#include "fsl_cache.h"
#include <span>
#include <algorithm>

static TaskHandle_t s_ibusNotifiedTaskHandle = nullptr;

void commandHandlerRegisterIsrTask(TaskHandle_t taskHandle) {
    s_ibusNotifiedTaskHandle = taskHandle;
}

extern "C" void ibus_idle_interrupt_callback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (s_ibusNotifiedTaskHandle != nullptr) {
        vTaskNotifyGiveFromISR(s_ibusNotifiedTaskHandle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

CommandHandler::CommandHandler(QueueHandle_t commandQueue)
    : m_commandQueue(commandQueue),
      m_assemblyIndex(0),
      m_synced(false),
      m_taskHandle(nullptr) {
}

void CommandHandler::start() {
    m_taskHandle = xTaskCreateStatic(
        taskEntry,
        "CommandTask",
        STACK_SIZE,
        this,
        tskIDLE_PRIORITY + 2,
        m_taskStack,
        &m_taskControlBlock
    );
}

TaskHandle_t CommandHandler::getTaskHandle() const {
    return m_taskHandle;
}

void CommandHandler::taskEntry(void* pvParameters) {
    static_cast<CommandHandler*>(pvParameters)->run();
}

void CommandHandler::run() {
    size_t readIndex = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        DCACHE_InvalidateByRange(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(g_ibusDmaRxBuffer)), IBUS_DMA_BUFFER_SIZE);

        uintptr_t dmaAddr = static_cast<uintptr_t>(IBUS_DMA_BASE->TCD[IBUS_DMA_CHANNEL].DADDR);
        uintptr_t baseAddr = reinterpret_cast<uintptr_t>(g_ibusDmaRxBuffer);
        size_t writeIndex = (static_cast<size_t>(dmaAddr - baseAddr)) % IBUS_DMA_BUFFER_SIZE;

        while (readIndex != writeIndex) {
            uint8_t byte = g_ibusDmaRxBuffer[readIndex];

            // --- Protocol Sync Logic ---
            if (!m_synced) {
                if (m_assemblyIndex == 0) {
                    if (byte == firmware::protocols::ibus::PACKET_LENGTH) {
                        m_assemblyBuffer[m_assemblyIndex++] = byte;
                    }
                } else if (m_assemblyIndex == 1) {
                    if (byte == firmware::protocols::ibus::COMMAND_SERVO) {
                        m_assemblyBuffer[m_assemblyIndex++] = byte;
                        m_synced = true;
                    } else {
                         // Sync failed
                         if (byte == firmware::protocols::ibus::PACKET_LENGTH) {
                             m_assemblyBuffer[0] = byte;
                             m_assemblyIndex = 1;
                         } else {
                             m_assemblyIndex = 0;
                         }
                    }
                }
            } else {
                m_assemblyBuffer[m_assemblyIndex++] = byte;

                if (m_assemblyIndex == firmware::protocols::ibus::PACKET_LENGTH) {
                    std::span<const uint8_t> packetSpan(m_assemblyBuffer, firmware::protocols::ibus::PACKET_LENGTH);

                    if (m_ibusHandler.processBuffer(packetSpan)) {
                        const auto& channels = m_ibusHandler.getAllChannels();
                        RC_Channels_t rcData;

                        for (size_t i = 0; i < std::min((size_t)IBUS_MAX_CHANNELS, channels.size()); i++) {
                            rcData.channels[i] = channels[i];
                        }

                        if (m_commandQueue != nullptr) {
                            xQueueOverwrite(m_commandQueue, &rcData);
                        }
                    }

                    m_synced = false;
                    m_assemblyIndex = 0;
                }
            }

            readIndex++;
            if (readIndex >= IBUS_DMA_BUFFER_SIZE) {
                readIndex = 0;
            }
        }
    }
}
