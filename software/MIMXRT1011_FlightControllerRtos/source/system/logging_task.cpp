/**
 * @file logging_task.cpp
 * @brief Implements the data logging task for Telemetry via SiK Radio (UART3).
 */
#include "flight_controller.h"
#include "fsl_lpuart.h"
#include "peripherals.h"
#include <cstring>
#include <algorithm>
#include "system/logging_task.h"
#include "fsl_lpuart.h"

// Telemetry Protocol Definition
// --- Telemetry Protocol Definition ---
namespace {
    constexpr uint8_t TELEM_SYNC_1 = 0x55;
    constexpr uint8_t TELEM_SYNC_2 = 0xAA;

    struct __attribute__((packed)) TelemetryHeader {
        uint8_t sync1;
        uint8_t sync2;
        uint8_t type;
        uint8_t payloadLen;
    };

    constexpr size_t HEADER_SIZE = sizeof(TelemetryHeader);
    constexpr size_t CHECKSUM_SIZE = sizeof(uint8_t);
    // Max payload is 4 * uint16_t (8 bytes) or 3 * float (12 bytes)
    constexpr size_t MAX_PAYLOAD_SIZE = 12;
    constexpr size_t MAX_PACKET_SIZE = HEADER_SIZE + MAX_PAYLOAD_SIZE + CHECKSUM_SIZE;

    uint8_t calculateChecksum(const uint8_t* data, size_t len) {
        uint8_t xorSum = 0;
        for (size_t i = 0; i < len; i++) {
            xorSum ^= data[i];
        }
        return xorSum;
    }
}

/**
 * @brief Task that waits for control data and transmits it via UART3.
 */
void loggingTask(void *pvParameters) {
    LogMessage_t message;
    uint8_t txBuffer[MAX_PACKET_SIZE];

    while (true) {
        if (xQueueReceive(g_controls_data_queue, &message, portMAX_DELAY) == pdPASS) {

            size_t packetIndex = 0;
            size_t payloadSize = 0;
            void* pPayloadData = nullptr;

            // Prepare Payload based on Type
            if (message.type == LOG_TYPE_ATTITUDE) {
                payloadSize = sizeof(LogAttitude_t);
                pPayloadData = &message.data.attitude;
            } else if (message.type == LOG_TYPE_COMMANDS) {
                payloadSize = sizeof(LogCommands_t);
                pPayloadData = &message.data.commands;
            } else if (message.type == LOG_TYPE_MAG_RAW) {
                payloadSize = sizeof(LogMagRaw_t);
                pPayloadData = &message.data.magRaw;
            } else if (message.type == LOG_TYPE_CAL_STATUS) {
                payloadSize = sizeof(LogCalStatus_t);
                pPayloadData = &message.data.calStatus;
            } else if (message.type == LOG_TYPE_SYSTEM_STATUS) {
				payloadSize = sizeof(LogSystemStatus_t);
				pPayloadData = &message.data.sysStatus;
            } else {
                continue; // Unknown type
            }

            // Header
            TelemetryHeader header;
            header.sync1 = TELEM_SYNC_1;
            header.sync2 = TELEM_SYNC_2;
            header.type = message.type;
            header.payloadLen = (uint8_t)payloadSize;

            memcpy(&txBuffer[packetIndex], &header, HEADER_SIZE);
            packetIndex += HEADER_SIZE;

            // Payload
            memcpy(&txBuffer[packetIndex], pPayloadData, payloadSize);

            // Checksum
            uint8_t checksum = calculateChecksum((uint8_t*)pPayloadData, payloadSize);
            packetIndex += payloadSize;
            txBuffer[packetIndex++] = checksum;

            // Transmit
            LPUART_WriteBlocking(LPUART3, txBuffer, packetIndex);
        }
    }
}
