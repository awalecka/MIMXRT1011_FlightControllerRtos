/**
 * @file logging_task.cpp
 * @brief Implements the data logging task for Telemetry via SiK Radio (UART3).
 */
#include "flight_controller.h"
#include "fsl_lpuart.h"
#include "peripherals.h"
#include <cstring>
#include <algorithm>

// --- Telemetry Protocol Definition ---
namespace {
    // Sync bytes
    constexpr uint8_t TELEM_SYNC_1 = 0x55;
    constexpr uint8_t TELEM_SYNC_2 = 0xAA;

    // Packet Types
    constexpr uint8_t PACKET_TYPE_ACTUATOR = 0x01;

    /**
     * @brief Fixed header for all telemetry packets.
     */
    struct __attribute__((packed)) TelemetryHeader {
        uint8_t sync1;
        uint8_t sync2;
        uint8_t type;
        uint8_t payloadLen;
    };

    /**
     * @brief Protocol sizing constants.
     * defining these here allows the buffer to resize automatically if the protocol changes.
     */
    constexpr size_t HEADER_SIZE = sizeof(TelemetryHeader);
    constexpr size_t CHECKSUM_SIZE = sizeof(uint8_t);

    // Define the maximum payload we intend to support (e.g., ActuatorOutput is ~12 bytes)
    // Setting a safe upper bound allows us to reuse this buffer for different packet types.
    constexpr size_t MAX_PAYLOAD_SIZE = sizeof(ActuatorOutput);

    // Total buffer size required
    constexpr size_t MAX_PACKET_SIZE = HEADER_SIZE + MAX_PAYLOAD_SIZE + CHECKSUM_SIZE;

    /**
     * @brief Calculates a simple XOR checksum.
     */
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
    ActuatorOutput surfaceCommands;

    // Statically allocate the buffer based on the calculated constants
    uint8_t txBuffer[MAX_PACKET_SIZE];

    while (true) {
        // Wait for telemetry data
        if (xQueueReceive(g_controls_data_queue, &surfaceCommands, portMAX_DELAY) == pdPASS) {

            // Compile-time check to ensure our payload fits in the buffer
            static_assert(sizeof(ActuatorOutput) <= MAX_PAYLOAD_SIZE, "ActuatorOutput exceeds telemetry buffer size!");

            size_t packetIndex = 0;

            // Construct Header
            TelemetryHeader header;
            header.sync1 = TELEM_SYNC_1;
            header.sync2 = TELEM_SYNC_2;
            header.type = PACKET_TYPE_ACTUATOR;
            header.payloadLen = (uint8_t)sizeof(ActuatorOutput);

            memcpy(&txBuffer[packetIndex], &header, HEADER_SIZE);
            packetIndex += HEADER_SIZE;

            // Copy Payload
            memcpy(&txBuffer[packetIndex], &surfaceCommands, sizeof(ActuatorOutput));

            // Calculate Checksum (Payload only)
            uint8_t checksum = calculateChecksum((uint8_t*)&surfaceCommands, sizeof(ActuatorOutput));
            packetIndex += sizeof(ActuatorOutput);

            // Append Checksum
            txBuffer[packetIndex++] = checksum;

            //bTransmit
            LPUART_WriteBlocking(LPUART3, txBuffer, packetIndex);
        }
    }
}
