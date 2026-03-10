#include "system/logging_task.h"
#include "telemetry/usb_telemetry.h"
#include "utils/common_types.h"
#include "fsl_lpuart.h"
#include "board.h"
#include <cstring>

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

LoggingTask::LoggingTask(QueueHandle_t controlsQueue)
    : m_controlsQueue(controlsQueue), m_taskHandle(nullptr) {
}

void LoggingTask::start() {
    m_taskHandle = xTaskCreateStatic(
        taskEntry,
        "LoggingTask",
        STACK_SIZE,
        this,
        tskIDLE_PRIORITY + 1,
        m_taskStack,
        &m_taskControlBlock
    );
}

void LoggingTask::taskEntry(void* pvParameters) {
    static_cast<LoggingTask*>(pvParameters)->run();
}

void LoggingTask::run() {
    LogMessage_t message;
    uint8_t txBuffer[MAX_PACKET_SIZE];

    UsbTelemetry::init();

    for (;;) {
        if (xQueueReceive(m_controlsQueue, &message, portMAX_DELAY) == pdPASS) {

            size_t packetIndex = 0;
            size_t payloadSize = 0;
            void* pPayloadData = nullptr;

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
                continue;
            }

            TelemetryHeader header;
            header.sync1 = TELEM_SYNC_1;
            header.sync2 = TELEM_SYNC_2;
            header.type = message.type;
            header.payloadLen = static_cast<uint8_t>(payloadSize);

            std::memcpy(&txBuffer[packetIndex], &header, HEADER_SIZE);
            packetIndex += HEADER_SIZE;

            std::memcpy(&txBuffer[packetIndex], pPayloadData, payloadSize);

            uint8_t checksum = calculateChecksum(static_cast<uint8_t*>(pPayloadData), payloadSize);
            packetIndex += payloadSize;
            txBuffer[packetIndex++] = checksum;

            if (UsbTelemetry::isConnected()) {
                if (!UsbTelemetry::isTxBusy()) {
                    UsbTelemetry::send(txBuffer, packetIndex);
                }
            } else {
                LPUART_WriteBlocking(TELE_LPUART_INSTANCE, txBuffer, packetIndex);
            }
        }
    }
}
