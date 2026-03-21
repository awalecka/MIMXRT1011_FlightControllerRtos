#include "telemetry/telemetry_manager.h"
#include "telemetry/usb_telemetry.h"

TelemetryManager::TelemetryManager()
    : telemetryQueue(nullptr), teleCounter(0), teleUpdateRate(10), statusDivider(0) {
}

void TelemetryManager::injectQueue(QueueHandle_t queueHandle) {
    telemetryQueue = queueHandle;
}

void TelemetryManager::update(const FullSensorData& sensorData,
                              const RC_Channels_t& rcData,
                              FlightState_t state) {
    if (telemetryQueue == nullptr) {
        return;
    }

    // Set divisor to 1 for 100Hz (USB) or 10 for 10Hz (UART)
    teleUpdateRate = UsbTelemetry::isConnected() ? 1 : 10;

    teleCounter++;
    if (teleCounter >= teleUpdateRate) {

        LogMessage_t attMsg;
        attMsg.type = LOG_TYPE_ATTITUDE;
        attMsg.data.attitude.roll = sensorData.rollDeg;
        attMsg.data.attitude.pitch = sensorData.pitchDeg;
        attMsg.data.attitude.yaw = sensorData.yawDeg;
        xQueueSend(telemetryQueue, &attMsg, 0);

        LogMessage_t cmdMsg;
        cmdMsg.type = LOG_TYPE_COMMANDS;
        cmdMsg.data.commands.aileron = rcData.channels[RC_CH_ROLL];
        cmdMsg.data.commands.elevator = rcData.channels[RC_CH_PITCH];
        cmdMsg.data.commands.rudder = rcData.channels[RC_CH_YAW];
        cmdMsg.data.commands.throttle = rcData.channels[RC_CH_THROTTLE];
        cmdMsg.data.commands.aux1 = rcData.channels[RC_CH_AUX1];
        cmdMsg.data.commands.aux2 = rcData.channels[RC_CH_AUX2];
        xQueueSend(telemetryQueue, &cmdMsg, 0);

        if (++statusDivider >= 5) {
            LogMessage_t sysMsg;
            sysMsg.type = LOG_TYPE_SYSTEM_STATUS;
            sysMsg.data.sysStatus.flightState = static_cast<uint8_t>(state);
            sysMsg.data.sysStatus.reserved = 0;
            sysMsg.data.sysStatus.cpuLoad = 0;
            xQueueSend(telemetryQueue, &sysMsg, 0);
            statusDivider = 0;
        }

        teleCounter = 0;
    }
}

void TelemetryManager::sendMagRaw(float x, float y, float z) {
    if (telemetryQueue == nullptr) {
        return;
    }

    LogMessage_t msg;
    msg.type = LOG_TYPE_MAG_RAW;
    msg.data.magRaw.x = x;
    msg.data.magRaw.y = y;
    msg.data.magRaw.z = z;
    xQueueSend(telemetryQueue, &msg, 0);
}

void TelemetryManager::sendCalStatus(bool success) {
    if (telemetryQueue == nullptr) {
        return;
    }

    LogMessage_t msg;
    msg.type = LOG_TYPE_CAL_STATUS;
    msg.data.calStatus.status = success ? 1 : 0;
    msg.data.calStatus.fitError = 0.0f;
    xQueueSend(telemetryQueue, &msg, 0);
}
