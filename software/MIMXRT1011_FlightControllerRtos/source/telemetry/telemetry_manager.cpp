#include "telemetry/telemetry_manager.h"

TelemetryManager::TelemetryManager()
    : teleCounter(0), teleUpdateRate(20), statusDivider(0) {}

void TelemetryManager::update(const FullSensorData& sensorData,
                              const RC_Channels_t& rcData,
                              FlightState_t state) {
    teleCounter++;
    if (teleCounter >= teleUpdateRate) {
        // 1. Attitude Packet
        LogMessage_t attMsg;
        attMsg.type = LOG_TYPE_ATTITUDE;
        attMsg.data.attitude.roll = sensorData.rollDeg;
        attMsg.data.attitude.pitch = sensorData.pitchDeg;
        attMsg.data.attitude.yaw = sensorData.yawDeg;
        xQueueSend(g_controls_data_queue, &attMsg, 0);

        // 2. Commands Packet (Raw RC)
        LogMessage_t cmdMsg;
        cmdMsg.type = LOG_TYPE_COMMANDS;
        cmdMsg.data.commands.aileron = rcData.channels[RC_CH_ROLL];
        cmdMsg.data.commands.elevator = rcData.channels[RC_CH_PITCH];
        cmdMsg.data.commands.rudder = rcData.channels[RC_CH_YAW];
        cmdMsg.data.commands.throttle = rcData.channels[RC_CH_THROTTLE];
        xQueueSend(g_controls_data_queue, &cmdMsg, 0);

        // 3. System Status (Low Priority: 1/5th rate)
        if (++statusDivider >= 5) {
            LogMessage_t sysMsg;
            sysMsg.type = LOG_TYPE_SYSTEM_STATUS;
            sysMsg.data.sysStatus.flightState = (uint8_t)state;
            sysMsg.data.sysStatus.reserved = 0;
            sysMsg.data.sysStatus.cpuLoad = 0;
            xQueueSend(g_controls_data_queue, &sysMsg, 0);
            statusDivider = 0;
        }

        teleCounter = 0;
    }
}

void TelemetryManager::sendMagRaw(float x, float y, float z) {
    LogMessage_t msg;
    msg.type = LOG_TYPE_MAG_RAW;
    msg.data.magRaw.x = x;
    msg.data.magRaw.y = y;
    msg.data.magRaw.z = z;
    xQueueSend(g_controls_data_queue, &msg, 0);
}

void TelemetryManager::sendCalStatus(bool success) {
    LogMessage_t msg;
    msg.type = LOG_TYPE_CAL_STATUS;
    msg.data.calStatus.status = success ? 1 : 0;
    msg.data.calStatus.fitError = 0.0f;
    xQueueSend(g_controls_data_queue, &msg, 0);
}
