#include "io/receiver.h"
#include "utils/utils.h"
#include <algorithm>
#include <iterator>

void Receiver::init() {
    std::fill(std::begin(m_cachedRcData.channels), std::end(m_cachedRcData.channels), 1500);
    m_cachedRcData.channels[RC_CH_THROTTLE] = 1000;
}

void Receiver::update() {
    RC_Channels_t tempBuffer;
    // Non-blocking peek/receive from the ISR-fed queue
    if (xQueueReceive(g_command_data_queue, &tempBuffer, 0) == pdPASS) {
        m_cachedRcData = tempBuffer;
    }
}

void Receiver::getSetpoint(Receiver::Setpoint& setpoint) {
    setpoint.rollDeg = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_ROLL]), 1000.0f, 2000.0f, -20.0f, 20.0f);
    setpoint.pitchDeg = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_PITCH]), 1000.0f, 2000.0f, -20.0f, 20.0f);
    setpoint.throttle = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_THROTTLE]), 1000.0f, 2000.0f, 0.0f, 100.0f);
}

void Receiver::getStickInput(Receiver::StickInput& input) {
    input.roll = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_ROLL]), 1000.0f, 2000.0f, -1.0f, 1.0f);
    input.pitch = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_PITCH]), 1000.0f, 2000.0f, -1.0f, 1.0f);
    input.yaw = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_YAW]), 1000.0f, 2000.0f, -1.0f, 1.0f);
    input.throttle = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_THROTTLE]), 1000.0f, 2000.0f, 0.0f, 100.0f);
}

uint16_t Receiver::getChannel(uint8_t channel) const {
    if (channel < IBUS_MAX_CHANNELS) {
        return m_cachedRcData.channels[channel];
    }
    return 0;
}

const RC_Channels_t& Receiver::getCachedData() const {
    return m_cachedRcData;
}
