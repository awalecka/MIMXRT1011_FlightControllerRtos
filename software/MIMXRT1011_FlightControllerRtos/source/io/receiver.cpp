#include "io/receiver.h"
#include "utils/utils.h"
#include <algorithm>
#include <iterator>

void Receiver::init() {
    // Initialize all channels to a safe neutral position
    std::fill(std::begin(m_cachedRcData.channels), std::end(m_cachedRcData.channels), 1500);

    // Explicitly drop throttle to the minimum to prevent accidental motor spin
    m_cachedRcData.channels[RC_CH_THROTTLE] = 1000;
}

void Receiver::injectQueue(QueueHandle_t commandQueue) {
    m_commandQueue = commandQueue;
}

void Receiver::update() {
    // Abort early if dependencies have not been injected
    if (m_commandQueue == nullptr) {
        return;
    }

    RC_Channels_t tempBuffer;

    // Non-blocking peek/receive from the ISR-fed command queue
    if (xQueueReceive(m_commandQueue, &tempBuffer, 0) == pdPASS) {
        m_cachedRcData = tempBuffer;
    }
}

void Receiver::getSetpoint(Receiver::Setpoint& setpoint) {
    // Map raw PWM values to angular setpoints for the attitude controller
    setpoint.rollDeg = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_ROLL]), 1000.0f, 2000.0f, -20.0f, 20.0f);
    setpoint.pitchDeg = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_PITCH]), 1000.0f, 2000.0f, -20.0f, 20.0f);
    setpoint.throttle = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_THROTTLE]), 1000.0f, 2000.0f, 0.0f, 100.0f);
}

void Receiver::getStickInput(Receiver::StickInput& input) {
    // Map raw PWM values to normalized floats for generic pass-through uses
    input.roll = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_ROLL]), 1000.0f, 2000.0f, -1.0f, 1.0f);
    input.pitch = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_PITCH]), 1000.0f, 2000.0f, -1.0f, 1.0f);
    input.yaw = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_YAW]), 1000.0f, 2000.0f, -1.0f, 1.0f);
    input.throttle = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_THROTTLE]), 1000.0f, 2000.0f, 0.0f, 100.0f);
    input.gear = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_AUX2]), 1000.0f, 2000.0f, -1.0f, 1.0f);
}

uint16_t Receiver::getChannel(uint8_t channel) const {
    // Safely boundary check the requested channel index
    if (channel < IBUS_MAX_CHANNELS) {
        return m_cachedRcData.channels[channel];
    }
    return 0;
}

const RC_Channels_t& Receiver::getCachedData() const {
    return m_cachedRcData;
}

Receiver::CommandGesture Receiver::getActiveGesture() {
    // Extract raw channel data for gesture threshold evaluation
    uint16_t thr = getChannel(RC_CH_THROTTLE);
    uint16_t yaw = getChannel(RC_CH_YAW);
    uint16_t pitch = getChannel(RC_CH_PITCH);

    // Determine current stick limits based on configuration constants
    bool thrLow = (thr < GESTURE_STICK_LOW);
    bool yawRight = (yaw > GESTURE_STICK_HIGH);
    bool yawLeft = (yaw < GESTURE_STICK_LOW);
    bool pitchUp = (pitch > GESTURE_STICK_HIGH);

    CommandGesture detectedGesture = CommandGesture::None;

    // Evaluate combinations against recognized system gestures
    if (thrLow && yawLeft && pitchUp) {
        detectedGesture = CommandGesture::Calibrate;
    } else if (thrLow && yawRight) {
        detectedGesture = CommandGesture::Arm;
    } else if (thrLow && yawLeft) {
        detectedGesture = CommandGesture::Disarm;
    }

    if (detectedGesture != CommandGesture::None) {
        // Handle initiation of a new gesture or tracking of an ongoing one
        if (!m_gestureActive || m_candidateGesture != detectedGesture) {
            m_gestureStartTime = xTaskGetTickCount();
            m_gestureActive = true;
            m_candidateGesture = detectedGesture;
        } else if ((xTaskGetTickCount() - m_gestureStartTime) > pdMS_TO_TICKS(GESTURE_TIME_MS)) {
            // Gesture has been held for the required duration
            m_gestureActive = false;
            return m_candidateGesture;
        }
    } else {
        // Reset state immediately if sticks leave the target zones
        m_gestureActive = false;
        m_candidateGesture = CommandGesture::None;
    }

    return CommandGesture::None;
}
