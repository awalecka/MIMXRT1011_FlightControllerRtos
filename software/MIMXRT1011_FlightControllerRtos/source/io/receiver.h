#ifndef RECEIVER_H
#define RECEIVER_H

#include "utils/common_types.h"
#include <FreeRTOS.h>
#include <queue.h>

/**
 * @brief Manages receiver inputs, normalizes channels, and detects stick gestures.
 */
class Receiver {
public:
    /** @brief PWM threshold for a stick pulled fully low. */
    static constexpr uint16_t GESTURE_STICK_LOW = 1050;

    /** @brief PWM threshold for a stick pushed fully high. */
    static constexpr uint16_t GESTURE_STICK_HIGH = 1900;

    /** @brief Required hold duration in milliseconds to validate a gesture. */
    static constexpr uint32_t GESTURE_TIME_MS = 1000;

    /**
     * @brief Available semantic commands derived from stick movements.
     */
    enum class CommandGesture {
        None,
        Arm,
        Disarm,
        Calibrate
    };

    /**
     * @brief Structure representing the calculated flight control setpoints.
     */
    struct Setpoint {
        float rollDeg;
        float pitchDeg;
        float throttle;
    };

    /**
     * @brief Structure representing normalized stick inputs [-1.0 to 1.0].
     */
    struct StickInput {
        float roll;
        float pitch;
        float yaw;
        float throttle;
        float gear;
    };

    /**
     * @brief Initializes the receiver with default centered values.
     */
    void init();

    /**
     * @brief Injects the command queue dependency for fetching RC data.
     * @param commandQueue Handle to the FreeRTOS RC command queue.
     */
    void injectQueue(QueueHandle_t commandQueue);

    /**
     * @brief Polls the injected queue and updates internal state if new data exists.
     */
    void update();

    /**
     * @brief Calculates and returns the current flight setpoints based on RC input.
     * @param setpoint Structure to populate with calculated setpoints in degrees.
     */
    void getSetpoint(Setpoint& setpoint);

    /**
     * @brief Retrieves the normalized float stick inputs for raw processing.
     * @param input Structure to populate with normalized stick data.
     */
    void getStickInput(StickInput& input);

    /**
     * @brief Gets the raw integer PWM value of a specific receiver channel.
     * @param channel The channel index to retrieve.
     * @return The raw PWM value, or zero if the index is out of bounds.
     */
    uint16_t getChannel(uint8_t channel) const;

    /**
     * @brief Retrieves the entire cached RC channel structure.
     * @return Constant reference to the raw cached RC data.
     */
    const RC_Channels_t& getCachedData() const;

    /**
     * @brief Evaluates current stick positions to detect sustained command gestures.
     * @return The active gesture, or None if no gesture is sustained long enough.
     */
    CommandGesture getActiveGesture();

private:
    QueueHandle_t m_commandQueue = nullptr;
    RC_Channels_t m_cachedRcData;

    uint32_t m_gestureStartTime = 0;
    bool m_gestureActive = false;
    CommandGesture m_candidateGesture = CommandGesture::None;
};

#endif // RECEIVER_H
