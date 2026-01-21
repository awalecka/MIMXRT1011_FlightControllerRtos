#ifndef RECEIVER_H
#define RECEIVER_H

#include "common_types.h"
#include <FreeRTOS.h>
#include <queue.h>

extern QueueHandle_t g_command_data_queue; // External reference to the queue

class Receiver {
public:
    struct Setpoint {
        float rollDeg;
        float pitchDeg;
        float throttle;
    };

    struct StickInput {
        float roll;
        float pitch;
        float yaw;
        float throttle;
    };

    void init();
    void update();
    void getSetpoint(Setpoint& setpoint);
    void getStickInput(StickInput& input);
    uint16_t getChannel(uint8_t channel) const;
    const RC_Channels_t& getCachedData() const;

private:
    RC_Channels_t m_cachedRcData;
};

#endif // RECEIVER_H
