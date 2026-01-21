#ifndef TELEMETRY_MANAGER_H
#define TELEMETRY_MANAGER_H

#include "common_types.h"
#include <FreeRTOS.h>
#include <queue.h>

extern QueueHandle_t g_controls_data_queue; // External reference

class TelemetryManager {
public:
    TelemetryManager();

    /**
     * @brief Queues telemetry packets based on internal rate limiting.
     */
    void update(const FullSensorData& sensorData,
                const RC_Channels_t& rcData,
                FlightState_t state);

    /**
     * @brief Immediately queues a raw magnetometer packet (for calibration visualization).
     */
    void sendMagRaw(float x, float y, float z);

    /**
     * @brief Immediately queues a calibration status packet.
     */
    void sendCalStatus(bool success);

private:
    int teleCounter;
    int teleUpdateRate; // Divisor
    int statusDivider;
};

#endif // TELEMETRY_MANAGER_H
