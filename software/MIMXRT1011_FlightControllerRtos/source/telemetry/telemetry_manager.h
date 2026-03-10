#ifndef TELEMETRY_MANAGER_H
#define TELEMETRY_MANAGER_H

#include <FreeRTOS.h>
#include <queue.h>
#include "common_types.h"

/**
 * @brief Manages the gathering and transmission of telemetry data.
 */
class TelemetryManager {
public:
    /**
     * @brief Constructs the telemetry manager with default rate limiters.
     */
    TelemetryManager();

    /**
     * @brief Injects the telemetry data queue dependency.
     * @param telemetryQueue Handle to the queue where telemetry output is sent.
     */
    void injectQueue(QueueHandle_t telemetryQueue);

    /**
     * @brief Queues telemetry packets based on internal rate limiting.
     * @param sensorData The processed sensor and attitude data.
     * @param rcData The current RC channel inputs.
     * @param state The current system flight state.
     */
    void update(const FullSensorData& sensorData,
                const RC_Channels_t& rcData,
                FlightState_t state);

    /**
     * @brief Immediately queues a raw magnetometer packet.
     * @param x X-axis raw value.
     * @param y Y-axis raw value.
     * @param z Z-axis raw value.
     */
    void sendMagRaw(float x, float y, float z);

    /**
     * @brief Immediately queues a calibration status packet.
     * @param success True if calibration was successful, false otherwise.
     */
    void sendCalStatus(bool success);

private:
    QueueHandle_t telemetryQueue;
    int teleCounter;
    int teleUpdateRate;
    int statusDivider;
};

#endif // TELEMETRY_MANAGER_H
