/**
 * @file imu_concepts.h
 * @brief Concepts for IMU sensor drivers to ensure compile-time interface compliance.
 */

#ifndef IMU_CONCEPTS_H
#define IMU_CONCEPTS_H

#include <concepts>
#include <cstdint>

/**
 * @brief Standard 3-axis float data structure for driver interchange.
 */
struct Axis3f {
    float x;
    float y;
    float z;
};

/**
 * @brief Concept for a 6-axis IMU (Accelerometer + Gyroscope).
 */
template<typename T>
concept SixAxisSensor = requires(T t, Axis3f& data) {
    { t.init() } -> std::same_as<int>;
    { t.readAccel(data) } -> std::same_as<int>;
    { t.readGyro(data) } -> std::same_as<int>;
};

/**
 * @brief Concept for a 3-axis Magnetometer.
 */
template<typename T>
concept MagnetometerSensor = requires(T t, Axis3f& data) {
    { t.init() } -> std::same_as<int>;
    { t.readMag(data) } -> std::same_as<int>;
};

#endif // IMU_CONCEPTS_H
