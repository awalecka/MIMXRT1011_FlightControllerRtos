#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

/**
 * @file common_types.h
 * @brief Shared POD types consumed by the GNC controllers.
 *
 * On the embedded target this header is part of the BSP.  The copy kept here
 * is used only by the host-side unit-test build; keep it in sync with the
 * firmware version by hand or via a symlink in the build system.
 */

 /** Full sensor data package delivered by the sensor fusion layer. */
struct FullSensorData {
    float rollDeg;           /**< Current roll  angle        [deg]. */
    float pitchDeg;          /**< Current pitch angle        [deg]. */
    float yawDeg;            /**< Current yaw   angle        [deg]. */
    float rollRateDps;       /**< Body roll  rate p          [deg/s]. */
    float pitchRateDps;      /**< Body pitch rate q          [deg/s]. */
    float yawRateDps;        /**< Body yaw   rate r          [deg/s]. */
    float trueAirspeedMs;    /**< Calibrated true airspeed   [m/s]. */
};

/** Normalised actuator demands in the range [-1, 1]. */
struct ActuatorOutput {
    float aileron;   /**< Roll  demand  [-1 = full left,  +1 = full right]. */
    float elevator;  /**< Pitch demand  [-1 = full down,  +1 = full up].    */
    float rudder;    /**< Yaw   demand  [-1 = full left,  +1 = full right]. */
};

#endif // COMMON_TYPES_H