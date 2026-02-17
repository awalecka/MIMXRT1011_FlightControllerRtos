#ifndef UTILS_H
#define UTILS_H

/**
 * @brief Maps a floating-point value from one range to another.
 * @param value The input floating-point value to map.
 * @param inMin The lower bound of the input range.
 * @param inMax The upper bound of the input range.
 * @param outMin The lower bound of the output range.
 * @param outMax The upper bound of the output range.
 * @return The mapped floating-point value within the output range.
 */
float mapFloat(float value, float inMin, float inMax, float outMin, float outMax);

/**
 * @brief Maps a number from one range to another (unsigned short).
 * @param value The number to map.
 * @param inMin The lower bound of the value's current range.
 * @param inMax The upper bound of the value's current range.
 * @param outMin The lower bound of the value's target range.
 * @param outMax The upper bound of the value's target range.
 * @return The mapped value as an unsigned short.
 */
unsigned short mapUshort(unsigned short value, unsigned short inMin, unsigned short inMax, unsigned short outMin, unsigned short outMax);

#endif // UTILS_H
