#ifndef UTILS_H
#define UTILS_H

/**
 * @brief Maps a floating-point value from one range to another.
 *
 * This function provides the same mapping functionality as the long version,
 * but for floating-point numbers (doubles).
 *
 * @param value The input floating-point value to map.
 * @param in_min The lower bound of the input range.
 * @param in_max The upper bound of the input range.
 * @param out_min The lower bound of the output range.
 * @param out_max The upper bound of the output range.
 * @return The mapped floating-point value within the output range.
 */
float map_float(float value, float in_min, float in_max, float out_min, float out_max);

/**
 * @brief Replicates the Arduino map() function for unsigned short values.
 *
 * It maps a number from one range to another.
 *
 * @param value The number to map.
 * @param in_min The lower bound of the value's current range.
 * @param in_max The upper bound of the value's current range.
 * @param out_min The lower bound of the value's target range.
 * @param out_max The upper bound of the value's target range.
 * @return The mapped value as an unsigned short.
 * @note Like the Arduino function, this implementation does not constrain the
 * return value to be within [out_min, out_max] if the input value is
 * outside [in_min, in_max]. Intermediate calculations are done with 'long'
 * to prevent overflow and handle range inversions correctly.
 */
unsigned short map_ushort(unsigned short value, unsigned short in_min, unsigned short in_max, unsigned short out_min, unsigned short out_max);

#endif // UTILS_H
