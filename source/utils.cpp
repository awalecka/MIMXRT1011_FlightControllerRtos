#include "utils.h"

float map_float(float value, float in_min, float in_max, float out_min, float out_max)
{
    if (in_max == in_min) {
        // Handle division by zero by returning the minimum of the output range.
        return out_min;
    }

    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned short map_ushort(unsigned short value, unsigned short in_min, unsigned short in_max, unsigned short out_min, unsigned short out_max)
{
    // Handle the edge case where the input range is zero to avoid division by zero.
    if (in_min == in_max) {
        // As a sensible default, return the minimum of the output range.
        return out_min;
    }

    // Perform linear interpolation using 'long' for intermediate calculations
    // to prevent overflow and handle negative results from range reversal.
    long result = ((long)value - in_min) * ((long)out_max - out_min) / ((long)in_max - in_min) + out_min;
    return (unsigned short)result;
}
