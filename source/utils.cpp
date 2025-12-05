/**
 * @file utils.cpp
 * @brief Utility functions for range mapping.
 */
#include "utils.h"

float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
    if (inMax == inMin) {
        // Handle division by zero by returning the minimum of the output range.
        return outMin;
    }
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

unsigned short mapUshort(unsigned short value, unsigned short inMin, unsigned short inMax, unsigned short outMin, unsigned short outMax) {
    // Handle the edge case where the input range is zero to avoid division by zero.
    if (inMin == inMax) {
        // As a sensible default, return the minimum of the output range.
        return outMin;
    }

    // Perform linear interpolation using 'long' for intermediate calculations
    // to prevent overflow and handle negative results from range reversal.
    long result = ((long)value - inMin) * ((long)outMax - outMin) / ((long)inMax - inMin) + outMin;
    return (unsigned short)result;
}
