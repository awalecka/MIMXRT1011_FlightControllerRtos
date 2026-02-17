/**
 * @file utils.cpp
 * @brief Utility functions for range mapping.
 */
#include "utils.h"

float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
    if (inMax == inMin) {
        return outMin;
    }
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

unsigned short mapUshort(unsigned short value, unsigned short inMin, unsigned short inMax, unsigned short outMin, unsigned short outMax) {
    if (inMin == inMax) {
        return outMin;
    }

    long result = (static_cast<long>(value) - inMin) * (static_cast<long>(outMax) - outMin) /
                  (static_cast<long>(inMax) - inMin) + outMin;

    return static_cast<unsigned short>(result);
}
