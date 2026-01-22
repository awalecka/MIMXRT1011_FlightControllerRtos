#include "system/settings.h"

// --- FACTORY CALIBRATION DATA ---
// PASTE OUTPUT FROM runViz.py BELOW THIS LINE
// ==============================================================================

// Example default (Unit Sphere / No Offset) - Replace with Python Output
static constexpr float kFactoryHardIron[3] = { 0.0f, 0.0f, 0.0f };
static constexpr float kFactorySoftIron[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f
};

// ==============================================================================
// END FACTORY CALIBRATION DATA

// Initialize the active parameters with the factory defaults
MagCalibrationParams Settings::s_activeParams = {
    .hardIron = {
        kFactoryHardIron[0], kFactoryHardIron[1], kFactoryHardIron[2]
    },
    .softIron = {
        kFactorySoftIron[0], kFactorySoftIron[1], kFactorySoftIron[2],
        kFactorySoftIron[3], kFactorySoftIron[4], kFactorySoftIron[5],
        kFactorySoftIron[6], kFactorySoftIron[7], kFactorySoftIron[8]
    },
    .isValid = true // Mark valid so FlightController applies it on boot
};

void Settings::saveMagCal(const MagCalibrationParams& params) {
    s_activeParams = params;
    s_activeParams.isValid = true;
    // TODO: Implement Flash persistence here to save runtime RLS updates
}

bool Settings::loadMagCal(MagCalibrationParams& params) {
    if (s_activeParams.isValid) {
        params = s_activeParams;
        return true;
    }
    return false;
}
