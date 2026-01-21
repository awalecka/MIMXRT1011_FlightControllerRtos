#include "system/settings.h"

MagCalibrationParams Settings::s_activeParams = { .isValid = false };

void Settings::saveMagCal(const MagCalibrationParams& params) {
    s_activeParams = params;
    s_activeParams.isValid = true;
    // TODO: Flash persistence
}

bool Settings::loadMagCal(MagCalibrationParams& params) {
    if (s_activeParams.isValid) {
        params = s_activeParams;
        return true;
    }
    return false;
}
