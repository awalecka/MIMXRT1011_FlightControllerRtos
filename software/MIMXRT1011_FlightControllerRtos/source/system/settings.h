#ifndef SETTINGS_H
#define SETTINGS_H

#include "common_types.h"

class Settings {
public:
    static void saveMagCal(const MagCalibrationParams& params);
    static bool loadMagCal(MagCalibrationParams& params);
private:
    static MagCalibrationParams s_activeParams;
};

#endif // SETTINGS_H
