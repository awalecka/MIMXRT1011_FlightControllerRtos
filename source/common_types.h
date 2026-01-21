#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>
#include "drivers/lsm6dsox.h" // For mapping enums if needed

// --- Constants ---
#define IBUS_MAX_CHANNELS 14
#define RC_CH_ROLL     0
#define RC_CH_PITCH    1
#define RC_CH_THROTTLE 2
#define RC_CH_YAW      3
#define RC_CH_AUX1     4

// --- Enums ---
typedef enum {
    STATE_BOOT,
    STATE_IDLE,
    STATE_FLIGHT,
    STATE_FAILSAFE,
    STATE_CALIBRATE
} FlightState_t;

// --- Data Structures ---
struct RC_Channels_t {
    unsigned short channels[IBUS_MAX_CHANNELS];
};

struct FullSensorData {
    float rollDeg;
    float pitchDeg;
    float yawDeg;
    float rollRateDps;
    float pitchRateDps;
    float yawRateDps;
    float trueAirspeedMs;
};

struct ActuatorOutput {
    float aileron;
    float elevator;
    float rudder;
};

struct MagCalibrationParams {
    float hardIron[3];
    float softIron[9]; // 3x3 Flattened
    bool isValid;
};

// --- Logging Structures ---
typedef enum {
    LOG_TYPE_ATTITUDE   = 0x01,
    LOG_TYPE_COMMANDS   = 0x02,
    LOG_TYPE_MAG_RAW    = 0x03,
    LOG_TYPE_CAL_STATUS = 0x04,
    LOG_TYPE_SYSTEM_STATUS= 0x05
} LogType_t;

#define PACKED __attribute__((packed))

typedef struct {
    float roll;
    float pitch;
    float yaw;
} PACKED LogAttitude_t;

typedef struct {
    uint16_t aileron;
    uint16_t elevator;
    uint16_t rudder;
    uint16_t throttle;
} PACKED LogCommands_t;

typedef struct {
    float x;
    float y;
    float z;
} PACKED LogMagRaw_t;

typedef struct {
    uint8_t status;
    float fitError;
} PACKED LogCalStatus_t;

typedef struct {
    uint8_t flightState;
    uint8_t reserved;
    uint16_t cpuLoad;
} PACKED LogSystemStatus_t;

typedef struct {
    LogType_t type;
    union {
        LogAttitude_t attitude;
        LogCommands_t commands;
        LogMagRaw_t magRaw;
        LogCalStatus_t calStatus;
        LogSystemStatus_t sysStatus;
    } data;
} LogMessage_t;

#endif // COMMON_TYPES_H
