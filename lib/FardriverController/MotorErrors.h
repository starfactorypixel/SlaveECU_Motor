/*

*/

#pragma once

#include <inttypes.h>

// Если uint16_t то ошибка в конце...
typedef enum : int32_t
{
    MOTOR_ENCODER_ALARM = (1 << 0),
    ACCELERATION_PEDAL_ALARM = (1 << 1),
    CURRENT_PROTECT_RESTART = (1 << 2),
    PHASE_CURRENT_MUTATION = (1 << 3),
    VOLTAGE_ALARM = (1 << 4),
    BURGLAR_ALARM = (1 << 5),
    MOTOR_OVER_TEMP = (1 << 6),
    CONTROLLER_OVER_TEMP = (1 << 7),
    PHASE_CURRENT_OVERFLOW = (1 << 8),
    PHASE_ZERO_ALARM = (1 << 9),
    PHASE_SHORT_ALARM = (1 << 10),
    LINE_CURR_ZERO_ALARM = (1 << 11),
    MOSFET_HIGHSIDE_BRIDGE_ALARM = (1 << 12),
    MOSFET_LOWSIDE_BRIDGE_ALARM = (1 << 13),
    MOE_CURRENT_ALARM = (1 << 14),
    PARCING_ALARM = (1 << 15)
} motor_error_t;
