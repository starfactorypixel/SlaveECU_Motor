/*
    Флаги ошибок контроллера.
    Пакет: 0x00, Данные: D6:D7.
*/

#pragma once

#include <inttypes.h>

typedef enum : uint16_t
{
    MOTOR_ENCODER_ALARM = (1U << 0),
    ACCELERATION_PEDAL_ALARM = (1U << 1),
    CURRENT_PROTECT_RESTART = (1U << 2),
    PHASE_CURRENT_MUTATION = (1U << 3),
    VOLTAGE_ALARM = (1U << 4),
    BURGLAR_ALARM = (1U << 5),
    MOTOR_OVER_TEMP = (1U << 6),
    CONTROLLER_OVER_TEMP = (1U << 7),
    PHASE_CURRENT_OVERFLOW = (1U << 8),
    PHASE_ZERO_ALARM = (1U << 9),
    PHASE_SHORT_ALARM = (1U << 10),
    LINE_CURR_ZERO_ALARM = (1U << 11),
    MOSFET_HIGHSIDE_BRIDGE_ALARM = (1U << 12),
    MOSFET_LOWSIDE_BRIDGE_ALARM = (1U << 13),
    MOE_CURRENT_ALARM = (1U << 14),
    PARCING_ALARM = (1U << 15)
} motor_error_t;
