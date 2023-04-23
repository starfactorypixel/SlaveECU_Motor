#ifndef MOTORECU_LOW_LEVEL_ABSTRACTION_H
#define MOTORECU_LOW_LEVEL_ABSTRACTION_H

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <cstring>

#include "CANLibrary.h"
#include "MotorData.h"

#ifdef __cplusplus
extern "C"
{
#endif

    enum motor_ecu_can_object_id_t : uint16_t
    {
        MOTOR_CANO_ID_BLOCK_INFO = 0x0100,
        MOTOR_CANO_ID_BLOCK_HEALTH = 0x0101,
        MOTOR_CANO_ID_BLOCK_CFG = 0x0102,
        MOTOR_CANO_ID_BLOCK_ERROR = 0x0103,
        MOTOR_CANO_ID_CONTROLLER_ERRORS = 0x0104,
        MOTOR_CANO_ID_RPM = 0x0105,
        MOTOR_CANO_ID_SPEED = 0x0106,
        MOTOR_CANO_ID_VOLTAGE = 0x0107,
        MOTOR_CANO_ID_CURRENT = 0x0108,
        MOTOR_CANO_ID_POWER = 0x0109,
        MOTOR_CANO_ID_GEAR_AND_ROLL = 0x010A,
        MOTOR_CANO_ID_TEMPERATURE = 0x010B,
        MOTOR_CANO_ID_ODOMETER = 0x010C,
    };

    // 0x0100	BlockInfo
    typedef block_info_t motor_block_info_t;

    // 0x0101	BlockHealth
    typedef block_health_t motor_block_health_t;

    // 0x0102	BlockCfg
    typedef block_cfg_t motor_block_cfg_t;

    // 0x0103	BlockError
    typedef block_error_t motor_block_error_t;

    // 0x0104	ControllerErrors
    /*
    struct __attribute__((__packed__)) motor_controller_errors_t
    {
        union
        {
            uint16_t errors_m_arr[2];
            struct
            {
                uint16_t errors_m1;
                uint16_t errors_m2;
            };
        };
    };
    */
    // 0x0105	RPM
    // 0x0106	Speed
    // 0x0107	Voltage
    // 0x0108	Current
    // 0x0109	Power
    // 0x010A	Gear+Roll
    // 0x010B	Temperature
    // 0x010D	Odometer

    struct __attribute__((__packed__)) motor_ecu_params_t
    {
        // 0x0100	BlockInfo
        motor_block_info_t block_info;

        // 0x0101	BlockHealth
        motor_block_health_t block_health;

        // 0x0102	BlockCfg
        motor_block_cfg_t block_cfg;

        // 0x0103	BlockError
        motor_block_error_t block_error;

        MotorData motor_1;
        MotorData motor_2;

        // 0x010C	Odometer
        uint32_t odometer;
        uint32_t odometer_last_update;
    };

    bool init_can_manager_for_motors(CANManager &cm, motor_ecu_params_t &motor_ecu_params);

#ifdef __cplusplus
}
#endif

#endif // MOTORECU_LOW_LEVEL_ABSTRACTION_H
