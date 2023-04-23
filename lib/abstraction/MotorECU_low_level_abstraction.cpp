#include "MotorECU_low_level_abstraction.h"

bool init_can_manager_for_motors(CANManager &cm, motor_ecu_params_t &motor_ecu_params)
{
    CANObject *co = nullptr;

    // 0x0100	BlockInfo
    // request | timer:15000	1 + X	{ type[0] data[1..7] }
    // Основная информация о блоке. См. "Системные параметры".
    init_block_info(cm, MOTOR_CANO_ID_BLOCK_INFO, motor_ecu_params.block_info);

    // 0x0101	BlockHealth
    // request | event	1 + X	{ type[0] data[1..7] }
    // Информация о здоровье блока. См. "Системные параметры".
    init_block_health(cm, MOTOR_CANO_ID_BLOCK_HEALTH, motor_ecu_params.block_health);

    // 0x0102	BlockCfg
    // request	1 + X	{ type[0] data[1..7] }
    //	Чтение и запись настроек блока. См. "Системные параметры".
    init_block_cfg(cm, MOTOR_CANO_ID_BLOCK_CFG, motor_ecu_params.block_cfg);

    // 0x0103	BlockError
    // request | event	1 + X	{ type[0] data[1..7] }
    // Ошибки блока. См. "Системные параметры".
    init_block_error(cm, MOTOR_CANO_ID_BLOCK_ERROR, motor_ecu_params.block_error);

    // 0x0104	ControllerErrors
    // request | timer:250	uint16_t	bitmask	1 + 2 + 2	{ type[0] m1[1..2] m2[3..4] }
    // Ошибки контроллеров: контроллер №1 — uint16, контроллер №2 — uint16
    co = cm.add_can_object(MOTOR_CANO_ID_CONTROLLER_ERRORS, "ControllerErrors");
    co->add_data_field(DF_UINT16, &motor_ecu_params.motor_1.Errors);
    co->add_data_field(DF_UINT16, &motor_ecu_params.motor_2.Errors);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 250);

    // 0x0105	RPM
    // request | timer:250	uint16_t	Об\м	1 + 2 + 2	{ type[0] m1[1..2] m2[3..4] }
    // Обороты двигателей: контроллер №1 — uint16, контроллер №2 — uint16
    co = cm.add_can_object(MOTOR_CANO_ID_RPM, "RPM");
    co->add_data_field(DF_UINT16, &motor_ecu_params.motor_1.RPM);
    co->add_data_field(DF_UINT16, &motor_ecu_params.motor_2.RPM);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 250);

    // 0x0106	Speed
    // request | timer:250	uint16_t	100м\ч	1 + 2 + 2	{ type[0] m1[1..2] m2[3..4] }
    // Расчетная скорость: контроллер №1 — uint16, контроллер №2 — uint16
    co = cm.add_can_object(MOTOR_CANO_ID_SPEED, "Speed");
    co->add_data_field(DF_UINT16, &motor_ecu_params.motor_1.Speed);
    co->add_data_field(DF_UINT16, &motor_ecu_params.motor_2.Speed);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 250);

    // 0x0107	Voltage
    // request | timer:500	uint16_t	100мВ	1 + 2 + 2	{ type[0] m1[1..2] m2[3..4] }
    // Напряжение на контроллерах в сотнях мВ: контроллер №1 — uint16, контроллер №2 — uint16
    co = cm.add_can_object(MOTOR_CANO_ID_VOLTAGE, "Voltage");
    co->add_data_field(DF_UINT16, &motor_ecu_params.motor_1.Voltage);
    co->add_data_field(DF_UINT16, &motor_ecu_params.motor_2.Voltage);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 500);

    // 0x0108	Current
    // request | timer:500	int16_t	100мА	1 + 2 + 2	{ type[0] m1[1..2] m2[3..4] }
    // Ток контроллеров в сотнях мА: контроллер №1 — int16, контроллер №2 — int16
    co = cm.add_can_object(MOTOR_CANO_ID_CURRENT, "Current");
    co->add_data_field(DF_INT16, &motor_ecu_params.motor_1.Current);
    co->add_data_field(DF_INT16, &motor_ecu_params.motor_2.Current);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 500);

    // 0x0109	Power
    // request | timer:500	int16_t	Вт	1 + 2 + 2	{ type[0] m1[1..2] m2[3..4] }
    // Потребляемая (отдаваемая) мощность в Вт: контроллер №1 — uint16, контроллер №2 — uint16
    co = cm.add_can_object(MOTOR_CANO_ID_POWER, "Power");
    co->add_data_field(DF_INT16, &motor_ecu_params.motor_1.Power);
    co->add_data_field(DF_INT16, &motor_ecu_params.motor_2.Power);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 500);

    // 0x010A	Gear+Roll
    // request | timer:500	uint8_t	bitmask	1 + 1+1 + 1+1	{ type[0] mg1[1] mr1[2] mg2[3] mr2[3] }
    // Передача и фактическое направление вращения
    co = cm.add_can_object(MOTOR_CANO_ID_GEAR_AND_ROLL, "Gear & Roll");
    co->add_data_field(DF_UINT8, &motor_ecu_params.motor_1.Gear);
    co->add_data_field(DF_UINT8, &motor_ecu_params.motor_1.Roll);
    co->add_data_field(DF_UINT8, &motor_ecu_params.motor_2.Gear);
    co->add_data_field(DF_UINT8, &motor_ecu_params.motor_2.Roll);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 500);

    // 0x010B	Temperature
    // request | timer:1000	int8_t	°C	1 + 1+1 + 1+1	{ type[0] mt1[1] ct1[2] mt2[3] ct2[4] }
    // Температура двигателей и контроллеров: №1 — int8 + int8, №2 — int8 + int8
    co = cm.add_can_object(MOTOR_CANO_ID_TEMPERATURE, "Temperature");
    co->add_data_field(DF_INT8, &motor_ecu_params.motor_1.TMotor);
    co->add_data_field(DF_INT8, &motor_ecu_params.motor_1.TController);
    co->add_data_field(DF_INT8, &motor_ecu_params.motor_2.TMotor);
    co->add_data_field(DF_INT8, &motor_ecu_params.motor_2.TController);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 1000);

    // 0x010D	Odometer
    // request | timer:5000	uint32_t	100м	1 + 4	{ type[0] m[1..4] }
    // Одометр (общий для авто), в сотнях метров
    co = cm.add_can_object(MOTOR_CANO_ID_ODOMETER, "Odometer");
    co->add_data_field(DF_UINT32, &motor_ecu_params.odometer);
    co->add_function(CAN_FUNC_REQUEST_IN);
    add_three_timers(*co, 5000);

    return true;
}
