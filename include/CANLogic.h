#pragma once
#include <DrakePinD.hpp>
#include <CANLibrary.h>

extern CAN_HandleTypeDef hcan;
extern void HAL_CAN_Send(uint16_t id, uint8_t *data_raw, uint8_t length_raw);

namespace CANLib
{
	static_assert(ENV_CAN_FIRST_ID == 0x0100 || ENV_CAN_FIRST_ID == 0x0130, "'ENV_CAN_FIRST_ID' must be 0x0100 or 0x0130 only!");
	
	static constexpr uint8_t CFG_CANObjectsCount = 32;
	static constexpr uint8_t CFG_CANFrameBufferSize = 16;
	static constexpr uint16_t CFG_CANFirstId = ENV_CAN_FIRST_ID;

	DrakePinD can_rs({GPIOA, GPIO_PIN_15}, DrakePin::OutputOpenDrain, DrakePin::High);
	
	CANManager<CFG_CANObjectsCount, CFG_CANFrameBufferSize> can_manager(&HAL_CAN_Send);
	
	CANObject<uint8_t,  7> obj_block_info(CFG_CANFirstId + 0);
	CANObject<uint8_t,  7> obj_block_health(CFG_CANFirstId + 1);
	CANObject<uint8_t,  7> obj_block_features(CFG_CANFirstId + 2);
	CANObject<uint8_t,  7> obj_block_error(CFG_CANFirstId + 3);

	CANObject<uint16_t, 1> obj_throttle_value_1(CFG_CANFirstId + 4);
	CANObject<uint16_t, 1> obj_throttle_value_2(CFG_CANFirstId + 5);
	CANObject<uint8_t,  1> obj_transmission_value_1(CFG_CANFirstId + 6);
	CANObject<uint8_t,  1> obj_transmission_value_2(CFG_CANFirstId + 7);
	CANObject<uint8_t,  1> obj_brakerecuperation_flag_1(CFG_CANFirstId + 8);
	CANObject<uint8_t,  1> obj_brakerecuperation_flag_2(CFG_CANFirstId + 9);
	CANObject<uint8_t,  1> obj_ignitionlock_flag_1(CFG_CANFirstId + 10);
	CANObject<uint8_t,  1> obj_ignitionlock_flag_2(CFG_CANFirstId + 11);
	CANObject<uint16_t, 1> obj_controller_errors_1(CFG_CANFirstId + 12);
	CANObject<uint16_t, 1> obj_controller_errors_2(CFG_CANFirstId + 13);
	CANObject<uint16_t, 1> obj_rpm_1(CFG_CANFirstId + 14, 250);
	CANObject<uint16_t, 1> obj_rpm_2(CFG_CANFirstId + 15, 250);
	CANObject<uint16_t, 1> obj_speed_1(CFG_CANFirstId + 16, 250);
	CANObject<uint16_t, 1> obj_speed_2(CFG_CANFirstId + 17, 250);
	CANObject<uint16_t, 1> obj_voltage_1(CFG_CANFirstId + 18, 500);
	CANObject<uint16_t, 1> obj_voltage_2(CFG_CANFirstId + 19, 500);
	CANObject<int16_t,  1> obj_current_1(CFG_CANFirstId + 20, 500);
	CANObject<int16_t,  1> obj_current_2(CFG_CANFirstId + 21, 500);
	CANObject<int16_t,  1> obj_power_1(CFG_CANFirstId + 22, 500);
	CANObject<int16_t,  1> obj_power_2(CFG_CANFirstId + 23, 500);
	CANObject<uint8_t,  2> obj_gear_1_roll_1(CFG_CANFirstId + 24, 500);
	CANObject<uint8_t,  2> obj_gear_2_roll_2(CFG_CANFirstId + 25, 500);
	CANObject<int16_t,  1> obj_temperature_motor_1(CFG_CANFirstId + 26, 1000);
	CANObject<int16_t,  1> obj_temperature_motor_2(CFG_CANFirstId + 27, 1000);
	CANObject<int16_t,  1> obj_temperature_controller_1(CFG_CANFirstId + 28, 1000);
	CANObject<int16_t,  1> obj_temperature_controller_2(CFG_CANFirstId + 29, 1000);
	CANObject<uint32_t, 1> obj_odometer(CFG_CANFirstId + 30, 5000);
	CANObject<int8_t,   2> obj_temperature_ext(CFG_CANFirstId + 31);

/*
	//*********************************************************************
	// CAN Blocks: specific blocks
	//*********************************************************************
	// 0x0104 ControllerErrors
	// request | timer:250
	// uint16_t bitmask 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Ошибки контроллеров: контроллер №1 — uint16, контроллер №2 — uint16
	CANObject<uint16_t, 2> obj_controller_errors(0x0104, 250, CAN_ERROR_DISABLED);

	// 0x0105 RPM
	// request | timer:250
	// uint16_t Об\м 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Обороты двигателей: контроллер №1 — uint16, контроллер №2 — uint16
	CANObject<uint16_t, 2> obj_controller_rpm(0x0105, 250, CAN_ERROR_DISABLED);

	// 0x0106 Speed
	// request | timer:250
	// uint16_t 100м\ч 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Расчетная скорость в сотнях метров в час: контроллер №1 — uint16, контроллер №2 — uint16
	CANObject<uint16_t, 2> obj_controller_speed(0x0106, 250, CAN_ERROR_DISABLED);

	// 0x0107 Voltage
	// request | timer:500
	// uint16_t 100мВ 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Напряжение на контроллерах в сотнях мВ: контроллер №1 — uint16, контроллер №2 — uint16
	CANObject<uint16_t, 2> obj_controller_voltage(0x0107, 500, CAN_ERROR_DISABLED);

	// 0x0108 Current
	// request | timer:500
	// int16_t 100мА 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Ток контроллеров в сотнях мА: контроллер №1 — int16, контроллер №2 — int16
	CANObject<int16_t, 2> obj_controller_current(0x0108, 500, CAN_ERROR_DISABLED);

	// 0x0109 Power
	// request | timer:500
	// int16_t Вт 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Потребляемая (отдаваемая) мощность в Вт: контроллер №1 — uint16, контроллер №2 — uint16
	CANObject<int16_t, 2> obj_controller_power(0x0109, 500, CAN_ERROR_DISABLED);

	// 0x010A Gear+Roll
	// request | timer:500
	// uint8_t bitmask 1 + 1+1 + 1+1 { type[0] mg1[1] mr1[2] mg2[3] mr2[3] }
	// Передача и фактическое направление вращения
	CANObject<uint8_t, 4> obj_controller_gear_n_roll(0x010A, 500, CAN_ERROR_DISABLED);

	// 0x010B TemperatureMotor
	// request | timer:1000
	// int16_t	°C	1 + 2 + 2	{ type[0] mt1[1..2] mt2[3..4] }
	// Температура двигателей: №1 — int16, №2 — int16
	CANObject<int16_t, 2> obj_motor_temperature(0x010B, 1000, CAN_ERROR_DISABLED);

	// 0x010B TemperatureController
	// request | timer:1000
	// int16_t	°C	1 + 2 + 2	{ type[0] ct1[1..2] ct2[3..4] }
	// Температура контроллеров: №1 — int16, №2 — int16
	CANObject<int16_t, 2> obj_controller_temperature(0x010C, 1000, CAN_ERROR_DISABLED);

	// 0x010C Odometer
	// request | timer:5000
	// uint32_t 100м 1 + 4 { type[0] m[1..4] }
	// Одометр (общий для авто), в сотнях метров
	CANObject<uint32_t, 1> obj_controller_odometer(0x010D, 5000, CAN_ERROR_DISABLED);
*/	

	void CAN_Enable()
	{
		HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
		HAL_CAN_Start(&hcan);
		
		can_rs.Off();
		
		return;
	}
	
	void CAN_Disable()
	{
		HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
		HAL_CAN_Stop(&hcan);
		
		can_rs.On();
		
		return;
	}
	
	inline void Setup()
	{
		can_rs.Init();
		
		set_block_info_params(obj_block_info);
		set_block_health_params(obj_block_health);
		set_block_features_params(obj_block_features);
		set_block_error_params(obj_block_error);
		
		can_manager.RegisterObject(obj_block_info);
		can_manager.RegisterObject(obj_block_health);
		can_manager.RegisterObject(obj_block_features);
		can_manager.RegisterObject(obj_block_error);

		can_manager.RegisterObject(obj_throttle_value_1);
		can_manager.RegisterObject(obj_throttle_value_2);
		can_manager.RegisterObject(obj_transmission_value_1);
		can_manager.RegisterObject(obj_transmission_value_2);
		can_manager.RegisterObject(obj_brakerecuperation_flag_1);
		can_manager.RegisterObject(obj_brakerecuperation_flag_2);
		can_manager.RegisterObject(obj_ignitionlock_flag_1);
		can_manager.RegisterObject(obj_ignitionlock_flag_2);
		can_manager.RegisterObject(obj_controller_errors_1);
		can_manager.RegisterObject(obj_controller_errors_2);
		can_manager.RegisterObject(obj_rpm_1);
		can_manager.RegisterObject(obj_rpm_2);
		can_manager.RegisterObject(obj_speed_1);
		can_manager.RegisterObject(obj_speed_2);
		can_manager.RegisterObject(obj_voltage_1);
		can_manager.RegisterObject(obj_voltage_2);
		can_manager.RegisterObject(obj_current_1);
		can_manager.RegisterObject(obj_current_2);
		can_manager.RegisterObject(obj_power_1);
		can_manager.RegisterObject(obj_power_2);
		can_manager.RegisterObject(obj_gear_1_roll_1);
		can_manager.RegisterObject(obj_gear_2_roll_2);
		can_manager.RegisterObject(obj_temperature_motor_1);
		can_manager.RegisterObject(obj_temperature_motor_2);
		can_manager.RegisterObject(obj_temperature_controller_1);
		can_manager.RegisterObject(obj_temperature_controller_2);
		can_manager.RegisterObject(obj_odometer);
		can_manager.RegisterObject(obj_temperature_ext);

		// Передача версий и типов в объект block_info
		obj_block_info.SetValue(0, (About::board_type << 3 | About::board_ver), CAN_TIMER_TYPE_NORMAL);
		obj_block_info.SetValue(1, (About::soft_ver << 2 | About::can_ver), CAN_TIMER_TYPE_NORMAL);

		CAN_Enable();
		
		return;
	}

	inline void Loop(uint32_t &current_time)
	{
		can_manager.Process(current_time);

		// Передача UpTime блока в объект block_info
		static uint32_t iter1000 = 0;
		if(current_time - iter1000 > 1000)
		{
			iter1000 = current_time;
			
			uint8_t *data = (uint8_t *)&current_time;
			obj_block_info.SetValue(2, data[0], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(3, data[1], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(4, data[2], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(5, data[3], CAN_TIMER_TYPE_NORMAL);
		}
		
		// При выходе обновляем время
		current_time = HAL_GetTick();
		
		return;
	}
}
