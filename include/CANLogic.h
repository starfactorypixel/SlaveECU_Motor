#pragma once

#include <CANLibrary.h>

void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length);

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;

namespace CANLib
{
	//*********************************************************************
	// CAN Library settings
	//*********************************************************************

	/// @brief Number of CANObjects in CANManager
	static constexpr uint8_t CFG_CANObjectsCount = 13;

	/// @brief The size of CANManager's internal CAN frame buffer
	static constexpr uint8_t CFG_CANFrameBufferSize = 16;

	//*********************************************************************
	// CAN Manager
	//*********************************************************************
	CANManager<CFG_CANObjectsCount, CFG_CANFrameBufferSize> can_manager(&HAL_CAN_Send);

	//*********************************************************************
	// CAN Blocks: common blocks
	//*********************************************************************
	// 0x0100 BlockInfo
	// request | timer:15000
	// 1 + X { type[0] data[1..7] }
	// Основная информация о блоке. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_info(0x0100);

	// 0x0101 BlockHealth
	// request | event Link
	// 1 + X { type[0] data[1..7] }
	// Информация о здоровье блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_health(0x0101);

	// 0x0102 BlockCfg
	// request Link
	// 1 + X { type[0] data[1..7] }
	// Чтение и запись настроек блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_cfg(0x0102);

	// 0x0103 BlockError
	// request | event Link
	// 1 + X { type[0] data[1..7] }
	// Ошибки блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_error(0x0103);

	//*********************************************************************
	// CAN Blocks: specific blocks
	//*********************************************************************
	// 0x0104 ControllerErrors
	// request | timer:250
	// uint16_t bitmask 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Ошибки контроллеров: контроллер №1 — uint16, контроллер №2 — uint16
	CANObject<uint16_t, 2> obj_controller_errors(0x0104);

	// 0x0105 RPM
	// request | timer:250
	// uint16_t Об\м 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Обороты двигателей: контроллер №1 — uint16, контроллер №2 — uint16
	CANObject<uint16_t, 2> obj_controller_rpm(0x0105);

	// 0x0106 Speed
	// request | timer:250
	// uint16_t 100м\ч 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Расчетная скорость в сотнях метров в час: контроллер №1 — uint16, контроллер №2 — uint16
	CANObject<uint16_t, 2> obj_controller_speed(0x0106);

	// 0x0107 Voltage
	// request | timer:500
	// uint16_t 100мВ 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Напряжение на контроллерах в сотнях мВ: контроллер №1 — uint16, контроллер №2 — uint16
	CANObject<uint16_t, 2> obj_controller_voltage(0x0107);

	// 0x0108 Current
	// request | timer:500
	// int16_t 100мА 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Ток контроллеров в сотнях мА: контроллер №1 — int16, контроллер №2 — int16
	CANObject<uint16_t, 2> obj_controller_current(0x0108);

	// 0x0109 Power
	// request | timer:500
	// int16_t Вт 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
	// Потребляемая (отдаваемая) мощность в Вт: контроллер №1 — uint16, контроллер №2 — uint16
	CANObject<uint16_t, 2> obj_controller_power(0x0109);

	// 0x010A Gear+Roll
	// request | timer:500
	// uint8_t bitmask 1 + 1+1 + 1+1 { type[0] mg1[1] mr1[2] mg2[3] mr2[3] }
	// Передача и фактическое направление вращения
	CANObject<uint8_t, 4> obj_controller_gear_n_roll(0x010A);

	// 0x010B Temperature
	// request | timer:1000
	// int8_t °C 1 + 1+1 + 1+1 { type[0] mt1[1] ct1[2] mt2[3] ct2[4] }
	// Температура двигателей и контроллеров: №1 — int8 + int8, №2 — int8 + int8
	CANObject<uint8_t, 4> obj_controller_temperature(0x010B);

	// 0x010C Odometer
	// request | timer:5000
	// uint32_t 100м 1 + 4 { type[0] m[1..4] }
	// Одометр (общий для авто), в сотнях метров
	CANObject<uint32_t, 1> obj_controller_odometer(0x010C);

	// вызывается, если по CAN пришла команда включения/выключения габаритов
	/*
	void side_beam_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
	 if (can_frame.data[0] == 0)
	 {
	 }
	 else
	 {
	 }

	 can_frame.initialized = true;
	 can_frame.function_id = CAN_FUNC_SET_OUT_OK;
	 // can_frame.data[0] doesn't change
	 // TODO: может читать установленное значение с порта и его присваивать в can_frame.data[0]?
	 // читать вот этой функцией: HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)
	 can_frame.raw_data_length = 2;
	}
	*/

	inline void Setup()
	{
		//*************************************************************
		// TODO: CANManager experiments
		// init normal timer for Block_Info
		obj_block_info.SetValue(0, 0x66, CAN_TIMER_TYPE_NORMAL);
		//*************************************************************

		// 0x0100 BlockInfo
		// request | timer:15000
		// 1 + X { type[0] data[1..7] }
		// Основная информация о блоке. См. "Системные параметры".
		obj_block_info.SetTimerPeriod(15000);
		can_manager.RegisterObject(obj_block_info);

		// 0x0101 BlockHealth
		// request | event Link
		// 1 + X { type[0] data[1..7] }
		// Информация о здоровье блока. См. "Системные параметры".
		obj_block_health.SetErrorEventDelay(300);
		can_manager.RegisterObject(obj_block_health);

		// 0x0102 BlockCfg
		// request Link
		// 1 + X { type[0] data[1..7] }
		// Чтение и запись настроек блока. См. "Системные параметры".
		can_manager.RegisterObject(obj_block_cfg);

		// 0x0103 BlockError
		// request | event Link
		// 1 + X { type[0] data[1..7] }
		// Ошибки блока. См. "Системные параметры".
		obj_block_error.SetErrorEventDelay(300);
		can_manager.RegisterObject(obj_block_error);

		// 0x0104 ControllerErrors
		// request | timer:250
		// uint16_t bitmask 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
		// Ошибки контроллеров: контроллер №1 — uint16, контроллер №2 — uint16
		obj_controller_errors.SetTimerPeriod(250);
		can_manager.RegisterObject(obj_controller_errors);

		// 0x0105 RPM
		// request | timer:250
		// uint16_t Об\м 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
		// Обороты двигателей: контроллер №1 — uint16, контроллер №2 — uint16
		obj_controller_rpm.SetTimerPeriod(250);
		can_manager.RegisterObject(obj_controller_rpm);

		// 0x0106 Speed
		// request | timer:250
		// uint16_t 100м\ч 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
		// Расчетная скорость в сотнях метров в час: контроллер №1 — uint16, контроллер №2 — uint16
		obj_controller_speed.SetTimerPeriod(250);
		can_manager.RegisterObject(obj_controller_speed);

		// 0x0107 Voltage
		// request | timer:500
		// uint16_t 100мВ 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
		// Напряжение на контроллерах в сотнях мВ: контроллер №1 — uint16, контроллер №2 — uint16
		obj_controller_voltage.SetTimerPeriod(500);
		can_manager.RegisterObject(obj_controller_voltage);

		// 0x0108 Current
		// request | timer:500
		// int16_t 100мА 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
		// Ток контроллеров в сотнях мА: контроллер №1 — int16, контроллер №2 — int16
		obj_controller_current.SetTimerPeriod(500);
		can_manager.RegisterObject(obj_controller_current);

		// 0x0109 Power
		// request | timer:500
		// int16_t Вт 1 + 2 + 2 { type[0] m1[1..2] m2[3..4] }
		// Потребляемая (отдаваемая) мощность в Вт: контроллер №1 — uint16, контроллер №2 — uint16
		obj_controller_power.SetTimerPeriod(500);
		can_manager.RegisterObject(obj_controller_power);

		// 0x010A Gear+Roll
		// request | timer:500
		// uint8_t bitmask 1 + 1+1 + 1+1 { type[0] mg1[1] mr1[2] mg2[3] mr2[3] }
		// Передача и фактическое направление вращения
		obj_controller_gear_n_roll.SetTimerPeriod(500);
		can_manager.RegisterObject(obj_controller_gear_n_roll);

		// 0x010B Temperature
		// request | timer:1000
		// int8_t °C 1 + 1+1 + 1+1 { type[0] mt1[1] ct1[2] mt2[3] ct2[4] }
		// Температура двигателей и контроллеров: №1 — int8 + int8, №2 — int8 + int8
		obj_controller_temperature.SetTimerPeriod(1000);
		can_manager.RegisterObject(obj_controller_temperature);

		// 0x010C Odometer
		// request | timer:5000
		// uint32_t 100м 1 + 4 { type[0] m[1..4] }
		// Одометр (общий для авто), в сотнях метров
		obj_controller_odometer.SetTimerPeriod(5000);
		can_manager.RegisterObject(obj_controller_odometer);
	}

	inline void Loop(uint32_t &current_time)
	{
		can_manager.Process(current_time);

		//*************************************************************
		// TODO: CANManager experiments
		/*
		static uint32_t iter = 0;
		if (current_time - iter > 1000)
		{
			iter = current_time;

			uint8_t *data = (uint8_t *)&current_time;
			obj_block_info.SetValue(3, data[0], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(4, data[1], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(5, data[2], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(6, data[3], CAN_TIMER_TYPE_NORMAL);
		}
		*/
		//*************************************************************

		static uint32_t time = 0;
		if(current_time - time > 500)
		{
			time = current_time;

			uint16_t rand1 = current_time ^ current_time / 2;
			uint16_t rand2 = current_time ^ current_time / 4;

			obj_controller_voltage.SetValue(0, rand1, CAN_TIMER_TYPE_NORMAL);
			obj_controller_voltage.SetValue(1, rand1+50, CAN_TIMER_TYPE_NORMAL);
			obj_controller_current.SetValue(0, rand2, CAN_TIMER_TYPE_NORMAL);
			obj_controller_current.SetValue(1, rand2-50, CAN_TIMER_TYPE_NORMAL);

			Serial::Print("tick\r\n");
		}

		current_time = HAL_GetTick();

		return;
	}
}
