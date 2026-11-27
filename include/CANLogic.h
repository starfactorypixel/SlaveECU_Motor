#pragma once
#include <DrakePinD.hpp>
#include <CANLibrary.h>
#include <CanObj/CanBlockInfo.hpp>
#include <CanObj/CanBlockCfg.hpp>
#include <CanObj/CanGuardedChannel.hpp>
#include <CanObj/CanCtrlParam.hpp>
#include <CanObj/CanInfoParamPtr.hpp>
#include "CanObj/CanMotorGearRollParam.hpp"
#include <CanObj/CanStreamObj.hpp>
#include "CANFunc.h"

extern CAN_HandleTypeDef hcan;
extern bool HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length);

namespace CANLib
{
	static_assert(ENV_CAN_FIRST_ID == 0x0100 || ENV_CAN_FIRST_ID == 0x0130, "'ENV_CAN_FIRST_ID' must be 0x0100 or 0x0130 only!");
	
	static constexpr uint8_t CFG_CANObjectsCount = 32;
	static constexpr uint16_t CAN_BASE_ID = ENV_CAN_FIRST_ID;

	DrakePinD can_rs({GPIOA, GPIO_PIN_15}, DrakePin::OutputOpenDrain, DrakePin::High);
	
	CANManager<CFG_CANObjectsCount> can_manager(&HAL_CAN_Send, &HAL_GetTick, &OnInterruptCtrl);

	CanBlockInfo obj_block_info(CAN_BASE_ID+0, OnStaticInfoReq, OnDynamicInfoReq);
	CanBlockCfg obj_block_cfg(CAN_BASE_ID+1, OnCfgSaveReset, block_cfg_table, block_cfg_table_count);
	CanGuardedChannel<uint16_t> obj_throttle_value_1(CAN_BASE_ID+4, Motors::MOTOR_1, MotorCtrl::SetThrottle, MotorCtrl::SetFailsafe);
	CanGuardedChannel<uint16_t> obj_throttle_value_2(CAN_BASE_ID+5, Motors::MOTOR_2, MotorCtrl::SetThrottle, MotorCtrl::SetFailsafe);

	template <Motors::motor_num_t motor, void (*SetFn)(uint8_t, uint8_t), typename T> 
	void Setter(uint8_t fId, T value)
	{
		if(fId != 0x01) return;
		SetFn(motor, value);
	}

	template <Motors::motor_num_t motor, uint8_t (*GetFn)(uint8_t), typename T> 
	T Getter()
	{
		return GetFn(motor);
	}

	// Если отправить CAN_FUNC_TOGGLE_IN или CAN_FUNC_ACTION_IN то Setter игнорирует команду а класс выполнит Getter и вернёт актуальное значение
	// Т.е. будет эффект как CAN_FUNC_REQUEST_IN

	CanCtrlParam<uint8_t> obj_transmission_value_1(CAN_BASE_ID+6, &Setter<Motors::MOTOR_1, &MotorCtrl::SetGear>, &Getter<Motors::MOTOR_1, &MotorCtrl::GetGear>);
	CanCtrlParam<uint8_t> obj_transmission_value_2(CAN_BASE_ID+7, &Setter<Motors::MOTOR_2, &MotorCtrl::SetGear>, &Getter<Motors::MOTOR_2, &MotorCtrl::GetGear>);
	CanCtrlParam<uint8_t> obj_brakerecuperation_flag_1(CAN_BASE_ID+8, &Setter<Motors::MOTOR_1, &MotorCtrl::SetBreak>, &Getter<Motors::MOTOR_1, &MotorCtrl::GetBreak>);
	CanCtrlParam<uint8_t> obj_brakerecuperation_flag_2(CAN_BASE_ID+9, &Setter<Motors::MOTOR_2, &MotorCtrl::SetBreak>, &Getter<Motors::MOTOR_2, &MotorCtrl::GetBreak>);
	CanCtrlParam<uint8_t> obj_ignitionlock_flag_1(CAN_BASE_ID+10, &Setter<Motors::MOTOR_1, &MotorCtrl::SetLock>, &Getter<Motors::MOTOR_1, &MotorCtrl::GetLock>);
	CanCtrlParam<uint8_t> obj_ignitionlock_flag_2(CAN_BASE_ID+11, &Setter<Motors::MOTOR_2, &MotorCtrl::SetLock>, &Getter<Motors::MOTOR_2, &MotorCtrl::GetLock>);

	//obj_controller_errors_1
	//obj_controller_errors_2

	// Т.к. нету проверки manager.common_data_ready[0] == true то даже пусыте данные будут считаться валидными.
	// Вариант сделать common_data ввиде класса с методами get set и возвращаеть через get ошибку если данных нет или устарели

	auto &data1 = Motors::manager.motor_data[Motors::MOTOR_1].data;
	auto &data2 = Motors::manager.motor_data[Motors::MOTOR_2].data;

	CanInfoParamPtr<uint16_t> obj_throttlevolt_1(CAN_BASE_ID+12, 1550, &data1.throttle_voltage);
	CanInfoParamPtr<uint16_t> obj_throttlevolt_2(CAN_BASE_ID+13, 1550, &data2.throttle_voltage);
	CanInfoParamPtr<uint16_t> obj_rpm_1(CAN_BASE_ID+14, 250, &data1.rpm);
	CanInfoParamPtr<uint16_t> obj_rpm_2(CAN_BASE_ID+15, 250, &data2.rpm);
	CanInfoParamPtr<uint16_t> obj_speed_1(CAN_BASE_ID+16, 250, &data1.speed);
	CanInfoParamPtr<uint16_t> obj_speed_2(CAN_BASE_ID+17, 250, &data2.speed);
	CanInfoParamPtr<uint16_t> obj_voltage_1(CAN_BASE_ID+18, 500, &data1.voltage);
	CanInfoParamPtr<uint16_t> obj_voltage_2(CAN_BASE_ID+19, 500, &data2.voltage);
	CanInfoParamPtr<int16_t> obj_current_1(CAN_BASE_ID+20, 500, &data1.current);
	CanInfoParamPtr<int16_t> obj_current_2(CAN_BASE_ID+21, 500, &data2.current);
	CanInfoParamPtr<int16_t> obj_power_1(CAN_BASE_ID+22, 500, &data1.power);
	CanInfoParamPtr<int16_t> obj_power_2(CAN_BASE_ID+23, 500, &data2.power);
	CanMotorGearRollParam obj_gear_roll_1(CAN_BASE_ID+24, &data1.gear, &data1.roll, 500);
	CanMotorGearRollParam obj_gear_roll_2(CAN_BASE_ID+25, &data2.gear, &data2.roll, 500);
	CanInfoParamPtr<int16_t> obj_temperature_motor_1(CAN_BASE_ID+26, 1000, &data1.temp_motor);
	CanInfoParamPtr<int16_t> obj_temperature_motor_2(CAN_BASE_ID+27, 1000, &data2.temp_motor);
	CanInfoParamPtr<int16_t> obj_temperature_controller_1(CAN_BASE_ID+28, 1000, &data1.temp_controller);
	CanInfoParamPtr<int16_t> obj_temperature_controller_2(CAN_BASE_ID+29, 1000, &data2.temp_controller);
	CanInfoParamPtr<uint32_t> obj_odometer(CAN_BASE_ID+30, 5000, &data1.odometer);													// может оба одометра?
	CanStreamObj<int16_t, 16> obj_temperature(CAN_BASE_ID+31);






	template <typename T> 
	static inline uint8_t GetTempStatus(uint8_t idx, const T value)
	{
		if(value <= 70)
			return CAN_FUNC_TIMER_NORMAL;
		if(value <= 120)
			return CAN_FUNC_TIMER_WARNING;
		
		return CAN_FUNC_TIMER_CRITICAL;
	}





	
	void CAN_Enable()
	{
		HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_Start(&hcan);
		
		can_rs.Off();
		
		return;
	}
	
	void CAN_Disable()
	{
		HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_Stop(&hcan);
		
		can_rs.On();
		
		return;
	}
	
	inline void Setup()
	{
		can_rs.Init();


		obj_temperature_motor_1.SetValueClassifier(GetTempStatus);
		obj_temperature_motor_2.SetValueClassifier(GetTempStatus);
		obj_temperature_controller_1.SetValueClassifier(GetTempStatus);
		obj_temperature_controller_2.SetValueClassifier(GetTempStatus);

		#warning Add timer value on template?
		obj_transmission_value_1.SetTimerPeriod(100);
		obj_transmission_value_2.SetTimerPeriod(100);
		
		can_manager.AddObject(obj_block_info);
		can_manager.AddObject(obj_block_cfg);
		can_manager.AddObject(obj_throttle_value_1);
		can_manager.AddObject(obj_throttle_value_2);
		can_manager.AddObject(obj_transmission_value_1);
		can_manager.AddObject(obj_transmission_value_2);
		can_manager.AddObject(obj_brakerecuperation_flag_1);
		can_manager.AddObject(obj_brakerecuperation_flag_2);
		can_manager.AddObject(obj_ignitionlock_flag_1);
		can_manager.AddObject(obj_ignitionlock_flag_2);
		//can_manager.AddObject(obj_controller_errors_1);
		//can_manager.AddObject(obj_controller_errors_2);
		can_manager.AddObject(obj_throttlevolt_1);
		can_manager.AddObject(obj_throttlevolt_2);
		can_manager.AddObject(obj_rpm_1);
		can_manager.AddObject(obj_rpm_2);
		can_manager.AddObject(obj_speed_1);
		can_manager.AddObject(obj_speed_2);
		can_manager.AddObject(obj_voltage_1);
		can_manager.AddObject(obj_voltage_2);
		can_manager.AddObject(obj_current_1);
		can_manager.AddObject(obj_current_2);
		can_manager.AddObject(obj_power_1);
		can_manager.AddObject(obj_power_2);
		can_manager.AddObject(obj_gear_roll_1);
		can_manager.AddObject(obj_gear_roll_2);
		can_manager.AddObject(obj_temperature_motor_1);
		can_manager.AddObject(obj_temperature_motor_2);
		can_manager.AddObject(obj_temperature_controller_1);
		can_manager.AddObject(obj_temperature_controller_2);
		can_manager.AddObject(obj_odometer);
		can_manager.AddObject(obj_temperature);

		obj_throttle_value_1.SetBypass(true);
		obj_throttle_value_2.SetBypass(true);

		CAN_Enable();
		
		return;
	}

	inline void Loop(uint32_t &current_time)
	{
		can_manager.Processing(/*current_time*/);
		
		current_time = HAL_GetTick();
		return;
	}
}

IBlockInfoSender &BlockInfoSender = CANLib::obj_block_info;
ICanStream &CanTempStream = CANLib::obj_temperature;
