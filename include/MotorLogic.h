#pragma once
#include <stm32f1xx_hal.h>
#include <MotorManager.h>
#include <drivers/MotorFardriverOld.h>
#include <drivers/MotorFardriverNew.h>
#include <CanObj/IBlockInfoSender.hpp>

extern UART_HandleTypeDef hMotor1Uart;
extern UART_HandleTypeDef hMotor2Uart;

extern IBlockInfoSender &BlockInfoSender;

/*
void OnMotorEvent(const uint8_t motor_idx, motor_packet_raw_t *raw_packet);
void OnMotorError(const uint8_t motor_idx, const motor_error_t code);
void OnMotorHWError(const uint8_t motor_idx, const uint8_t code);
void OnMotorTX(const uint8_t motor_idx, const uint8_t *raw, const uint8_t raw_len);
*/
namespace Motors
{
	void MOTOR_UART_TX(uint8_t idx, const uint8_t *raw, const uint16_t length);
	void MOTOR_ERROR(uint8_t idx, MotorDeviceInterface::error_code_t code);
	
#warning Check packet req connection from controller. Check data
	
	MotorFardriverNew motor1;
	MotorFardriverNew motor2;
	MotorManager manager(MOTOR_UART_TX, MOTOR_ERROR);
	
	struct data_t
	{
		UART_HandleTypeDef *hal;		// Указатель на объект HAL USART
		uint8_t hot[200];				// Горячий массив данных (Работа в прерывании)
	} uart_data[2];
	
	enum motor_num_t : uint8_t { MOTOR_1 = 0, MOTOR_2 = 1 };
	
	
	inline void UART_RX(uint8_t idx, const uint16_t length)
	{
		//DEBUG_LOG_TOPIC("MOTOR-RX", "idx: %d, length : %d\n", idx, length);

		manager.RawRx(idx, uart_data[idx].hot, length);
		
		return;
	}
	
	void MOTOR_UART_TX(uint8_t idx, const uint8_t *raw, const uint16_t length)
	{
		HAL_UART_Transmit(uart_data[idx].hal, (uint8_t *)raw, length, 64U);
		
		return;
	}
	
	void MOTOR_ERROR(uint8_t idx, MotorDeviceInterface::error_code_t code)
	{
		DEBUG_LOG_TOPIC("MOTOR-ERR", "idx: %d, code: %d\n", idx, code);

		BlockInfoSender.SendErrorMsg(idx+1, code, manager.common_data[idx].errors);

		/*
		switch(idx)
		{
			// Если ошибка связи, то это не отправиться в CAN.
			
			case MOTOR_1:
			{
				if(code == MotorDeviceInterface::ERROR_CTRL)
					CANLib::obj_controller_errors_1.SetValue(0, manager.common_data[MOTOR_1].errors, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				
				break;
			}
			case MOTOR_2:
			{
				if(code == MotorDeviceInterface::ERROR_CTRL)
					CANLib::obj_controller_errors_2.SetValue(0, manager.common_data[MOTOR_2].errors, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
		}
		*/
	}
	
	
	inline void Setup()
    {
		memset(uart_data, 0x00, sizeof(uart_data));
		
		uart_data[MOTOR_1].hal = &hMotor1Uart;
		uart_data[MOTOR_2].hal = &hMotor2Uart;

/*
		motor1.SetReadyCallback([](const FdNew::packet_raw_t *data)
		{
			if(data->_A1 != 0xB6) return;

			// Заполяем данные, но при этом частота обновлений будет примерно 1.5с
		});
*/	
		motor1.SetWheelDiameter(Config::obj.body.wheel_diameter);
		motor2.SetWheelDiameter(Config::obj.body.wheel_diameter);
		manager.SetModel(MOTOR_1, motor1);
		manager.SetModel(MOTOR_2, motor2);
		
		HAL_UARTEx_ReceiveToIdle_IT(uart_data[MOTOR_1].hal, uart_data[MOTOR_1].hot, sizeof(uart_data[MOTOR_1].hot));
		HAL_UARTEx_ReceiveToIdle_IT(uart_data[MOTOR_2].hal, uart_data[MOTOR_2].hot, sizeof(uart_data[MOTOR_2].hot));
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		manager.Tick(current_time);

		static uint32_t tick_25 = 0;
		if(current_time - tick_25 > 50)
		{
			tick_25 = current_time;

/*
			uint32_t total_odometer = 0;
			MotorManagerData::common_data_t *common_data;
			
			if(manager.common_data_ready[0] == true)
			{
				common_data = &manager.common_data[0];
				
				CANLib::obj_rpm_1.SetValue(0, common_data->rpm, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_speed_1.SetValue(0, common_data->speed, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_voltage_1.SetValue(0, common_data->voltage, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_current_1.SetValue(0, common_data->current, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_power_1.SetValue(0, common_data->power, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_gear_1_roll_1.SetValue(0, common_data->gear, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_gear_1_roll_1.SetValue(1, common_data->roll, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_temperature_motor_1.SetValue(0, common_data->temp_motor, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_temperature_controller_1.SetValue(0, common_data->temp_controller, CAN_TIMER_TYPE_NORMAL);
				total_odometer = common_data->odometer;
			}

			if(manager.common_data_ready[1] == true)
			{
				common_data = &manager.common_data[1];
				
				CANLib::obj_rpm_2.SetValue(0, common_data->rpm, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_speed_2.SetValue(0, common_data->speed, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_voltage_2.SetValue(0, common_data->voltage, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_current_2.SetValue(0, common_data->current, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_power_2.SetValue(0, common_data->power, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_gear_2_roll_2.SetValue(0, common_data->gear, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_gear_2_roll_2.SetValue(1, common_data->roll, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_temperature_motor_2.SetValue(0, common_data->temp_motor, CAN_TIMER_TYPE_NORMAL);
				CANLib::obj_temperature_controller_2.SetValue(0, common_data->temp_controller, CAN_TIMER_TYPE_NORMAL);
				total_odometer = (common_data->odometer > total_odometer) ? common_data->odometer : total_odometer;
			}
			
		CANLib::obj_odometer.SetValue(0, total_odometer, CAN_TIMER_TYPE_NORMAL);


*/	
		}

		
		current_time = HAL_GetTick();
		
		return;
	}
}
