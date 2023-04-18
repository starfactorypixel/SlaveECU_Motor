#include <Arduino.h>
#include "FardriverController.h"

FardriverController<1> motor1;
FardriverController<2> motor2;

void OnMotorEvent(const uint8_t motor_idx, const uint8_t data[16])
{
	/*
	// Пример.
	switch (data[1])
	{
		case 0x00:
		{
			motor_packet_0_t *data = (motor_packet_0_t*) &data;
			// работа с data пакета 0.
			
			break;
		}
		case 0x01:
		{
			motor_packet_1_t *data = (motor_packet_1_t*) &data;
			// работа с data пакета 1.
			
			break;
		}
		default:
		{
			break;
		}
	}
	*/
	
	return;
}

void OnMotorError(const uint8_t motor_idx, const motor_error_t code)
{
	return;
}

void OnMotorTX(const uint8_t motor_idx, const uint8_t *data, const uint8_t data_len)
{
	Serial.write(data, data_len);
	
	return;
}

void setup()
{

	motor1.SetEventCallback(OnMotorEvent);
	motor2.SetEventCallback(OnMotorEvent);
	motor1.SetErrorCallback(OnMotorError);
	motor2.SetErrorCallback(OnMotorError);
	motor1.SetTXCallback(OnMotorTX);
	motor2.SetTXCallback(OnMotorTX);
	
	return;
}

void loop()
{
	static uint32_t current_time = millis();
	
	motor1.Processing(current_time);
	motor2.Processing(current_time);

	if(Serial.available() > 0)
	{
		motor1.RXByte( Serial.read() , current_time);
		motor2.RXByte( Serial.read() , current_time);
	}
	
	motor1.IsActive();
	motor2.IsActive();
	
	return;
}
