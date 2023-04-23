#include <Arduino.h>
#include <LiquidCrystal.h>
#include "FardriverController.h"
#include "MotorData.h"
#include "CANLibrary.h"
#include "MotorECU_low_level_abstraction.h"

// TODO: temporary length of the wheel
// After describing of BlockCfg we need to change it to the dynamic parameter
#define MOTOR_ECU_WHEEL_LENGTH 0.0179 // D=0.57m, WL=Pi*D и делим на 100 для скорости в 100м/ч

LiquidCrystal LCD(3, 11, 14, 16, 15, 17); // RS, E, D4, D5, D6, D7
void Draw();

FardriverController<1> motor1;
FardriverController<2> motor2;

motor_ecu_params_t motor_ecu_params = {0};
CANManager can_manager(&millis);

void OnMotorEvent(const uint8_t motor_idx, motor_packet_raw_t *raw_packet)
{
	MotorData *motor_data = nullptr;
	switch (motor_idx)
	{
	case 1:
		motor_data = &motor_ecu_params.motor_1;
		break;

	case 2:
		motor_data = &motor_ecu_params.motor_2;
		break;

	default:
		return;
	}

	switch (raw_packet->_A1)
	{
	case 0x00:
	{
		motor_packet_0_t *data = (motor_packet_0_t *)raw_packet;
		motor_data->Roll = (data->Roll & 0x03);
		motor_data->Gear = (data->Gear & 0x03);
		motor_data->RPM = data->RPM; // Об\м
		// TODO: speed calculation should be optimized
		motor_data->Speed = 60 * data->RPM * MOTOR_ECU_WHEEL_LENGTH; // 100м\ч
		motor_data->D2 = raw_packet->D2;
		motor_data->Errors = data->ErrorFlags;

		// odometer: 100m per bit
		uint32_t curr_time = millis();
		if (motor_ecu_params.odometer_last_update == 0)
		{
			motor_ecu_params.odometer_last_update = curr_time;
		}
		// crazy math: odometer will use both moter data =)
		// TODO: odometer calculation should be optimized
		motor_ecu_params.odometer += motor_data->Speed * (curr_time - motor_ecu_params.odometer_last_update) / 3600000;

		break;
	}
	case 0x01:
	{
		motor_packet_1_t *data = (motor_packet_1_t *)raw_packet;
		motor_data->Voltage = data->Voltage; // 100мВ
		// TODO: Ток 250мА/бит, int16
		motor_data->Current = (data->Current >> 2) * 10;		 // 100мА
		motor_data->Power = data->Voltage * data->Current * 100; // Вт

		break;
	}
	case 0x04:
	{
		// TODO: Градусы : uint8, но до 200 градусов. Если больше то int8
		motor_data->TController = (raw_packet->D2 <= 200) ? (uint8_t)raw_packet->D2 : (int8_t)raw_packet->D2;

		break;
	}
	case 0x0D:
	{
		// TODO: Градусы : uint8, но до 200 градусов. Если больше то int8
		motor_data->TMotor = (raw_packet->D0 <= 200) ? (uint8_t)raw_packet->D0 : (int8_t)raw_packet->D0;

		break;
	}
	default:
	{
		break;
	}
	}

	return;
}

void OnMotorError(const uint8_t motor_idx, const motor_error_t code)
{
	return;
}

void OnMotorTX(const uint8_t motor_idx, const uint8_t *raw, const uint8_t raw_len)
{
	Serial.write(raw, raw_len);

	return;
}

void setup()
{
	Serial.begin(19200);
	// while (!Serial){}

	LCD.begin(20, 4);
	LCD.home();
	pinMode(6, OUTPUT);
	digitalWrite(6, HIGH);

	motor1.SetEventCallback(OnMotorEvent);
	// motor2.SetEventCallback(OnMotorEvent);
	motor1.SetErrorCallback(OnMotorError);
	// motor2.SetErrorCallback(OnMotorError);
	motor1.SetTXCallback(OnMotorTX);
	// motor2.SetTXCallback(OnMotorTX);

	init_can_manager_for_motors(can_manager, motor_ecu_params);

	return;
}

void loop()
{
	uint32_t current_time = millis();

	motor1.Processing(current_time);
	// motor2.Processing(current_time);

	while (Serial.available() > 0)
	{
		motor1.RXByte(Serial.read(), current_time);
		// motor2.RXByte( Serial.read() , current_time);
	}

	motor1.IsActive();
	// motor2.IsActive();

	static uint32_t half_second = 0;
	if (current_time - half_second > 250)
	{
		half_second = current_time;

		Draw();
	}

	static uint32_t last_tick = 0;
	// CANManager checks data every 300 ms
	if (current_time - last_tick > 300)
	{
		last_tick = current_time;

		// do all stuff and process RX frames
		can_manager.process();

		// send TX frames if there are any
		if (can_manager.has_tx_frames_for_transmission())
		{
			// CAN_Send_All_Frames(can_manager);
		}
	}

	return;
}

#include <string.h>

char LCDCurrentString[4][20] = {0x20};
char buffer[4][20];

char chars_gear[] = {'N', 'F', 'R', 'L'};
char chars_rool[] = {'S', 'F', 'R', 'r'};

void Draw()
{
	memset(&buffer, 0x20, sizeof(buffer));

	sprintf(buffer[0], "RPM: %04d Gear: %c %02X", motor_ecu_params.motor_1.RPM,
			chars_gear[motor_ecu_params.motor_1.Gear], motor_ecu_params.motor_1.D2);
	sprintf(buffer[1], "V:   %02d.%d Roll: %c", (motor_ecu_params.motor_1.Voltage / 10), (motor_ecu_params.motor_1.Voltage % 10),
			chars_rool[motor_ecu_params.motor_1.Roll]);
	sprintf(buffer[2], "A: %04d.%d Tm: %02d E%02X", (motor_ecu_params.motor_1.Current / 4), (motor_ecu_params.motor_1.Current % 4),
			motor_ecu_params.motor_1.TMotor, (motor_ecu_params.motor_1.Errors >> 8));
	sprintf(buffer[3], "W: %05d  Tc: %02d e%02X", (uint16_t)((motor_ecu_params.motor_1.Voltage / 10.0) * (motor_ecu_params.motor_1.Current / 4.0)),
			motor_ecu_params.motor_1.TController, (motor_ecu_params.motor_1.Errors & 0xFF));

	for (uint8_t i = 0; i < sizeof(buffer); ++i)
	{
		uint8_t a = i % sizeof(buffer[0]);
		uint8_t b = i / sizeof(buffer[0]);

		if (buffer[b][a] != LCDCurrentString[b][a])
		{
			LCD.setCursor(a, b);
			LCD.print(buffer[b][a]);
			LCDCurrentString[b][a] = buffer[b][a];
		}
	}
}
