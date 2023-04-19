#include <Arduino.h>
#include <LiquidCrystal.h>
//#include "utils.h"
#include "FardriverController.h"
#include "MotorData.h"

LiquidCrystal LCD(3, 11, 14, 16, 15, 17); // RS, E, D4, D5, D6, D7
void Draw();

FardriverController<1> motor1;
//FardriverController<2> motor2;

MotorData ParsedData;

void OnMotorEvent(const uint8_t motor_idx, motor_packet_raw_t *raw_packet)
{
	switch (raw_packet->_A1)
	{
		case 0x00:
		{
			motor_packet_0_t *data = (motor_packet_0_t*) raw_packet;
			ParsedData.Roll = (data->Roll & 0x03);
			ParsedData.Gear = (data->Gear & 0x03);
			ParsedData.RPM = data->RPM;
			ParsedData.D2 = raw_packet->D2;
			ParsedData.Errors = data->ErrorFlags;
			
			break;
		}
		case 0x01:
		{
			motor_packet_1_t *data = (motor_packet_1_t*) raw_packet;
			ParsedData.Voltage = data->Voltage;
			ParsedData.Current = data->Current;
			
			break;
		}
		case 0x04:
		{
			ParsedData.TController = raw_packet->D2;
			
			break;
		}
		case 0x0D:
		{
			ParsedData.TMotor = raw_packet->D0;
			
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
	//while (!Serial){}
	
	LCD.begin(20, 4);
	LCD.home();
	pinMode(6, OUTPUT);
	digitalWrite(6, HIGH);
	
	motor1.SetEventCallback(OnMotorEvent);
	//motor2.SetEventCallback(OnMotorEvent);
	motor1.SetErrorCallback(OnMotorError);
	//motor2.SetErrorCallback(OnMotorError);
	motor1.SetTXCallback(OnMotorTX);
	//motor2.SetTXCallback(OnMotorTX);
	
	return;
}

void loop()
{
	uint32_t current_time = millis();
	
	motor1.Processing(current_time);
	//motor2.Processing(current_time);

	while(Serial.available() > 0)
	{
		motor1.RXByte( Serial.read() , current_time);
		//motor2.RXByte( Serial.read() , current_time);
	}
	
	motor1.IsActive();
	//motor2.IsActive();

	static uint32_t half_second = 0;
	if(current_time - half_second > 250)
	{
		half_second = current_time;

		Draw();
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
	
	sprintf(buffer[0], "RPM: %04d Gear: %c %02X", ParsedData.RPM, chars_gear[ParsedData.Gear], ParsedData.D2);
	sprintf(buffer[1], "V:   %02d.%d Roll: %c", (ParsedData.Voltage / 10), (ParsedData.Voltage % 10), chars_rool[ParsedData.Roll]);
	sprintf(buffer[2], "A: %04d.%d Tm: %02d E%02X", (ParsedData.Current / 4), (ParsedData.Current % 4), ParsedData.TMotor, (ParsedData.Errors >> 8));
	sprintf(buffer[3], "W: %05d  Tc: %02d e%02X", (uint16_t)((ParsedData.Voltage / 10.0) * (ParsedData.Current / 4.0)), ParsedData.TController, (ParsedData.Errors & 0xFF));
	
	for(uint8_t i = 0; i < sizeof(buffer); ++i)
	{
		uint8_t a = i % sizeof(buffer[0]);
		uint8_t b = i / sizeof(buffer[0]);
		
		if(buffer[b][a] != LCDCurrentString[b][a])
		{
			LCD.setCursor(a, b);
			LCD.print(buffer[b][a]);
			LCDCurrentString[b][a] = buffer[b][a];
		}
	}
}
