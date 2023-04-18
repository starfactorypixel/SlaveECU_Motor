#include <Arduino.h>
#include "FardriverController.h"

FardriverController<1> motor1;
FardriverController<2> motor2;


void OnMotorError(uint8_t num, motor_error_t code)
{

}

void setup()
{


	motor1.SetErrorCallback(OnMotorError);
	motor2.SetErrorCallback(OnMotorError);
}


void loop()
{
	static uint32_t current_time = millis();
	
	motor1.Processing(current_time);
	motor2.Processing(current_time);
}
