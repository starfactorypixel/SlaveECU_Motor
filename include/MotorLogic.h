#pragma once

#include <FardriverController.h>
#include <stm32f1xx_hal.h>

void OnMotorEvent(const uint8_t motor_idx, motor_packet_raw_t *raw_packet);
void OnMotorError(const uint8_t motor_idx, const motor_error_t code);
void OnMotorTX(const uint8_t motor_idx, const uint8_t *raw, const uint8_t raw_len);

namespace Motors
{
    FardriverController<1> motor1;
    FardriverController<2> motor2;

    inline void Setup()
    {
        motor1.SetEventCallback(OnMotorEvent);
        motor2.SetEventCallback(OnMotorEvent);
        motor1.SetErrorCallback(OnMotorError);
        motor2.SetErrorCallback(OnMotorError);
        motor1.SetTXCallback(OnMotorTX);
        motor2.SetTXCallback(OnMotorTX);
    }

    inline void Loop(uint32_t &current_time)
    {
        motor1.Processing(current_time);
        motor2.Processing(current_time);

        current_time = HAL_GetTick();
    }

    inline void ProcessBytes(const uint8_t motor_idx, uint8_t *data, uint16_t data_length, uint32_t &current_time)
    {
        FardriverControllerInterface *motor = nullptr;
        switch (motor_idx)
        {
        case 1:
            motor = &motor1;
            break;

        case 2:
            motor = &motor2;
            break;

        default:
            return;
        }

        for (uint16_t i = 0; i < data_length; i++)
        {
            motor->RXByte(data[i], current_time);
        }
    }
}
