#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

#define hDebugUart huart1
#define hMotor1Uart huart2
#define hMotor2Uart huart3

#ifdef __cplusplus
}
#endif

#endif
