/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "main.h"

#include <cmath>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <ConstantLibrary.h>
#include <LoggerLibrary.h>
#include <About.h>
#include <Leds.h>
#include <CANLogic.h>
#include <MotorLogic.h>

ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef hDebugUart; // debug log
UART_HandleTypeDef huart2; // motor 1
UART_HandleTypeDef huart3; // motor 2

/* Private variables ---------------------------------------------------------*/

// For Timers
uint32_t Timer1 = 0;

uint32_t odometer_last_update = 0;

// Hardcoded speed calc.
float WheelDiameter = 680;							// Диаметр колеса, мм.
float WheelLenght = M_PI * WheelDiameter;			// Длина колеса, мм.
uint32_t SpeedCoef = (WheelLenght * 60.0F) + 0.5F;	// Коэффициент скорости, просто добавить RPM и поделить на 100000.

//------------------------ For UART
#define UART_BUFFER_SIZE 128

uint8_t huart2_rx_buff_hot[UART_BUFFER_SIZE] = {0};
//uint8_t huart2_rx_buff_cold[UART_BUFFER_SIZE] = {0};
//uint16_t huart2_bytes_received = 0;
//bool huart2_rx_flag = false;

uint8_t huart3_rx_buff_hot[UART_BUFFER_SIZE] = {0};
//uint8_t huart3_rx_buff_cold[UART_BUFFER_SIZE] = {0};
//uint16_t huart3_bytes_received = 0;
//bool huart3_rx_flag = false;
//------------------------

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);

/*
//-------------------------------- Прерывание от USART по заданному ранее количеству байт
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // motor 1
    if (huart->Instance == USART2)
    {
        huart2_bytes_received = 16;
        memcpy(huart2_rx_buff_cold, huart2_rx_buff_hot, 16);
        huart2_rx_flag = true;
        HAL_UART_Receive_IT(&huart2, huart2_rx_buff_hot, 16);
    }

    // motor 2
    if (huart->Instance == USART3)
    {
        huart3_bytes_received = 16;
        memcpy(huart3_rx_buff_cold, huart3_rx_buff_hot, 16);
        huart3_rx_flag = true;
        HAL_UART_Receive_IT(&huart3, huart3_rx_buff_hot, 16);
    }
}
*/

//-------------------------------- Прерывание от USART по флагу Idle
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint32_t time = HAL_GetTick();
	
	if(huart->Instance == USART1)
	{

	}
	
	if(huart->Instance == USART2)
	{
		Motors::RXEventProcessing(1, huart2_rx_buff_hot, Size, time);
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, huart2_rx_buff_hot, UART_BUFFER_SIZE);
	}

	if(huart->Instance == USART3)
	{
		Motors::RXEventProcessing(2, huart3_rx_buff_hot, Size, time);
		HAL_UARTEx_ReceiveToIdle_IT(&huart3, huart3_rx_buff_hot, UART_BUFFER_SIZE);
	}

	return;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        DEBUG_LOG_TOPIC("uart2", "ERR: %d\r\n", huart->ErrorCode);

        HAL_UART_AbortReceive_IT(&huart2);
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, huart2_rx_buff_hot, UART_BUFFER_SIZE);
    }

    if(huart->Instance == USART3)
    {
        DEBUG_LOG_TOPIC("uart3", "ERR: %d\r\n", huart->ErrorCode);

        HAL_UART_AbortReceive_IT(&huart3);
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, huart3_rx_buff_hot, UART_BUFFER_SIZE);
    }

   // __HAL_USART_CLEAR_FEFLAG(huart);

    
    
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader = {0};
	uint8_t RxData[8] = {0};
	
	if( HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK )
	{
		CANLib::can_manager.IncomingCANFrame(RxHeader.StdId, RxData, RxHeader.DLC);
	}
	
	return;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	Leds::obj.SetOn(Leds::LED_YELLOW, 100);
	
	DEBUG_LOG_TOPIC("CAN", "RX error event, code: 0x%08lX\n", HAL_CAN_GetError(hcan));
	
	return;
}

void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length)
{
	CAN_TxHeaderTypeDef TxHeader = {0};
	uint8_t TxData[8] = {0};
    uint32_t TxMailbox = 0;
	
	TxHeader.StdId = id;
	TxHeader.ExtId = 0;
	TxHeader.RTR  = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = length;
	TxHeader.TransmitGlobalTime = DISABLE;
	memcpy(TxData, data, length);
	
	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0 )
	{
		Leds::obj.SetOn(Leds::LED_YELLOW);
	}
	Leds::obj.SetOff(Leds::LED_YELLOW);
	
	if( HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK )
	{
		Leds::obj.SetOn(Leds::LED_YELLOW, 100);

		DEBUG_LOG_TOPIC("CAN", "TX error event, code: 0x%08lX\n", HAL_CAN_GetError(&hcan));
	}
	
	return;
}



/// @brief Callback function: It is called when correct packet from motor controller PCB is received.
/// @param motor_idx Index of the motor
/// @param raw_packet Pointer to the structure with data.
void OnMotorEvent(const uint8_t motor_idx, motor_packet_raw_t *raw_packet)
{
    if (motor_idx > 2 || motor_idx == 0)
        return;

    uint8_t idx = motor_idx - 1;

    switch (raw_packet->_A1)
    {
    case 0x00:
    {
        motor_packet_0_t *packet0 = (motor_packet_0_t *)raw_packet;

		// RPM fix. Контроллер возвращает RPMx4.
		packet0->RPM >>= 2;

        CANLib::obj_controller_rpm.SetValue(idx, packet0->RPM, CAN_TIMER_TYPE_NORMAL);

        // TODO: Длина окружности колеса захардкожена!!
        //#warning Wheel length is a const hardcoded value!
        // TODO: Optimization of speed calculation needed!
        // D=0.57m, WHEEL_LENGTH=Pi*D и делим на 100 для скорости в 100м/ч
        // при RPM >= 61019 об/мин получим переполнение
		//CANLib::obj_controller_speed.SetValue(idx, (uint16_t)(60 * packet0->RPM * 0.0179), CAN_TIMER_TYPE_NORMAL);
		CANLib::obj_controller_speed.SetValue(idx, (uint16_t)((SpeedCoef * packet0->RPM) / 100000UL), CAN_TIMER_TYPE_NORMAL);
        
		// TODO: Добавить сюда флаги пониженной передачи и кнопки закиси азота..
		// А пока просто фиксим значения до 2 младших бит.
        CANLib::obj_controller_gear_n_roll.SetValue(2 * idx, FardriverController<>::FixGear(packet0->Gear), CAN_TIMER_TYPE_NORMAL);
        CANLib::obj_controller_gear_n_roll.SetValue(2 * idx + 1, FardriverController<>::FixRoll(packet0->Roll), CAN_TIMER_TYPE_NORMAL);

		DEBUG_LOG_TOPIC("GearRoll", "Motor: %d, Gear: %02X, Roll: %02X;\r\n", motor_idx, packet0->Gear, packet0->Roll);

        uint16_t spd1 = CANLib::obj_controller_speed.GetValue(0);
        uint16_t spd2 = CANLib::obj_controller_speed.GetValue(1);
		//uint16_t avg_spd = (spd1 & spd2) + ((spd1 ^ spd2) >> 1);
		uint16_t avg_spd = ((spd1 + spd2) >> 1);
        uint32_t odometer_value = CANLib::obj_controller_odometer.GetValue(0);
        odometer_value += avg_spd * (HAL_GetTick() - odometer_last_update) / 3600000;
        odometer_last_update = HAL_GetTick();
        CANLib::obj_controller_odometer.SetValue(0, odometer_value, CAN_TIMER_TYPE_NORMAL);
        break;
    }

    case 0x01:
    {
        motor_packet_1_t *packet1 = (motor_packet_1_t *)raw_packet;
        
        int16_t current = (packet1->Current * 10) / 4;
        int16_t power = ((uint32_t)abs(packet1->Current) * (uint32_t)packet1->Voltage) / 40U;
        if(packet1->Current < 0) power = -power;
        
		CANLib::obj_controller_voltage.SetValue(idx, packet1->Voltage, CAN_TIMER_TYPE_NORMAL);
        CANLib::obj_controller_current.SetValue(idx, current, CAN_TIMER_TYPE_NORMAL);
        CANLib::obj_controller_power.SetValue(idx, power, CAN_TIMER_TYPE_NORMAL);
        
		break;
    }

    case 0x04:
    {
        // Градусы : uint8, но до 200 градусов. Если больше то int8
        CANLib::obj_controller_temperature.SetValue(idx, FardriverController<>::FixTemp(raw_packet->D2), CAN_TIMER_TYPE_NORMAL);
        break;
    }

    case 0x0D:
    {
        // Градусы : uint8, но до 200 градусов. Если больше то int8
        CANLib::obj_motor_temperature.SetValue(idx, FardriverController<>::FixTemp(raw_packet->D0), CAN_TIMER_TYPE_NORMAL);
        break;
    }

    default:
        return;
    }
}

/// @brief Callback function: It is called when motor controller reports errors
/// @param motor_idx Index of the motor
/// @param code Motor error code
void OnMotorError(const uint8_t motor_idx, const motor_error_t code)
{
    if (motor_idx > 2 || motor_idx == 0)
        return;

    CANLib::obj_controller_errors.SetValue(motor_idx - 1, (uint16_t)code, CAN_TIMER_TYPE_NORMAL, CAN_EVENT_TYPE_NORMAL);
}

void OnMotorHWError(const uint8_t motor_idx, const uint8_t code)
{
	DEBUG_LOG_TOPIC("MotorErr", "motor: %d, code: %d\r", motor_idx, code);

	uint8_t value_old = CANLib::obj_block_health.GetValue(6);
	uint8_t value_new = (motor_idx == 2) ? ((code << 4) | (value_old & 0x0F)) : (code | (value_old & 0xF0));
	CANLib::obj_block_health.SetValue(6, value_new, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
	#warning Move to BlockError ???
	
	return;
}

/// @brief Callback function: It is called by FardriverController classes for sending data to the PCB of motor controllers.
/// @param motor_idx Index of the motor
/// @param raw Pointer to the raw data buffer for sending
/// @param raw_len Raw data length
void OnMotorTX(const uint8_t motor_idx, const uint8_t *raw, const uint8_t raw_len)
{
    UART_HandleTypeDef *motor_huart = nullptr;
    switch (motor_idx)
    {
    case 1:
        motor_huart = &huart2;
        break;

    case 2:
        motor_huart = &huart3;
        break;

    default:
        return;
    }

    HAL_UART_Transmit(motor_huart, (uint8_t *)raw, raw_len, 100);
}

/// @brief Peripherals initialization: GPIO, DMA, CAN, SPI, USART, ADC, Timers
void InitPeripherals()
{
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_CAN_Init();
    MX_TIM1_Init();
    MX_USART3_UART_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();

    HAL_TIM_Base_Start_IT(&htim1);
};

/// @brief The application entry point.
/// @return int
int main()
{
    HAL_Init();
    SystemClock_Config();
    InitPeripherals();

    // CAN free mailbox error = Yellow LED
    // Global Error handler (infinite loop) = Green LED
    // unused = Red LED
    // unused = Blue LED
    Leds::Setup();

    /* активируем события которые будут вызывать прерывания  */
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);

    HAL_CAN_Start(&hcan);

    // Настройка прерывания от uart (нужное раскоментировать)
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, huart2_rx_buff_hot, UART_BUFFER_SIZE); // настроить прерывание huart на прием по флагу Idle
    //HAL_UART_Receive_IT(&huart2, huart2_rx_buff_hot, 16);                       // настроить прерывание huart на прием по достижения количества 16 байт
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, huart3_rx_buff_hot, UART_BUFFER_SIZE); // настроить прерывание huart на прием по флагу Idle
    //HAL_UART_Receive_IT(&huart3, huart3_rx_buff_hot, 16);                       // настроить прерывание huart на прием по достижения количества 16 байт

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	About::Setup();
    CANLib::Setup();
    Motors::Setup();

	Leds::obj.SetOn(Leds::LED_GREEN, 50, 1950);

    uint32_t current_time = HAL_GetTick();
    while (1)
    {
		About::Loop(current_time);
        Leds::Loop(current_time);
        CANLib::Loop(current_time);
        Motors::Loop(current_time);
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{
    // https://istarik.ru/blog/stm32/159.html

    CAN_FilterTypeDef sFilterConfig;

    // CAN interface initialization
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 4;
    hcan.Init.Mode = CAN_MODE_NORMAL; // CAN_MODE_NORMAL
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;   // DISABLE
    hcan.Init.AutoBusOff = ENABLE;           // DISABLE
    hcan.Init.AutoWakeUp = ENABLE;           // DISABLE
    hcan.Init.AutoRetransmission = DISABLE;  // DISABLE
    hcan.Init.ReceiveFifoLocked = ENABLE;    // DISABLE
    hcan.Init.TransmitFifoPriority = ENABLE; // DISABLE
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }

    // CAN filtering initialization
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    // sFilterConfig.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 63999;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 10;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 89;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
    hDebugUart.Instance = USART1;
    hDebugUart.Init.BaudRate = 500000;
    hDebugUart.Init.WordLength = UART_WORDLENGTH_8B;
    hDebugUart.Init.StopBits = UART_STOPBITS_1;
    hDebugUart.Init.Parity = UART_PARITY_NONE;
    hDebugUart.Init.Mode = UART_MODE_TX_RX;
    hDebugUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hDebugUart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&hDebugUart) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 19200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 19200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PC14 PC15 */
    GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PB8 PB9 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        Timer1++;
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	Leds::obj.SetOn(Leds::LED_RED);
	Leds::obj.SetOff(Leds::LED_YELLOW);
	Leds::obj.SetOff(Leds::LED_GREEN);
	Leds::obj.SetOff(Leds::LED_BLUE);
	
	while (1)
	{

	}
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif // USE_FULL_ASSERT
