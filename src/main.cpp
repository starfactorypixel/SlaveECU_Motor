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

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <Leds.h>
#include <CANLogic.h>
#include <MotorLogic.h>
// #include "FardriverController.h"

ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1; // debug log
UART_HandleTypeDef huart2; // motor 1
UART_HandleTypeDef huart3; // motor 2

/* Private variables ---------------------------------------------------------*/

// For Timers
uint32_t Timer1 = 0;

uint32_t odometer_last_update = 0;

//------------------------ For UART
#define UART_BUFFER_SIZE 128

uint8_t huart2_rx_buff_hot[UART_BUFFER_SIZE] = {0};
uint8_t huart2_rx_buff_cold[UART_BUFFER_SIZE] = {0};
uint16_t huart2_bytes_received = 0;
bool huart2_rx_flag = false;

uint8_t huart3_rx_buff_hot[UART_BUFFER_SIZE] = {0};
uint8_t huart3_rx_buff_cold[UART_BUFFER_SIZE] = {0};
uint16_t huart3_bytes_received = 0;
bool huart3_rx_flag = false;
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

//-------------------------------- Прерывание от USART по флагу Idle
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // motor 1
    if (huart->Instance == USART2)
    {
        huart2_bytes_received = Size;
        memcpy(huart2_rx_buff_cold, huart2_rx_buff_hot, Size);
        huart2_rx_flag = true;
        HAL_UARTEx_ReceiveToIdle_IT(&huart2, huart2_rx_buff_hot, UART_BUFFER_SIZE);
    }

    // motor 2
    if (huart->Instance == USART3)
    {
        huart3_bytes_received = Size;
        memcpy(huart3_rx_buff_cold, huart3_rx_buff_hot, Size);
        huart3_rx_flag = true;
        HAL_UARTEx_ReceiveToIdle_IT(&huart3, huart3_rx_buff_hot, UART_BUFFER_SIZE);
    }
}

/// @brief Callback function of CAN receiver.
/// @param hcan Pointer to the structure that contains CAN configuration.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        CANLib::can_manager.IncomingCANFrame(RxHeader.StdId, RxData, RxHeader.DLC);
        // LOG("RX: CAN 0x%04lX", RxHeader.StdId);
    }
}

/// @brief Callback function for CAN error handler
/// @param hcan Pointer to the structure that contains CAN configuration.
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t er = HAL_CAN_GetError(hcan);
    LOG("CAN ERROR: %lu %08lX", (unsigned long)er, (unsigned long)er);
}

/// @brief Sends data via CAN bus
/// @param id CANObject ID
/// @param data Pointer to the CAN frame data buffer (8 bytes max)
/// @param length Length of the CAN frame data buffer
void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    uint32_t TxMailbox = 0;

    TxHeader.StdId = id;                   // Standard frame ID (sets to 0 if extended one used)
    TxHeader.ExtId = 0;                    // Extended frame ID (sets to 0 if standard one used)
    TxHeader.RTR = CAN_RTR_DATA;           // CAN_RTR_DATA: CAN frame with data will be sent
                                           // CAN_RTR_REMOTE: remote CAN frame will be sent
    TxHeader.IDE = CAN_ID_STD;             // CAN_ID_STD: CAN frame with standard ID
                                           // CAN_ID_EXT: CAN frame with extended ID
    TxHeader.DLC = length;                 // Data length of the CAN frame
    TxHeader.TransmitGlobalTime = DISABLE; // Time Triggered Communication Mode

    memcpy(TxData, data, length);

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        Leds::ledsObj.SetOn(Leds::ledsObj.LED_YELLOW);
    }
    Leds::ledsObj.SetOff(Leds::ledsObj.LED_YELLOW);

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        LOG("CAN TX ERROR: 0x%04lX", TxHeader.StdId);
    }
}

/// @brief Callback function: It is called when correct packet from motor controller PCB is received.
/// @param motor_idx Index of the motor
/// @param raw_packet Pointer to the structure with data.
void OnMotorEvent(const uint8_t motor_idx, motor_packet_raw_t *raw_packet)
{
    if (motor_idx > 1 || motor_idx == 0)
        return;

    uint8_t idx = motor_idx - 1;

    switch (raw_packet->_A1)
    {
    case 0x00:
    {
        motor_packet_0_t *packet0 = (motor_packet_0_t *)raw_packet;
        CANLib::obj_controller_rpm.SetValue(idx, packet0->RPM, CAN_TIMER_TYPE_NORMAL);

        // TODO: Длина окружности колеса захардкожена!!
        #warning Wheel length is a const hardcoded value!
        // TODO: Optimization of speed calculation needed!
        // D=0.57m, WHEEL_LENGTH=Pi*D и делим на 100 для скорости в 100м/ч
        // при RPM >= 61019 об/мин получим переполнение
        CANLib::obj_controller_speed.SetValue(idx, (uint16_t)(60 * packet0->RPM * 0.0179), CAN_TIMER_TYPE_NORMAL);
        
        CANLib::obj_controller_gear_n_roll.SetValue(2 * idx, packet0->Gear, CAN_TIMER_TYPE_NORMAL);
        CANLib::obj_controller_gear_n_roll.SetValue(2 * idx + 1, packet0->Roll, CAN_TIMER_TYPE_NORMAL);

        uint16_t spd1 = CANLib::obj_controller_speed.GetTypedValue(0);
        uint16_t spd2 = CANLib::obj_controller_speed.GetTypedValue(1);
        uint16_t avg_spd = (spd1 & spd2) + ((spd1 ^ spd2) >> 1);
        uint32_t odometer_value = CANLib::obj_controller_odometer.GetTypedValue(0);
        odometer_value += avg_spd * (HAL_GetTick() - odometer_last_update) / 3600000;
        odometer_last_update = HAL_GetTick();
        CANLib::obj_controller_odometer.SetValue(0, odometer_value, CAN_TIMER_TYPE_NORMAL);
        break;
    }

    case 0x01:
    {
        motor_packet_1_t *packet1 = (motor_packet_1_t *)raw_packet;
        CANLib::obj_controller_voltage.SetValue(idx, packet1->Voltage, CAN_TIMER_TYPE_NORMAL);
        CANLib::obj_controller_current.SetValue(idx, (packet1->Current >> 2) * 10, CAN_TIMER_TYPE_NORMAL);
        CANLib::obj_controller_power.SetValue(idx, packet1->Voltage * (packet1->Current >> 2) * 10, CAN_TIMER_TYPE_NORMAL);
        break;
    }

    case 0x04:
    {
        // Градусы : uint8, но до 200 градусов. Если больше то int8
        CANLib::obj_controller_temperature.SetValue(2 * idx + 1, (raw_packet->D2 <= 200) ? (uint8_t)raw_packet->D2 : (int8_t)raw_packet->D2, CAN_TIMER_TYPE_NORMAL);
        break;
    }

    case 0x0D:
    {
        // Градусы : uint8, но до 200 градусов. Если больше то int8
        CANLib::obj_controller_temperature.SetValue(2 * idx, (raw_packet->D0 <= 200) ? (uint8_t)raw_packet->D0 : (int8_t)raw_packet->D0, CAN_TIMER_TYPE_NORMAL);
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
    if (motor_idx > 1 || motor_idx == 0)
        return;

    CANLib::obj_controller_errors.SetValue(motor_idx - 1, (uint16_t)code, CAN_TIMER_TYPE_NORMAL, CAN_EVENT_TYPE_NORMAL);
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
    HAL_UART_Receive_IT(&huart2, huart2_rx_buff_hot, 16);                       // настроить прерывание huart на прием по достижения количества 16 байт
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, huart3_rx_buff_hot, UART_BUFFER_SIZE); // настроить прерывание huart на прием по флагу Idle
    HAL_UART_Receive_IT(&huart3, huart3_rx_buff_hot, 16);                       // настроить прерывание huart на прием по достижения количества 16 байт

    CANLib::Setup();
    Motors::Setup();

    uint32_t current_time = HAL_GetTick();
    while (1)
    {
        Leds::Loop(current_time);
        CANLib::Loop(current_time);
        Motors::Loop(current_time);

        if (huart2_rx_flag)
        {
            Motors::ProcessBytes(1, huart2_rx_buff_cold, huart2_bytes_received, current_time);
            huart2_rx_flag = false;
        }

        if (huart3_rx_flag)
        {
            Motors::ProcessBytes(2, huart3_rx_buff_cold, huart3_bytes_received, current_time);
            huart3_rx_flag = false;
        }
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
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 4;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
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
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
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
    huart2.Init.BaudRate = 115200;
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
    huart3.Init.BaudRate = 115200;
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
    Leds::ledsObj.SetOn(Leds::ledsObj.LED_GREEN);
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
