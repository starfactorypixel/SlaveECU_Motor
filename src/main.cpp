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
#include <CANLogic.h>
#include "FardriverController.h"

// Green LED is on while free CAN mailboxes are not available.
// When at least one mailbox is free, LED will go off.
#define LedGreen_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define LedGreen_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)

ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Private variables ---------------------------------------------------------*/
FardriverController<1> motor1;
FardriverController<2> motor2;

// For Timers
uint32_t Timer1 = 0;

//------------------------ For UART
#define UART_BUFFER_SIZE 128
uint8_t receiveBuff_huart3[UART_BUFFER_SIZE];
uint8_t receiveBuffStat_huart3[UART_BUFFER_SIZE];
uint16_t ReciveUart3Size = 0;
uint8_t FlagReciveUART3 = 0;

uint8_t receiveBuff_huart2[UART_BUFFER_SIZE];
uint8_t receiveBuffStat_huart2[UART_BUFFER_SIZE];
uint16_t ReciveUart2Size = 0;
uint8_t FlagReciveUART2 = 0;

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

void OnMotorEvent(const uint8_t motor_idx, motor_packet_raw_t *raw_packet)
{
    /*
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
*/
    return;
}

void OnMotorError(const uint8_t motor_idx, const motor_error_t code)
{
    return;
}

void OnMotorTX(const uint8_t motor_idx, const uint8_t *raw, const uint8_t raw_len)
{
    // Serial.write(raw, raw_len);

    return;
}

//-------------------------------- Прерывание от USART по заданному ранее количеству байт
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        // что нибудь делаем
        HAL_UART_Receive_IT(&huart2, (uint8_t *)receiveBuff_huart2, 16); // настроить прерывание huart на прием по достижения количества 16 байт
    }
    if (huart == &huart3)
    {
        // что нибудь делаем
        HAL_UART_Receive_IT(&huart3, (uint8_t *)receiveBuff_huart3, 16); // настроить прерывание huart на прием по достижения количества 16 байт
    }
}

//-------------------------------- Прерывание от USART по флагу Idle
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        ReciveUart3Size = Size;
        if (Size == 16 && receiveBuff_huart3[0] == 0x05)
        {
            FlagReciveUART3 = 1;

            // переписать пакет в буфер для хранения
            for (uint16_t i = 0; i != Size; i++)
            {
                receiveBuffStat_huart3[i] = receiveBuff_huart3[i];
            }
            HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *)receiveBuff_huart3, 1000); // настроить huart на прием следующего пакета
        }
        else
        {
            HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t *)receiveBuff_huart3, 1000); // настроить huart на прием следующего пакета
        }
    }
    if (huart->Instance == USART2)
    {
        ReciveUart2Size = Size;
        if (Size == 16 && receiveBuff_huart2[0] == 0x05)
        {
            FlagReciveUART2 = 1;

            // переписать пакет в буфер для хранения
            for (uint16_t i = 0; i != Size; i++)
            {
                receiveBuffStat_huart2[i] = receiveBuff_huart2[i];
            }
            HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)receiveBuff_huart2, 200); // настроить huart на прием следующего пакета
        }
        else
        {
            HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t *)receiveBuff_huart2, 200); // настроить huart на прием следующего пакета
        }
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
        LedGreen_ON;
    }
    LedGreen_OFF;

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        LOG("CAN TX ERROR: 0x%04lX", TxHeader.StdId);
    }
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

/// @brief Make all LEDs blink on startup
void InitialLEDblinks()
{
    LedGreen_OFF;

    LedGreen_ON;
    HAL_Delay(500);
    LedGreen_OFF;
};

/// @brief The application entry point.
/// @return int
int main()
{
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    InitPeripherals();
    InitialLEDblinks();

    /* активируем события которые будут вызывать прерывания  */
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);

    HAL_CAN_Start(&hcan);

    // Настройка прерывания от uart (нужное раскоментировать)
    //	HAL_UARTEx_ReceiveToIdle_IT(&huart3, (uint8_t*) receiveBuff_huart3, 1000);		// настроить прерывание huart на прием по флагу Idle
    //	HAL_UART_Receive_IT(&huart3, (uint8_t*)receiveBuff_huart3, 16);	 // настроить прерывание huart на прием по достижения количества 16 байт

    CANLib::Setup();

    motor1.SetEventCallback(OnMotorEvent);
    // motor2.SetEventCallback(OnMotorEvent);
    motor1.SetErrorCallback(OnMotorError);
    // motor2.SetErrorCallback(OnMotorError);
    motor1.SetTXCallback(OnMotorTX);
    // motor2.SetTXCallback(OnMotorTX);

    uint32_t current_time = HAL_GetTick();
    while (1)
    {
        CANLib::Loop(current_time);

        motor1.Processing(current_time);
        // motor2.Processing(current_time);

        /*
        while (Serial.available() > 0)
        {
            motor1.RXByte(Serial.read(), current_time);
            // motor2.RXByte( Serial.read() , current_time);
        }
        */

        motor1.IsActive();
        // motor2.IsActive();

        // HAL_UART_Transmit(&huart2, RxData, 8, 100);
        // HAL_UART_Transmit(&huart3, RxData, 8, 100);
        // LedGreen_ON;
        // HAL_Delay(50);
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
    /* User can add his own implementation to report the HAL error return state */
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
