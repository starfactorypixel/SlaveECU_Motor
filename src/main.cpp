#include "main.h"
#include <cmath>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <ConstantLibrary.h>
#include <LoggerLibrary.h>
#include <SPI.h>
#include <CANBackEvents.h>
#include <About.h>
#include <Leds.h>
#include <Config.h>
#include <MotorLogic.h>
#include <MotorCtrl.h>
#include <Analog.h>
#include <Temperature.h>
#include <OneWire.h>
#include <CANLogic.h>

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
CAN_HandleTypeDef hcan;
CRC_HandleTypeDef hcrc;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader = {0};
	uint8_t RxData[8] = {0};
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
	{
		CANLib::can_manager.PushFrameToRX(RxHeader.StdId, RxData, RxHeader.DLC);
	}
	
	return;
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CANLib::can_manager.TryToSendCANFrameFromTXQueue();
	
	return;
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CANLib::can_manager.TryToSendCANFrameFromTXQueue();
	
	return;
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CANLib::can_manager.TryToSendCANFrameFromTXQueue();
	
	return;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t code = HAL_CAN_GetError(hcan);
	if((code & (HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_ALST2)) != 0)
		return;
	
	Leds::obj.SetOn(Leds::LED_RED, 100);
	DEBUG_LOG_TOPIC("CAN", "RX error event, code: 0x%08lX\n", HAL_CAN_GetError(hcan));
	HAL_CAN_ResetError(hcan);
	
	return;
}

bool HAL_CAN_Send(can_object_id_t id, uint8_t *TxData, uint8_t length)
{
	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) return false;
	
	uint32_t TxMailbox;
	CAN_TxHeaderTypeDef TxHeader = { id, 0, CAN_ID_STD, CAN_RTR_DATA, length, DISABLE };
	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) == HAL_OK) return true;
	
	Leds::obj.SetOn(Leds::LED_RED, 100);
	DEBUG_LOG_TOPIC("CAN", "TX error event, code: 0x%08lX\n", HAL_CAN_GetError(&hcan));
	
	return false;
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	Motors::data_t *obj;
	
	// USART1 == Debug
	if(huart->Instance == USART1)
	{

	}
	
	// USART2 == MOTOR1
	else if(huart->Instance == USART2)
	{
		obj = &Motors::uart_data[Motors::MOTOR_1];
		
		Motors::UART_RX(Motors::MOTOR_1, Size);
		
		HAL_UARTEx_ReceiveToIdle_IT(obj->hal, obj->hot, sizeof(obj->hot));
	}
	
	// USART3 == MOTOR2
	else if(huart->Instance == USART3)
	{
		obj = &Motors::uart_data[Motors::MOTOR_2];
		
		Motors::UART_RX(Motors::MOTOR_2, Size);
		
		HAL_UARTEx_ReceiveToIdle_IT(obj->hal, obj->hot, sizeof(obj->hot));
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	Motors::data_t *obj;

	// USART1 == Debug
	if(huart->Instance == USART1)
	{
		DEBUG_LOG_TOPIC("USART1", "ERR: %d\n", huart->ErrorCode);
		
		HAL_UART_AbortReceive_IT(&hDebugUart);
	}
	
	// USART2 == MOTOR1
	else if(huart->Instance == USART2)
	{
		obj = &Motors::uart_data[Motors::MOTOR_1];
		
		DEBUG_LOG_TOPIC("USART2", "ERR: %d\n", huart->ErrorCode);
		
		HAL_UART_AbortReceive_IT(obj->hal);
		HAL_UARTEx_ReceiveToIdle_IT(obj->hal, obj->hot, sizeof(obj->hot));
	}
	
	// USART3 == MOTOR2
	else if(huart->Instance == USART3)
	{
		obj = &Motors::uart_data[Motors::MOTOR_2];
		
		DEBUG_LOG_TOPIC("USART3", "ERR: %d\n", huart->ErrorCode);
		
		HAL_UART_AbortReceive_IT(obj->hal);
		HAL_UARTEx_ReceiveToIdle_IT(obj->hal, obj->hot, sizeof(obj->hot));
	}
}


/*
void OnMotorHWError(const uint8_t motor_idx, const uint8_t code)
{
	DEBUG_LOG_TOPIC("MotorErr", "motor: %d, code: %d\r", motor_idx, code);

	uint8_t value_old = CANLib::obj_block_health.GetValue(6);
	uint8_t value_new = (motor_idx == 2) ? ((code << 4) | (value_old & 0x0F)) : (code | (value_old & 0xF0));
	CANLib::obj_block_health.SetValue(6, value_new, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
	#warning Move to BlockError ???
	
	return;
}
*/



/// @brief The application entry point.
/// @return int
int main()
{
	HAL_Init();
	SystemClock_Config();
	
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_CAN_Init();
	MX_CRC_Init();
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM3_Init();
	
	About::Setup();
    Leds::Setup();
	Analog::Setup();
	SPI::Setup();
	Motors::Setup();
	MotorCtrl::Setup();
	OneWire::Setup();
	TempStream::Setup();
	CANLib::Setup();

	uint32_t current_time = HAL_GetTick();
	while(1)
	{
		About::Loop(current_time);
		Leds::Loop(current_time);
		Analog::Loop(current_time);
		SPI::Loop(current_time);
		Motors::Loop(current_time);
		MotorCtrl::Loop(current_time);
		OneWire::Loop(current_time);
		TempStream::Loop(current_time);
		CANLib::Loop(current_time);
	}
}



void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_ADC1_Init(void)
{
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = Analog::regular_channel_count;
	if(HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	
	Analog::RegularConfig();
}

static void MX_ADC2_Init(void)
{
	hadc2.Instance = ADC2;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	if(HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_CAN_Init(void)
{
	CAN_FilterTypeDef sFilterConfig = {0};
	
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 4;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoWakeUp = ENABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = ENABLE;
	hcan.Init.TransmitFifoPriority = ENABLE;
	if(HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
	
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	sFilterConfig.SlaveStartFilterBank = 0;
	if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_CRC_Init(void)
{
	hcrc.Instance = CRC;
	if(HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_SPI2_Init(void)
{
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if(HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_USART1_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 500000;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if(HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
}

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
	if(HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
}

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
	if(HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_TIM3_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 63;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1023;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	
	HAL_TIM_MspPostInit(&htim3);
}

static void MX_DMA_Init(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();
	
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
}

void Error_Handler(void)
{
	__disable_irq();
	
	Leds::obj.SetOff();
	Leds::obj.SetOn(Leds::LED_RED);
	while(1)
	{

	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
