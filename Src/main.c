/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "string.h"

#define PressureSensorAddr (0x28 << 1)
#define DS3231Addr (0b1101000 << 1)
#define GeigerAddr (0x08 << 1)
#define MAX30205Addr (0b1001000 << 1)
#define HDC1080Addr (0b1000000 << 1)
#define INA3221Addr (0b1000010 << 1)

#define L3GD20HAddr     (0b1101011 << 1)	//gyro
#define LSM303DAddr  	(0b0011101 << 1)	//acc and magn
#define LPS331APAddr  	(0b1011101 << 1)	//baro

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId CommandTaskHandle;
osThreadId LoraTaskHandle;
osThreadId WIFITaskHandle;
osThreadId BLETaskHandle;
osThreadId WatchdogTaskHandle;
osThreadId DS18TaskHandle;
osMessageQId qFromGPSHandle;
osMessageQId qToLoraHandle;
osMessageQId qFromLoraHandle;
osMessageQId qToWatchdogHandle;
osMessageQId qToWIFIHandle;
osMessageQId qToBLEHandle;
osMessageQId qFromDS18Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS my_fatfs;
FIL my_file;
UINT my_error;

static const uint64_t DSAddr[7] =
{ 0x28997F8D0A000015, 0x28197C8D0A0000B1, 0x28F44E8D0A0000D5,
		0x28545C8D0A000076, 0x28545C8D0A000076, 0x28545C8D0A000076,
		0x28545C8D0A000076 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
void StartCommandTask(void const * argument);
void StartLoraTask(void const * argument);
void StartWIFITask(void const * argument);
void StartBLETask(void const * argument);
void StartWatchdogTask(void const * argument);
void StartDS18Task(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC_Init();
	MX_I2C2_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	MX_SPI2_Init();
	MX_I2C1_Init();
	MX_SDIO_SD_Init();
	MX_UART4_Init();
	MX_USART1_UART_Init();
	MX_TIM7_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of CommandTask */
	osThreadDef(CommandTask, StartCommandTask, osPriorityHigh, 0, 2048);
	CommandTaskHandle = osThreadCreate(osThread(CommandTask), NULL);

	/* definition and creation of LoraTask */
	osThreadDef(LoraTask, StartLoraTask, osPriorityNormal, 0, 128);
	LoraTaskHandle = osThreadCreate(osThread(LoraTask), NULL);

	/* definition and creation of WIFITask */
	osThreadDef(WIFITask, StartWIFITask, osPriorityNormal, 0, 128);
	WIFITaskHandle = osThreadCreate(osThread(WIFITask), NULL);

	/* definition and creation of BLETask */
	osThreadDef(BLETask, StartBLETask, osPriorityNormal, 0, 128);
	BLETaskHandle = osThreadCreate(osThread(BLETask), NULL);

	/* definition and creation of WatchdogTask */
	osThreadDef(WatchdogTask, StartWatchdogTask, osPriorityNormal, 0, 128);
	WatchdogTaskHandle = osThreadCreate(osThread(WatchdogTask), NULL);

	/* definition and creation of DS18Task */
	osThreadDef(DS18Task, StartDS18Task, osPriorityIdle, 0, 128);
	DS18TaskHandle = osThreadCreate(osThread(DS18Task), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s) */
	/* definition and creation of qFromGPS */
	osMessageQDef(qFromGPS, 256, uint8_t);
	qFromGPSHandle = osMessageCreate(osMessageQ(qFromGPS), NULL);

	/* definition and creation of qToLora */
	osMessageQDef(qToLora, 30, uint8_t);
	qToLoraHandle = osMessageCreate(osMessageQ(qToLora), NULL);

	/* definition and creation of qFromLora */
	osMessageQDef(qFromLora, 10, uint8_t);
	qFromLoraHandle = osMessageCreate(osMessageQ(qFromLora), NULL);

	/* definition and creation of qToWatchdog */
	osMessageQDef(qToWatchdog, 5, uint8_t);
	qToWatchdogHandle = osMessageCreate(osMessageQ(qToWatchdog), NULL);

	/* definition and creation of qToWIFI */
	osMessageQDef(qToWIFI, 4, uint8_t);
	qToWIFIHandle = osMessageCreate(osMessageQ(qToWIFI), NULL);

	/* definition and creation of qToBLE */
	osMessageQDef(qToBLE, 4, uint8_t);
	qToBLEHandle = osMessageCreate(osMessageQ(qToBLE), NULL);

	/* definition and creation of qFromDS18 */
	osMessageQDef(qFromDS18, 7, uint16_t);
	qFromDS18Handle = osMessageCreate(osMessageQ(qFromDS18), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
	hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
	hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.NbrOfConversion = 1;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 3;

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;

	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 32;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 65535;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART1 init function */
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
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
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
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			D5_Pin | D4_Pin | D3_Pin | D2_Pin | LoRa_RST_Pin | LoRa_RX_Pin
					| RAD_CS_Pin | WIFI_GPIO0_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			D1_Pin | D0_Pin | LoRa_CS_Pin | WiFi_RST_Pin | WiFi_CH_PD_Pin
					| WiFi_GPIO2_Pin | BLE_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			LoRa_TX_Pin | Thermal_Signal_Pin | Buzzer_Signal_Pin | BLE_KEY_Pin
					| OneWire_PULLUP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OneWire_GPIO_Port, OneWire_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : D5_Pin D4_Pin D3_Pin D2_Pin
	 LoRa_RX_Pin RAD_CS_Pin WIFI_GPIO0_Pin */
	GPIO_InitStruct.Pin = D5_Pin | D4_Pin | D3_Pin | D2_Pin | LoRa_RX_Pin
			| RAD_CS_Pin | WIFI_GPIO0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : D1_Pin D0_Pin LoRa_CS_Pin WiFi_RST_Pin
	 WiFi_CH_PD_Pin WiFi_GPIO2_Pin BLE_RST_Pin */
	GPIO_InitStruct.Pin = D1_Pin | D0_Pin | LoRa_CS_Pin | WiFi_RST_Pin
			| WiFi_CH_PD_Pin | WiFi_GPIO2_Pin | BLE_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LoRa_RST_Pin */
	GPIO_InitStruct.Pin = LoRa_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(LoRa_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LoRa_TX_Pin Thermal_Signal_Pin Buzzer_Signal_Pin BLE_KEY_Pin */
	GPIO_InitStruct.Pin = LoRa_TX_Pin | Thermal_Signal_Pin | Buzzer_Signal_Pin
			| BLE_KEY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : OneWire_Pin OneWire_PULLUP_Pin */
	GPIO_InitStruct.Pin = OneWire_Pin | OneWire_PULLUP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : Button_Pin */
	GPIO_InitStruct.Pin = Button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void delayMicroseconds(uint32_t time)
{
	__HAL_TIM_SET_COUNTER(&htim7, 0);
	__HAL_TIM_ENABLE(&htim7);
	while (__HAL_TIM_GET_COUNTER(&htim7) <= time)
		;
	__HAL_TIM_DISABLE(&htim7);
}

static void oneWireResetPulse(void)
{
	HAL_GPIO_WritePin(OneWire_GPIO_Port, OneWire_Pin, GPIO_PIN_RESET);
	for (int i = 1; i <= 48; ++i)
	{
		delayMicroseconds(10);
	}
	HAL_GPIO_WritePin(OneWire_GPIO_Port, OneWire_Pin, GPIO_PIN_SET);
	for (int i = 1; i <= 150; ++i)
	{
		delayMicroseconds(10);
	}
}

static void oneWireSendBit(uint16_t bit)
{
	if (bit == 0)
	{
		HAL_GPIO_WritePin(OneWire_GPIO_Port, OneWire_Pin, GPIO_PIN_RESET);
		delayMicroseconds(65);
		HAL_GPIO_WritePin(OneWire_GPIO_Port, OneWire_Pin, GPIO_PIN_SET);
		delayMicroseconds(10);
	}
	else
	{
		HAL_GPIO_WritePin(OneWire_GPIO_Port, OneWire_Pin, GPIO_PIN_RESET);
		delayMicroseconds(10);
		HAL_GPIO_WritePin(OneWire_GPIO_Port, OneWire_Pin, GPIO_PIN_SET);
		delayMicroseconds(65);
	}
}

static uint16_t oneWireReadBit(void)
{
	uint16_t bit = 0;
	HAL_GPIO_WritePin(OneWire_GPIO_Port, OneWire_Pin, GPIO_PIN_RESET);
	delayMicroseconds(5);
	HAL_GPIO_WritePin(OneWire_GPIO_Port, OneWire_Pin, GPIO_PIN_SET);
	delayMicroseconds(5);

	if (HAL_GPIO_ReadPin(OneWire_GPIO_Port, OneWire_Pin) == GPIO_PIN_SET)
		bit = 1;
	else
		bit = 0;
	delayMicroseconds(55);
	return bit;
}

static void oneWireSendByte(uint16_t value)
{
	uint16_t i, tmp;
	for (i = 0; i < 8; ++i)
	{
		tmp = value >> i;
		tmp &= 0x01;
		oneWireSendBit(tmp);
	}
}

static uint16_t oneWireReadByte(void)
{
	uint16_t i, value = 0;
	for (i = 0; i < 8; ++i)
	{
		if (oneWireReadBit())
			value |= 0x01 << i;
	}
	return value;
}

static void DS18B20ConvertTemperature()
{
	__disable_irq();
	HAL_GPIO_WritePin(OneWire_PULLUP_GPIO_Port, OneWire_PULLUP_Pin,
			GPIO_PIN_SET);
	oneWireResetPulse();
	oneWireSendByte(0xCC);
	oneWireSendByte(0x44);
	__enable_irq();
	osDelay(750);
	HAL_GPIO_WritePin(OneWire_PULLUP_GPIO_Port, OneWire_PULLUP_Pin,
			GPIO_PIN_RESET);
}

static int16_t ReadDS18B20(uint64_t ROM)
{
	int16_t iTemperatura = 0;
	uint8_t CalculatedCRC = 0;
	uint8_t memory[9];
	__disable_irq();
	oneWireResetPulse();
	oneWireSendByte(0x55);
	oneWireSendByte((ROM >> 56) & 0xFF);
	oneWireSendByte((ROM >> 48) & 0xFF);
	oneWireSendByte((ROM >> 40) & 0xFF);
	oneWireSendByte((ROM >> 32) & 0xFF);
	oneWireSendByte((ROM >> 24) & 0xFF);
	oneWireSendByte((ROM >> 16) & 0xFF);
	oneWireSendByte((ROM >> 8) & 0xFF);
	oneWireSendByte(ROM & 0xFF);
	oneWireSendByte(0xBE);
	for (int i = 0; i <= 8; ++i)
	{
		memory[i] = oneWireReadByte();
	}
	oneWireResetPulse();
	__enable_irq();
	for (uint8_t i = 0; i < 8; i++)
	{
		uint8_t inbyte = memory[i];
		for (uint8_t j = 0; j < 8; j++)
		{
			uint8_t mix = (CalculatedCRC ^ inbyte) & 0x01;
			CalculatedCRC >>= 1;
			if (mix)
				CalculatedCRC ^= 0x8C;
			inbyte >>= 1;
		}
	}
	memory[1] &= 0x0F;
	memory[1] = (memory[1] << 4);
	iTemperatura = (memory[1] << 4);
	iTemperatura += memory[0];
	iTemperatura = (iTemperatura * 100) / 16;
	if (CalculatedCRC != memory[8])
		iTemperatura = 40400;
	return iTemperatura;
}

static inline void WriteToSD(uint8_t mode, uint8_t* buf, uint8_t len)
{
	FRESULT fresult;
	const char my_file_name[] = "datalog.bin";
	uint32_t fileSize;
	fresult = f_mount(&my_fatfs, SD_Path, 1);
	if (fresult == FR_OK)
	{
		fresult = f_open(&my_file, my_file_name,
		FA_WRITE | FA_OPEN_ALWAYS);
		fileSize = f_size(&my_file);
		if (fresult == FR_OK)
		{
			fresult = f_lseek(&my_file, fileSize);
			if (mode == 0)
			{
				fresult = f_write(&my_file, (const void*)0x00, 1, &my_error);
				fresult = f_write(&my_file, buf, len, &my_error);
			}
			else
			{
				fresult = f_write(&my_file, (const void*)0x01, 1, &my_error);
				fresult = f_write(&my_file, buf, len, &my_error);
			}
			if (fresult == FR_OK)
			{
				fresult = f_close(&my_file);
				if (fresult == FR_OK)
				{
					HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
				}
				else
				{
					HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
				}

			}
			else
			{
				HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
			}
		}
		else
		{
			HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
		}
	}
	else
	{
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
	}
	fresult = f_mount(0, SD_Path, 0);
}

static uint8_t ReadI2CSensors(uint8_t* buf)
{
	uint8_t i2cData[3];
	uint16_t calc;
	uint8_t returnCode = 0;
	memset(i2cData, 0, sizeof(i2cData));
	if (HAL_I2C_Mem_Read(&hi2c2, DS3231Addr, 0x00, 1, i2cData, 3, 100)
			== HAL_OK)
	{
		buf[0] = ((i2cData[0] & 0b01110000) >> 4) * 10
				+ (i2cData[0] & 0b00001111);
		buf[1] = ((i2cData[1] & 0b01110000) >> 4) * 10
				+ (i2cData[1] & 0b00001111);
		buf[2] = ((i2cData[2] & 0b01110000) >> 4) * 10
				+ (i2cData[2] & 0b00001111);
	}
	else
	{
		returnCode |= 0x01;
	}

	memset(i2cData, 0, sizeof(i2cData));

	if (HAL_I2C_Master_Receive(&hi2c2, PressureSensorAddr, (uint8_t*) i2cData,
			2, 100) == HAL_OK)
	{
		calc = (((i2cData[0] << 8) + i2cData[1]) - 1638) * 15 * 100 / 13107;
		buf[3] = (uint8_t) (calc >> 8);
		buf[4] = (uint8_t) (calc & 0xFF);
	}
	else
	{
		returnCode |= 0x02;
	}

	return returnCode;
}
/* USER CODE END 4 */

/* StartCommandTask function */
void StartCommandTask(void const * argument)
{
	/* init code for FATFS */
	MX_FATFS_Init();

	/* USER CODE BEGIN 5 */
	uint8_t SDBuffer[64];
	uint8_t ErrBuff;

	/* Infinite loop */
	for (;;)
	{
		ErrBuff = ReadI2CSensors(SDBuffer);
		if (ErrBuff)
			WriteToSD(1, &ErrBuff, 1);

		WriteToSD(0, SDBuffer, 10);
		HAL_GPIO_TogglePin(D2_GPIO_Port, D2_Pin);
		osDelay(1000);
	}
	/* USER CODE END 5 */
}

/* StartLoraTask function */
void StartLoraTask(void const * argument)
{
	/* USER CODE BEGIN StartLoraTask */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartLoraTask */
}

/* StartWIFITask function */
void StartWIFITask(void const * argument)
{
	/* USER CODE BEGIN StartWIFITask */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartWIFITask */
}

/* StartBLETask function */
void StartBLETask(void const * argument)
{
	/* USER CODE BEGIN StartBLETask */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartBLETask */
}

/* StartWatchdogTask function */
void StartWatchdogTask(void const * argument)
{
	/* USER CODE BEGIN StartWatchdogTask */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartWatchdogTask */
}

/* StartDS18Task function */
void StartDS18Task(void const * argument)
{
	/* USER CODE BEGIN StartDS18Task */
	int16_t DS18B20Temp[7];
	uint32_t BeginTick = 0;
	/* Infinite loop */
	for (;;)
	{
		BeginTick = osKernelSysTick();
		DS18B20ConvertTemperature();
		for (int i = 0; i < 7; i++)
		{
			DS18B20Temp[i] = ReadDS18B20(DSAddr[i]);
			xQueueSend(qFromDS18Handle, &DS18B20Temp[i], 10);
		}
		osDelayUntil(&BeginTick, 5000);
	}
	/* USER CODE END StartDS18Task */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM2)
	{
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
