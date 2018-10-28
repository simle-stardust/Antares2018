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
#include "stdlib.h"
#include "aes.h"

#define PressureSensorAddr (0x28 << 1)
#define DS3231Addr (0b1101000 << 1)
#define GeigerAddr (0x08 << 1)
#define MAX30205Addr (0b1001000 << 1)
#define HDC1080Addr (0b1000000 << 1)
#define INA3221Addr (0b1000010 << 1)

#define L3GD20HAddr     (0b1101011 << 1)	//gyro
#define LSM303DAddr  	(0b0011101 << 1)	//acc and magn
#define LPS331APAddr  	(0b1011101 << 1)	//baro

typedef struct loraUPframe_t
{
	uint16_t status;
	int16_t alt;
	int16_t temp;
} loraUPframe_t;

typedef struct loraDNframe_t
{
	uint8_t dest;
	uint8_t command;
} loraDNframe_t;

typedef struct ds18b20_t
{
	int16_t t1;
	int16_t t2;
	int16_t t3;
	int16_t t4;
	int16_t t5;
	int16_t t6;
	int16_t t7;
} ds18b20_t;

#define DEV_ADDR                              (uint32_t)0x260113C5
// network session key
static const uint8_t NwkSKey[16] =
{
		0xB8, 0x7E, 0x16, 0x94, 0x08, 0x6E, 0x6C, 0xA0,
		0x39, 0xD8, 0xE8, 0x39, 0x27, 0x8B, 0x6E, 0x10
};

// application session key
static const uint8_t ArtSKey[16] =
{
		0xE9, 0x74, 0xF8, 0xBC, 0xD7, 0x07, 0x47, 0x3F,
		0x05, 0xE2, 0x59, 0x09, 0x77, 0x5F, 0xEF, 0xED
};

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
uint8_t GPSchar;

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
static void UART_Init();
static inline uint8_t setBit(uint8_t value, uint8_t bit);
static inline uint8_t clearBit(uint8_t value, uint8_t bit);
static void writeLoraRegister(uint8_t address, uint8_t data);
static uint8_t readLoraRegister(uint8_t address);
static void Lora_Init();
static uint8_t LoraReceive();
static void LoraTransmitByte(uint8_t *data, uint8_t size);

void USART2_IRQHandler()
{
	if (((USART2->SR) & USART_SR_RXNE) > 0)
		GPSchar = USART2->DR;
	xQueueSendFromISR(qFromGPSHandle, &GPSchar, NULL);
}
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
  osMessageQDef(qFromGPS, 512, uint8_t);
  qFromGPSHandle = osMessageCreate(osMessageQ(qFromGPS), NULL);

  /* definition and creation of qToLora */
  osMessageQDef(qToLora, 5, loraUPframe_t);
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
  osMessageQDef(qFromDS18, 2, ds18b20_t);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, D5_Pin|D4_Pin|D3_Pin|D2_Pin 
                          |LoRa_RST_Pin|LoRa_RX_Pin|RAD_CS_Pin|WIFI_GPIO0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D1_Pin|D0_Pin|LoRa_CS_Pin|WiFi_RST_Pin 
                          |WiFi_CH_PD_Pin|WiFi_GPIO2_Pin|BLE_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LoRa_TX_Pin|Thermal_Signal_Pin|Buzzer_Signal_Pin|BLE_KEY_Pin 
                          |OneWire_PULLUP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OneWire_GPIO_Port, OneWire_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : D5_Pin D4_Pin D3_Pin D2_Pin 
                           LoRa_RX_Pin RAD_CS_Pin WIFI_GPIO0_Pin */
  GPIO_InitStruct.Pin = D5_Pin|D4_Pin|D3_Pin|D2_Pin 
                          |LoRa_RX_Pin|RAD_CS_Pin|WIFI_GPIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin D0_Pin LoRa_CS_Pin WiFi_RST_Pin 
                           WiFi_CH_PD_Pin WiFi_GPIO2_Pin BLE_RST_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D0_Pin|LoRa_CS_Pin|WiFi_RST_Pin 
                          |WiFi_CH_PD_Pin|WiFi_GPIO2_Pin|BLE_RST_Pin;
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
  GPIO_InitStruct.Pin = LoRa_TX_Pin|Thermal_Signal_Pin|Buzzer_Signal_Pin|BLE_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OneWire_Pin OneWire_PULLUP_Pin */
  GPIO_InitStruct.Pin = OneWire_Pin|OneWire_PULLUP_Pin;
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
static void UART_Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	//clock enable for UART1
	//alternate functions of PA9 and PA10 set to UART1 AF=7
	GPIOA->AFR[0] |= (0x00000700 & GPIO_AFRL_AFSEL2);
	GPIOA->AFR[0] |= (0x00007000 & GPIO_AFRL_AFSEL3);
	GPIOA->MODER |= (0x000000F0 & 0x000000A0);
	GPIOA->OSPEEDR |= (0x000000F0 & 0x000000A0);			//high speed
	//GPIOA->PUPDR |= (0x000000F0 & 0x00000050); //pullup

	NVIC_SetPriority(USART2_IRQn, 0x05);
	NVIC_EnableIRQ(USART2_IRQn);			// set enable IRQ

	USART2->BRR = 3333;						//baud rate = 9600
	// usart+transmitter+receiver enable
	USART2->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;

}

static inline uint8_t setBit(uint8_t value, uint8_t bit)
{
	value |= (1 << bit);
	return value;
}

static inline uint8_t clearBit(uint8_t value, uint8_t bit)
{
	value &= ~(1 << bit);
	return value;
}

static void writeLoraRegister(uint8_t address, uint8_t data)
{
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	address = setBit(address, 7);
	HAL_SPI_Transmit(&hspi1, &address, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
}

static uint8_t readLoraRegister(uint8_t address)
{
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	uint8_t value = 0x00;
	address = clearBit(address, 7);
	HAL_SPI_Transmit(&hspi1, &address, 1, 1000);
	HAL_SPI_Receive(&hspi1, &value, 1, 1000);
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
	;
	return value;
}

static void Lora_Init()
{
	HAL_GPIO_WritePin(LoRa_RST_GPIO_Port, LoRa_RST_Pin, GPIO_PIN_SET);
	osDelay(100);
	HAL_GPIO_WritePin(LoRa_RST_GPIO_Port, LoRa_RST_Pin, GPIO_PIN_RESET);
	osDelay(10);
	writeLoraRegister(0x01, 0x00);
	writeLoraRegister(0x01, 0x80);
	writeLoraRegister(0x01, 0x81);
	writeLoraRegister(0x23, 255);
	writeLoraRegister(0x0C, 0x23);
	writeLoraRegister(0x1F, 0x8F);
	writeLoraRegister(0x1D, 0x0B);
	writeLoraRegister(0x1E, 0xC4);
	writeLoraRegister(0x06, 0xD8);
	writeLoraRegister(0x07, 0xFA);
	writeLoraRegister(0x08, 0xE0);
	writeLoraRegister(0x09, 0x0F);
}

static uint8_t LoraReceive()
{
	HAL_GPIO_WritePin(LoRa_RX_GPIO_Port, LoRa_RX_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LoRa_TX_GPIO_Port, LoRa_TX_Pin, GPIO_PIN_RESET);
	writeLoraRegister(0x11, 0xFF);
	writeLoraRegister(0x12, 0b11111111);
	writeLoraRegister(0x11, 0x00);
	writeLoraRegister(0x0D, 0x00);
	uint8_t RxDoneInterrupt = readLoraRegister(0x12);
	uint8_t RxTimeoutInterrupt = RxDoneInterrupt;
	RxDoneInterrupt &= (1 << 6);
	RxTimeoutInterrupt &= (1 << 7);
	writeLoraRegister(0x01, 0x86);
	//HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
	while (RxDoneInterrupt == 0 && RxTimeoutInterrupt == 0)
	{
		RxDoneInterrupt = readLoraRegister(0x12);
		RxTimeoutInterrupt = RxDoneInterrupt;
		RxDoneInterrupt &= (1 << 6);
		RxTimeoutInterrupt &= (1 << 7);
	}
	uint8_t NbofBytes = readLoraRegister(0x13);
	if (RxTimeoutInterrupt > 0)
		NbofBytes = 0;
	//HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
	return NbofBytes;
}

static void LoraTransmitByte(uint8_t *data, uint8_t size)
{
	HAL_GPIO_WritePin(LoRa_TX_GPIO_Port, LoRa_TX_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LoRa_RX_GPIO_Port, LoRa_RX_Pin, GPIO_PIN_RESET);
	writeLoraRegister(0x22, size);
	writeLoraRegister(0x11, 0xFF);
	writeLoraRegister(0x12, 0b11111111);
	writeLoraRegister(0x11, 0x00);
	uint8_t TxDoneInterrupt = readLoraRegister(0x12);
	writeLoraRegister(0x0D, 0x80);
	for (int i = 0; i < size; i++)
	{
		writeLoraRegister(0x00, *data);
		data++;
	}
	writeLoraRegister(0x01, 0x83);
	TxDoneInterrupt = readLoraRegister(0x12);
	TxDoneInterrupt &= (1 << 3);
	while ((TxDoneInterrupt == 0))
	{
		TxDoneInterrupt = readLoraRegister(0x12);
		TxDoneInterrupt &= (1 << 3);
	}
	readLoraRegister(0x01);
}

static void LoraWANTransmitByte(uint8_t* data, uint8_t size)
{
	uint8_t UplinkFrame[20];
	static uint32_t licznik;
    for (int i=0; i<size; i++)
    {
        UplinkFrame[9+i] = data[i];
    }
    UplinkFrame[0] = 0x40; //data unconfirmed up
    UplinkFrame[1] = (uint8_t)(DEV_ADDR & 0xFF);
    UplinkFrame[2] = (uint8_t)(DEV_ADDR >> 8);
    UplinkFrame[3] = (uint8_t)(DEV_ADDR >> 16);
    UplinkFrame[4] = (uint8_t)(DEV_ADDR >> 24);
    UplinkFrame[5] = 0x00; //FCtrl
    UplinkFrame[6] = (uint8_t)(licznik >> 8); //Fcnt msb
    UplinkFrame[7] = (uint8_t)(licznik & 0xFF); //Fcnt lsb
    UplinkFrame[8] = 0x01; //port
    aes_cipher(ArtSKey, DEV_ADDR, licznik, /*up*/0, &UplinkFrame[9], size);
    aes_appendMic(NwkSKey, DEV_ADDR, licznik, /*up*/0, UplinkFrame, size + 9);
    LoraTransmitByte(UplinkFrame, size + 13);
    licznik++;
}

uint8_t LoraWANParseDN(uint8_t* data, uint8_t len)
{
	uint8_t returncode = 0x00;
	uint16_t cnt = ((uint16_t) (data[7] << 8) + (data[6] & 0xFF));
	uint32_t addr = ((uint32_t) ((data[4] << 24) + (data[3] << 16)
			+ (data[2] << 8) + (data[1] & 0xFF)));
	if ((aes_verifyMic(NwkSKey, DEV_ADDR, cnt, 1, data, len - 4))
			&& (addr == DEV_ADDR ))
	{
		returncode = 0x01;
		aes_cipher(ArtSKey, DEV_ADDR, cnt, 1, &data[9], len - 13);
	}
	return returncode;
}

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

static inline uint16_t WriteToSD(uint8_t* buf, uint8_t len)
{
	uint16_t returnCode=0;
	FRESULT fresult;
	const char my_file_name[] = "datalog.txt";
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
			fresult = f_write(&my_file, buf, len, &my_error);
			if (fresult == FR_OK)
			{
				fresult = f_close(&my_file);
				if (fresult == FR_OK)
				{
					//HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
				}
				else
				{
					returnCode = SD_ERR;
					//HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
				}

			}
			else
			{
				returnCode = SD_ERR;
				//HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
			}
		}
		else
		{
			returnCode = SD_ERR;
			//HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
		}
	}
	else
	{
		returnCode = SD_ERR;
		//HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
	}
	fresult = f_mount(0, SD_Path, 0);
	return returnCode;
}

static uint16_t ReadI2CSensors(uint8_t* buf)
{
	uint8_t i2cData[3];
	uint16_t calc;
	uint8_t returnCode = 0;
	memset(i2cData, 0, sizeof(i2cData));
	if (HAL_I2C_Mem_Read(&hi2c2, DS3231Addr, 0x00, 1, i2cData, 3, 100)
			== HAL_OK)
	{
		buf[2] = ((i2cData[0] & 0b01110000) >> 4) * 10
				+ (i2cData[0] & 0b00001111);
		buf[1] = ((i2cData[1] & 0b01110000) >> 4) * 10
				+ (i2cData[1] & 0b00001111);
		buf[0] = ((i2cData[2] & 0b01110000) >> 4) * 10
				+ (i2cData[2] & 0b00001111);
	}
	else
	{
		returnCode |= I2C_RTC_ERR;
	}
	memset(i2cData, 0, sizeof(i2cData));

	if (HAL_I2C_Master_Receive(&hi2c2, PressureSensorAddr, (uint8_t*) i2cData,
			2, 100) == HAL_OK)
	{
		calc = (((i2cData[0] << 8) + i2cData[1]) - 1638) * 15 * 100 / 13107;
		buf[3] = (uint8_t) (calc / 1000);
		buf[4] = (uint8_t) (calc % 1000);
	}
	else
	{
		returnCode |= I2C_PRESSURE_ERR;
	}
	return returnCode;
}

static uint16_t ReadGPS(int16_t* altitude, uint8_t* SDBuf)
{
	uint16_t returnCode=0;
	uint8_t ReceivedGPS;
	uint8_t len;
	char linijkaGPS[100];
	uint8_t GPSReadChecksum = 0;
	uint8_t GPSCalcChecksum = 1;
	char LiterkaLat = 0, LiterkaLon = 0;
	char czasGPS[9];
	char Lattitude[10];
	char Longtitude[10];
	int licznik2 = 0;
	while (xQueueReceive(qFromGPSHandle, &ReceivedGPS, 1))
	{
		if (ReceivedGPS == 0x0A)
		{
			if (strncmp(linijkaGPS, "$GPGGA", 6) == 0) //znaleziono gngga
			{
				GPSCalcChecksum = linijkaGPS[1];
				for (int i = 2; i <= 100; ++i)
				{
					if (linijkaGPS[i] == 0x2A)
						break;
					else
						GPSCalcChecksum ^= linijkaGPS[i];
				}
				char *schowek;
				strtok(linijkaGPS, ",");
				schowek = strtok(NULL, ",");
				strcpy(czasGPS, schowek);
				schowek = strtok(NULL, ",");
				strcpy(Lattitude, schowek);
				schowek = strtok(NULL, ",");
				LiterkaLat = *schowek;
				schowek = strtok(NULL, ",");
				strcpy(Longtitude, schowek);
				schowek = strtok(NULL, ",");
				LiterkaLon = *schowek;
				schowek = strtok(NULL, ",");
				schowek = strtok(NULL, ",");
				schowek = strtok(NULL, ",");
				schowek = strtok(NULL, ",");
				*altitude = (int16_t)strtol(schowek, NULL, 10);
				schowek = strtok(NULL, "*");
				schowek = strtok(NULL, "\r");
				GPSReadChecksum = strtol(schowek, NULL, 16);
			}
			licznik2 = 0;
			memset(linijkaGPS, 0, sizeof linijkaGPS);
		}
		else
		{
			linijkaGPS[licznik2] = ReceivedGPS;
			licznik2++;
			if (licznik2 == 100)
				licznik2 = 0;
		}
	}
	memset(linijkaGPS, 0, sizeof linijkaGPS);
	if (GPSCalcChecksum != GPSReadChecksum)
	{
		//HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_RESET);
		*altitude = 0;
		strcpy(Longtitude, "00000.00000");
		strcpy(Lattitude, "0000.00000");
		strcpy(czasGPS, "000000.00");
		LiterkaLat = 0x20;
		LiterkaLon = 0x20;
		returnCode = GPS_ERR;
	}
	else
	{
		//HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_SET);
	}
	len = sprintf((char*)SDBuf, "%s,%s%c,%s%c,%d, ", czasGPS, Longtitude, LiterkaLon, Lattitude, LiterkaLat, *altitude);
	WriteToSD(SDBuf, len);
	return returnCode;
}

/* USER CODE END 4 */

/* StartCommandTask function */
void StartCommandTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();
  UART_Init();

  /* USER CODE BEGIN 5 */
	uint32_t tick = osKernelSysTick();
	uint8_t SDBuffer[64];
	uint8_t SDBufLen=0;
	uint8_t RawDataBuffer[20];
	loraUPframe_t DataToSend;
	uint16_t ErrBuff = 0;
	ds18b20_t tempbuf;
	uint32_t CycleNb=0;
	int16_t Altitude=0;
	memset(SDBuffer, 0, sizeof(SDBuffer));
	/* Infinite loop */
	for (;;)
	{
		ErrBuff=0;
		tick = osKernelSysTick();
		// --------------- I2C Sensors Readout -------------------------------
		ErrBuff |= ReadI2CSensors(RawDataBuffer);
		SDBufLen += sprintf((char*)SDBuffer, "%d:%d:%d,%d%d,", RawDataBuffer[0], RawDataBuffer[1], RawDataBuffer[2], RawDataBuffer[3], RawDataBuffer[4]);
		WriteToSD(SDBuffer, SDBufLen);
		memset(SDBuffer, 0, sizeof(SDBuffer));
		SDBufLen=0;
		// --------------- DS18B20 Readout ------------------------------------
		xQueueReceive(qFromDS18Handle, &tempbuf, 0);
		SDBufLen += sprintf((char*)SDBuffer, "%d,%d,%d,%d,%d,%d,%d,", tempbuf.t1, tempbuf.t2, tempbuf.t3, tempbuf.t4, tempbuf.t5, tempbuf.t6, tempbuf.t7);
		ErrBuff |= WriteToSD(SDBuffer, SDBufLen);
		if ((tempbuf.t1 == 40400) || (tempbuf.t2 == 40400) || (tempbuf.t3 == 40400) || (tempbuf.t4 == 40400) ||
				(tempbuf.t5 == 40400) || (tempbuf.t6 == 40400) || (tempbuf.t7 == 40400))
		{
			ErrBuff |= DS18_ERR;
		}
		memset(SDBuffer, 0, sizeof(SDBuffer));
		SDBufLen=0;
		// ---------------- GPS Readout ----------------------------------------
		ErrBuff |= ReadGPS(&Altitude, SDBuffer);
		//----------------- LORA Transmission ----------------------------------------
		DataToSend.alt = Altitude;
		DataToSend.temp = tempbuf.t2;
		DataToSend.status = ErrBuff;
		if (CycleNb % 10 == 0)
		{
			xQueueSend(qToLoraHandle, &DataToSend, 2);
		}

		// ------------------ some things -------------------------------------------
		CycleNb++;
		for (int i=0; i<16; i++)
		{
			if ((ErrBuff & (1 << i)) != 0)
			{
				SDBufLen+= sprintf((char*)SDBuffer, "%s,", error_strings[i]);
				WriteToSD(SDBuffer, SDBufLen);
				memset(SDBuffer, 0, sizeof(SDBuffer));
				SDBufLen=0;
			}
		}
		WriteToSD((uint8_t *)"\r\n", 2);
		HAL_GPIO_TogglePin(D0_GPIO_Port, D0_Pin);
		HAL_GPIO_TogglePin(D1_GPIO_Port, D1_Pin);
		HAL_GPIO_TogglePin(D2_GPIO_Port, D2_Pin);
		HAL_GPIO_TogglePin(D3_GPIO_Port, D3_Pin);
		HAL_GPIO_TogglePin(D4_GPIO_Port, D4_Pin);
		HAL_GPIO_TogglePin(D5_GPIO_Port, D5_Pin);
		osDelayUntil(&tick, 1000);
	}
  /* USER CODE END 5 */ 
}

/* StartLoraTask function */
void StartLoraTask(void const * argument)
{
  /* USER CODE BEGIN StartLoraTask */
	Lora_Init();
	uint32_t tick = osKernelSysTick();
	loraUPframe_t DataToSend;
	loraDNframe_t ReceivedFrame;
	uint8_t ReceivedNbOfBytes;
	uint8_t Payload[8];
	uint8_t FifoCurrRxAddr;
	uint8_t ReceivedData[50];
	//osDelay(5000);
	/* Infinite loop */
	for (;;)
	{
		tick = osKernelSysTick();
		if (xQueueReceive(qToLoraHandle, &DataToSend, 0))
		{
			Payload[0] = (DataToSend.alt >> 8);
			Payload[1] = (DataToSend.alt & 0xFF);
			Payload[2] = (DataToSend.temp >> 8);
			Payload[3] = (DataToSend.temp & 0xFF);
			Payload[4] = ReceivedFrame.dest;
			Payload[5] = ReceivedFrame.command;
			Payload[6] = (DataToSend.status >> 8);
			Payload[7] = (DataToSend.status & 0xFF);
			LoraWANTransmitByte(Payload, 8);
			ReceivedNbOfBytes = LoraReceive();
			if (ReceivedNbOfBytes)
			{
				FifoCurrRxAddr = readLoraRegister(0x10);
				writeLoraRegister(0x0D, FifoCurrRxAddr);
				memset(ReceivedData, 0, sizeof(ReceivedData));
				for (int i = 0; i < ReceivedNbOfBytes; i++)
				{
					ReceivedData[i] = readLoraRegister(0x00);
				}
				if (LoraWANParseDN(ReceivedData, ReceivedNbOfBytes))
				{
					ReceivedFrame.dest = ReceivedData[9];
					ReceivedFrame.command = ReceivedData[10];
					if (ReceivedFrame.command == 0x01)
					{
						//HAL_GPIO_TogglePin(D1_GPIO_Port, D1_Pin);
					}
					else if (ReceivedFrame.command == 0x02)
					{
						//HAL_GPIO_TogglePin(D2_GPIO_Port, D2_Pin);
					}
				}
				else
				{
					ReceivedFrame.dest = 0xFF;
					ReceivedFrame.command = 0xFF;
				}
			}
			else
			{
				ReceivedFrame.dest = 0xFF;
				ReceivedFrame.command = 0xFF;
			}
		}
		osDelayUntil(&tick, 1000);
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
	ds18b20_t DS18B20Temp;
	uint32_t BeginTick = 0;
	/* Infinite loop */
	for (;;)
	{
		BeginTick = osKernelSysTick();
		DS18B20ConvertTemperature();
		DS18B20Temp.t1 = ReadDS18B20(DSAddr[0]);
		DS18B20Temp.t2 = ReadDS18B20(DSAddr[1]);
		DS18B20Temp.t3 = ReadDS18B20(DSAddr[2]);
		DS18B20Temp.t4 = ReadDS18B20(DSAddr[3]);
		DS18B20Temp.t5 = ReadDS18B20(DSAddr[4]);
		DS18B20Temp.t6 = ReadDS18B20(DSAddr[5]);
		DS18B20Temp.t7 = ReadDS18B20(DSAddr[6]);
		xQueueSend(qFromDS18Handle, &DS18B20Temp, 10);
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
  if (htim->Instance == TIM2) {
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
