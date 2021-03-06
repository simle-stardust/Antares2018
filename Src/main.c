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
  * Copyright (c) 2020 STMicroelectronics International N.V. 
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

#define GPS_RXBUF_SIZE    256

#define PressureSensorAddr (0x28 << 1)
#define DS3231Addr (0b1101000 << 1)
#define GeigerAddr (0x08 << 1)
#define MAX30205Addr (0b1001000 << 1)
#define HDC1080Addr (0b1000000 << 1)
#define INA3221Addr (0b1000010 << 1)

#define L3GD20HAddr     (0b1101011 << 1)	//gyro
#define LSM303DAddr  	(0b0011101 << 1)	//acc and magn
#define LPS331APAddr  	(0b1011101 << 1)	//baro
#define LEDDRIVERAddr   (0b0100000 << 1)

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

typedef struct loraUPframe_t_
{
	uint16_t status;
	uint16_t statusKom;
	int32_t alt;
	uint32_t lat;
	uint32_t lon;
	uint16_t hdop;
} loraUPframe_t;

typedef struct loraTempInfo_t_
{
	int16_t mean;
	uint8_t highByte;
	uint8_t middleByte;
	uint8_t lowByte;
	uint16_t statusKom;
} loraTempInfo_t;

typedef struct loraDNframe_t_
{
	uint8_t dest;
	uint8_t command;
} loraDNframe_t;

typedef struct ds18b20_t_
{
	int16_t t1;
	int16_t t2;
	int16_t t3;
} ds18b20_t;

typedef struct gps_data_t_
{
	int32_t altitude;
	uint32_t lon;
	uint32_t lat;
	char czasGPS[10];
	uint8_t status;
	char LiterkaLat;
	char LiterkaLon;
	uint16_t hdop;
} gps_data_t;

typedef struct wifi_set_t_
{
	uint8_t h;
	uint8_t m;
	uint8_t s;
	int16_t ds18[3];
	uint16_t hum;
	uint16_t press;
	uint32_t lat;
	uint32_t lon;
	int32_t alt;
	uint16_t hdop;
	uint16_t status;
} wifi_set_t;

typedef struct wifi_get_t_
{
	int16_t RTDTemp[12];
	uint16_t statusKom;
} wifi_get_t;

#define FREQUENCY  868100000lu // in Mhz! (868.1)

#define DEV_ADDR                              (uint32_t)0x260112BA
// network session key
static const uint8_t NwkSKey[16] =
{ 0x47, 0xB8, 0xCE, 0xE5, 0xA2, 0x82, 0xBC, 0xB4, 0x73, 0xA0, 0xC1, 0xEF, 0x5C, 0xF3, 0x01, 0x6A };

// application session key
static const uint8_t ArtSKey[16] =
{ 0x4F, 0xE9, 0xA8, 0x61, 0x02, 0x33, 0xA6, 0xA4, 0x7C, 0x87, 0x0D, 0xC4, 0x9C, 0x10, 0x37, 0xF1 };

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

osThreadId CommandTaskHandle;
osThreadId LoraTaskHandle;
osThreadId WIFITaskHandle;
osThreadId WatchdogTaskHandle;
osThreadId DS18TaskHandle;
osThreadId GpsParserTaskHandle;
osMessageQId qFromGPSHandle;
osMessageQId qToLoraHandle;
osMessageQId qFromLoraHandle;
osMessageQId qToWatchdogHandle;
osMessageQId qToWifiHandle;
osMessageQId qFromWifiHandle;
osMessageQId qFromDS18Handle;
osMessageQId qToWifiSetValHandle;
osMessageQId qFromWifiErrHandle;
osSemaphoreId GpsBinarySemHandle;
osSemaphoreId WiFiTxDoneHandle;
osSemaphoreId WiFiRxDoneHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS my_fatfs;
FIL my_file;
UINT my_error;

uint8_t GpsRxBuf[GPS_RXBUF_SIZE];

static const uint64_t DSAddr[3] =
{ 0x28C6F3400C000080, 0x2856D4400C000098, 0x288DEF400C000050};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM7_Init(void);
static void MX_UART4_Init(void);
void StartCommandTask(void const * argument);
void StartLoraTask(void const * argument);
void StartWIFITask(void const * argument);
void StartWatchdogTask(void const * argument);
void StartDS18Task(void const * argument);
void StartGpsParserTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void UART_Init();
static inline uint8_t setBit(uint8_t value, uint8_t bit);
static inline uint8_t clearBit(uint8_t value, uint8_t bit);
static uint8_t writeLoraRegister(uint8_t address, uint8_t data);
static uint8_t readLoraRegister(uint8_t address, uint8_t* data);
static void Lora_Init();
static uint8_t LoraReceive();
static uint8_t LoraTransmitByte(uint8_t *data, uint8_t size);
static gps_data_t GpsParse(const char* data);
void LoRa_SendToGround(loraUPframe_t* data, loraTempInfo_t* tempInfo, uint16_t recvFrame);


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)
	{
		xSemaphoreGiveFromISR(WiFiRxDoneHandle, NULL);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)
	{
		xSemaphoreGiveFromISR(WiFiTxDoneHandle, NULL);
	}
}


void USART2_IRQHandler()
{
	if ((USART2->SR & USART_SR_IDLE) && (USART2->CR1 & USART_CR1_IDLEIE))
	{
		// clear by reading dr
		volatile uint32_t tmpreg;
		tmpreg = USART2->SR;
		(void) tmpreg;
		tmpreg = USART2->DR;
		(void) tmpreg;
		xSemaphoreGiveFromISR(GpsBinarySemHandle, NULL);
	}
}

void DMA1_Channel6_IRQHandler()
{
	//clear interrupt flag
	if (((DMA1->ISR) & (DMA_ISR_TCIF6)))
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF6;
		xSemaphoreGiveFromISR(GpsBinarySemHandle, NULL);
	}
	if (((DMA1->ISR) & (DMA_ISR_HTIF6)))
	{
		DMA1->IFCR |= DMA_IFCR_CHTIF6;
		xSemaphoreGiveFromISR(GpsBinarySemHandle, NULL);
	}
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_TIM7_Init();
  MX_UART4_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of GpsBinarySem */
  osSemaphoreDef(GpsBinarySem);
  GpsBinarySemHandle = osSemaphoreCreate(osSemaphore(GpsBinarySem), 1);

  /* definition and creation of WiFiTxDone */
  osSemaphoreDef(WiFiTxDone);
  WiFiTxDoneHandle = osSemaphoreCreate(osSemaphore(WiFiTxDone), 1);

  /* definition and creation of WiFiRxDone */
  osSemaphoreDef(WiFiRxDone);
  WiFiRxDoneHandle = osSemaphoreCreate(osSemaphore(WiFiRxDone), 1);

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
  osThreadDef(WIFITask, StartWIFITask, osPriorityNormal, 0, 512);
  WIFITaskHandle = osThreadCreate(osThread(WIFITask), NULL);

  /* definition and creation of WatchdogTask */
  osThreadDef(WatchdogTask, StartWatchdogTask, osPriorityNormal, 0, 128);
  WatchdogTaskHandle = osThreadCreate(osThread(WatchdogTask), NULL);

  /* definition and creation of DS18Task */
  osThreadDef(DS18Task, StartDS18Task, osPriorityNormal, 0, 128);
  DS18TaskHandle = osThreadCreate(osThread(DS18Task), NULL);

  /* definition and creation of GpsParserTask */
  osThreadDef(GpsParserTask, StartGpsParserTask, osPriorityNormal, 0, 512);
  GpsParserTaskHandle = osThreadCreate(osThread(GpsParserTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of qFromGPS */
  osMessageQDef(qFromGPS, 2, gps_data_t);
  qFromGPSHandle = osMessageCreate(osMessageQ(qFromGPS), NULL);

  /* definition and creation of qToLora */
  osMessageQDef(qToLora, 1, loraUPframe_t);
  qToLoraHandle = osMessageCreate(osMessageQ(qToLora), NULL);

  /* definition and creation of qFromLora */
  osMessageQDef(qFromLora, 1, uint16_t);
  qFromLoraHandle = osMessageCreate(osMessageQ(qFromLora), NULL);

  /* definition and creation of qToWatchdog */
  osMessageQDef(qToWatchdog, 5, uint8_t);
  qToWatchdogHandle = osMessageCreate(osMessageQ(qToWatchdog), NULL);

  /* definition and creation of qToWifi */
  osMessageQDef(qToWifi, 1, uint8_t);
  qToWifiHandle = osMessageCreate(osMessageQ(qToWifi), NULL);

  /* definition and creation of qFromWifi */
  osMessageQDef(qFromWifi, 1, loraTempInfo_t);
  qFromWifiHandle = osMessageCreate(osMessageQ(qFromWifi), NULL);

  /* definition and creation of qFromDS18 */
  osMessageQDef(qFromDS18, 2, ds18b20_t);
  qFromDS18Handle = osMessageCreate(osMessageQ(qFromDS18), NULL);

  /* definition and creation of qToWifiSetVal */
  osMessageQDef(qToWifiSetVal, 1, wifi_set_t);
  qToWifiSetValHandle = osMessageCreate(osMessageQ(qToWifiSetVal), NULL);

  /* definition and creation of qFromWifiErr */
  osMessageQDef(qFromWifiErr, 1, uint8_t);
  qFromWifiErrHandle = osMessageCreate(osMessageQ(qFromWifiErr), NULL);

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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
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

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, D0_Pin|D1_Pin|D2_Pin|Power_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, D3_Pin|D4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Thermal_Signal_Internal_Pin|OneWire_PULLUP_Pin|LoRa_RST_Pin|LoRa_TX_Pin 
                          |Thermal_Signal_Camera_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LoRa_RX_Pin|Thermal_Signal_Pin|Buzzer_Signal_Pin|Wifi_RST_Pin 
                          |Wifi_GPIO0_Pin|Wifi_GPIO2_Pin|Wifi_GPIO4_Pin|Wifi_GPIO5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OneWire_GPIO_Port, OneWire_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D0_Pin D1_Pin D2_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D3_Pin D4_Pin */
  GPIO_InitStruct.Pin = D3_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : Thermal_Signal_Internal_Pin OneWire_PULLUP_Pin LoRa_TX_Pin Thermal_Signal_Camera_Pin 
                           Power_LED_Pin */
  GPIO_InitStruct.Pin = Thermal_Signal_Internal_Pin|OneWire_PULLUP_Pin|LoRa_TX_Pin|Thermal_Signal_Camera_Pin 
                          |Power_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LoRa_CS_Pin Buzzer_Pin */
  GPIO_InitStruct.Pin = LoRa_CS_Pin|Buzzer_Pin;
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

  /*Configure GPIO pins : LoRa_RX_Pin Thermal_Signal_Pin Buzzer_Signal_Pin Wifi_RST_Pin 
                           Wifi_GPIO0_Pin Wifi_GPIO2_Pin Wifi_GPIO4_Pin Wifi_GPIO5_Pin */
  GPIO_InitStruct.Pin = LoRa_RX_Pin|Thermal_Signal_Pin|Buzzer_Signal_Pin|Wifi_RST_Pin 
                          |Wifi_GPIO0_Pin|Wifi_GPIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Wifi_GPIO4_Pin|Wifi_GPIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OneWire_Pin */
  GPIO_InitStruct.Pin = OneWire_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(OneWire_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t ReadADC(uint8_t adcnum)
{
	ADC_ChannelConfTypeDef adc_ch;

	switch (adcnum)
	{
	case 0:
		//ADC ele
		adc_ch.Channel = ADC_CHANNEL_19;
		break;
	case 1:
		adc_ch.Channel = ADC_CHANNEL_1;
		break;
	default:
		adc_ch.Channel = ADC_CHANNEL_19;
		break;
	}

	adc_ch.Rank = ADC_REGULAR_RANK_1;
	adc_ch.SamplingTime = ADC_SAMPLETIME_24CYCLES;
	HAL_ADC_ConfigChannel(&hadc, &adc_ch);
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	return HAL_ADC_GetValue(&hadc);

	return 0;
}

static void UART_Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	//clock enable for UART2
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;  //USART2 RX uses DMA1 channel6
	//alternate functions of PA2 and PA3 set to UART2 AF=7
	GPIOA->AFR[0] |= (0x00007700);
	GPIOA->MODER |= (0x000000A0);
	GPIOA->OSPEEDR |= (0x000000F0);
	GPIOA->PUPDR |= (0x00000050);

	NVIC_SetPriority(USART2_IRQn, 0x05);
	NVIC_EnableIRQ(USART2_IRQn);			// set enable IRQ

	NVIC_SetPriority(DMA1_Channel6_IRQn, 0x05);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	USART2->BRR = 3333;						//baud rate = 9600

	DMA1_Channel6->CMAR = (uint32_t) GpsRxBuf;
	DMA1_Channel6->CPAR = (uint32_t) &(USART2->DR);
	DMA1_Channel6->CNDTR = GPS_RXBUF_SIZE;
	DMA1_Channel6->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_EN | DMA_CCR_HTIE | DMA_CCR_TCIE;

	USART2->CR3 |= USART_CR3_DMAR;
	USART2->CR1 |= USART_CR1_UE | USART_CR1_PEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_IDLEIE;
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

static uint8_t writeLoraRegister(uint8_t address, uint8_t data)
{
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	address = setBit(address, 7);
	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
	if (HAL_SPI_Transmit(&hspi1, &data, 1, 100) != HAL_OK)
	{
		HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
		return 0x01;
	}
	else
	{
		HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
		return 0x00;
	}
}

static uint8_t readLoraRegister(uint8_t address, uint8_t* data)
{
	HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_RESET);
	address = clearBit(address, 7);
	HAL_SPI_Transmit(&hspi1, &address, 1, 100);
	if (HAL_SPI_Receive(&hspi1, data, 1, 100) != HAL_OK)
	{
		return 0x01;
	}
	else
	{
		HAL_GPIO_WritePin(LoRa_CS_GPIO_Port, LoRa_CS_Pin, GPIO_PIN_SET);
		return 0x00;
	}
}

static void Lora_Init()
{
	uint64_t frf = ((uint64_t) FREQUENCY << 19) / 32000000;

	HAL_GPIO_WritePin(LoRa_RST_GPIO_Port, LoRa_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(LoRa_RST_GPIO_Port, LoRa_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);

	//Opmode - sleep -> lora -> standby
	writeLoraRegister(0x01, 0x00);
	writeLoraRegister(0x01, 0x80);
	writeLoraRegister(0x01, 0x81);

	// Max Payload length = 255
	writeLoraRegister(0x23, 255);

	// LNA gain = maximum, LNA boot = yes
	writeLoraRegister(0x0C, 0x23);

	// Bandwidth = 125kHz, CR 4/5, Explicit Header Mode, CRC enable, Low Data Rate Optimization on
	writeLoraRegister(0x1D, 0x0B);

	// Spreading Factor 12, LNA gain set by the internal AGC loop, RX timeout MSB = 0b11
	writeLoraRegister(0x1E, 0xC4);   //bylo C5
	// RX timeout LSB = 0xFF
	writeLoraRegister(0x1F, 0x1E);   //bylo A0

	// Sync Word = 0x34 (LoRaWAN sync word)
	writeLoraRegister(0x39, 0x34);

	//868.1 MHz - LoRaWAN channel 1
	writeLoraRegister(0x06, (uint8_t) (frf >> 16));
	writeLoraRegister(0x07, (uint8_t) (frf >> 8));
	writeLoraRegister(0x08, (uint8_t) (frf >> 0));

	// max output power, no PA_BOOST
	writeLoraRegister(0x09, 0x8F);
}

static uint8_t LoraReceive()
{
	uint8_t RxDoneInterrupt;
	uint8_t NbofBytes;
	uint8_t RxTimeoutInterrupt;
	HAL_GPIO_WritePin(LoRa_RX_GPIO_Port, LoRa_RX_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LoRa_TX_GPIO_Port, LoRa_TX_Pin, GPIO_PIN_RESET);
	writeLoraRegister(0x11, 0xFF);
	writeLoraRegister(0x12, 0b11111111);
	writeLoraRegister(0x11, 0x00);
	writeLoraRegister(0x0D, 0x00);
	readLoraRegister(0x12, &RxDoneInterrupt);
	RxTimeoutInterrupt = RxDoneInterrupt;
	RxDoneInterrupt &= (1 << 6);
	RxTimeoutInterrupt &= (1 << 7);
	writeLoraRegister(0x01, 0x86);
	while (RxDoneInterrupt == 0 && RxTimeoutInterrupt == 0)
	{
		if (readLoraRegister(0x12, &RxDoneInterrupt) != 0x00)
		{
			return 0x00;
		}
		else
		{
			RxTimeoutInterrupt = RxDoneInterrupt;
			RxDoneInterrupt &= (1 << 6);
			RxTimeoutInterrupt &= (1 << 7);
		}
	}
	readLoraRegister(0x13, &NbofBytes);
	if (RxTimeoutInterrupt > 0)
		NbofBytes = 0;
	return NbofBytes;
}

static uint8_t LoraTransmitByte(uint8_t *data, uint8_t size)
{
	uint8_t TxDoneInterrupt;
	HAL_GPIO_WritePin(LoRa_TX_GPIO_Port, LoRa_TX_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LoRa_RX_GPIO_Port, LoRa_RX_Pin, GPIO_PIN_RESET);
	writeLoraRegister(0x22, size);
	writeLoraRegister(0x11, 0xFF);
	writeLoraRegister(0x12, 0b11111111);
	writeLoraRegister(0x11, 0x00);
	readLoraRegister(0x12, &TxDoneInterrupt);
	writeLoraRegister(0x0D, 0x80);
	for (int i = 0; i < size; i++)
	{
		writeLoraRegister(0x00, *data);
		data++;
	}
	writeLoraRegister(0x01, 0x83);
	readLoraRegister(0x12, &TxDoneInterrupt);
	TxDoneInterrupt &= (1 << 3);
	while ((TxDoneInterrupt == 0))
	{
		if (readLoraRegister(0x12, &TxDoneInterrupt) != 0x00)
		{
			return 0x01;
		}
		else
		{
			TxDoneInterrupt &= (1 << 3);
		}
	}
	return 0x00;
}

static uint8_t LoraWANTransmitByte(uint8_t* data, uint8_t size)
{
	uint8_t UplinkFrame[32];
	static uint32_t licznik;
	for (int i = 0; i < size; i++)
	{
		UplinkFrame[9 + i] = data[i];
	}
	UplinkFrame[0] = 0x40; //data unconfirmed up
	UplinkFrame[1] = (uint8_t) (DEV_ADDR & 0xFF);
	UplinkFrame[2] = (uint8_t) (DEV_ADDR >> 8);
	UplinkFrame[3] = (uint8_t) (DEV_ADDR >> 16);
	UplinkFrame[4] = (uint8_t) (DEV_ADDR >> 24);
	UplinkFrame[5] = 0x00; //FCtrl
	UplinkFrame[6] = (uint8_t) (licznik & 0xFF); //Fcnt lsb
	UplinkFrame[7] = (uint8_t) (licznik >> 8); //Fcnt msb
	UplinkFrame[8] = 0x01; //port
	aes_cipher(ArtSKey, DEV_ADDR, licznik, /*up*/0, &UplinkFrame[9], size);
	aes_appendMic(NwkSKey, DEV_ADDR, licznik, /*up*/0, UplinkFrame, size + 9);
	if (LoraTransmitByte(UplinkFrame, size + 13) == 0)
	{
		licznik++;
		return 0x00;
	}
	else
	{
		return 0x01;
	}
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
	//HAL_GPIO_WritePin(OneWire_PULLUP_GPIO_Port, OneWire_PULLUP_Pin,
	//		GPIO_PIN_RESET);
	oneWireResetPulse();
	oneWireSendByte(0xCC);
	oneWireSendByte(0x44);
	__enable_irq();
	osDelay(750);
	//HAL_GPIO_WritePin(OneWire_PULLUP_GPIO_Port, OneWire_PULLUP_Pin,
	//		GPIO_PIN_SET);
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
		iTemperatura = 4040;
	return iTemperatura;
}

void LoRa_SendToGround(loraUPframe_t* data, loraTempInfo_t* tempInfo, uint16_t recvFrame)
{
	uint8_t Payload[23];
	Payload[0] = (data->alt >> 24);
	Payload[1] = (data->alt >> 16);
	Payload[2] = (data->alt >> 8);
	Payload[3] = (data->alt & 0xFF);
	Payload[4] = (data->lat >> 24);
	Payload[5] = (data->lat >> 16);
	Payload[6] = (data->lat >> 8);
	Payload[7] = (data->lat & 0xFF);
	Payload[8] = (data->lon >> 24);
	Payload[9] = (data->lon >> 16);
	Payload[10] = (data->lon >> 8);
	Payload[11] = (data->lon & 0xFF);
	Payload[12] = (tempInfo->mean >> 8);
	Payload[13] = (tempInfo->mean & 0xFF);
	Payload[14] = tempInfo->highByte;
	Payload[15] = tempInfo->middleByte;
	Payload[16] = tempInfo->lowByte;
	Payload[17] = (uint8_t) (recvFrame >> 8);
	Payload[18] = (uint8_t) (recvFrame & 0xFF);
	Payload[19] = (data->status >> 8);
	Payload[20] = (data->status & 0xFF);
	Payload[21] = (data->statusKom >> 8);
	Payload[22] = (data->statusKom & 0xFF);
	Payload[23] = (data->hdop >> 8);
	Payload[24] = (data->hdop & 0xFF);
	LoraWANTransmitByte(Payload, 25);
}

static uint16_t WriteToSD(uint8_t* buf, uint8_t len)
{
	char my_file_name[20];
	static uint8_t datalogNum = 0;
	uint16_t returnCode = 0;
	FRESULT fresult;
	uint32_t fileSize;

	memset(my_file_name, ' ', sizeof(my_file_name));
	sprintf(my_file_name, "datalog%d.txt", datalogNum);
	__disable_irq();
	fresult = f_open(&my_file, my_file_name, FA_WRITE | FA_OPEN_ALWAYS);
	fileSize = f_size(&my_file);
	fresult = f_close(&my_file);
	if (fileSize > 1000000)
	{
		datalogNum++;
		sprintf(my_file_name, "datalog%d.txt", datalogNum);
		// reset file object
		memset(&my_file, 0x00, sizeof(my_file));
		fileSize  = 0;
	}
	__enable_irq();
	__disable_irq();
	fresult = f_open(&my_file, my_file_name, FA_WRITE | FA_OPEN_ALWAYS);
	if (fresult == FR_OK)
	{
		fresult = f_lseek(&my_file, fileSize);
		fresult = f_write(&my_file, buf, len, &my_error);
		if (fresult == FR_OK)
		{
			fresult = f_close(&my_file);
			if (fresult != FR_OK)
			{
				returnCode = SD_ERR;
			}
		}
		else
		{
			returnCode = SD_ERR;
		}
	}
	else
	{
		returnCode = SD_ERR;
	}
	__enable_irq();
	osDelay(10);
	return returnCode;
}

static uint16_t ReadI2CSensors(uint8_t* buf)
{
	uint8_t i2cData[3];
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
		//calc = (((i2cData[0] << 8) + i2cData[1]) - 1638) * 15 * 100 / 13107;
		//buf[3] = (uint8_t) (calc >> 8);
		//buf[4] = (uint8_t) (calc & 0xFF);
		buf[3] = i2cData[0];
		buf[4] = i2cData[1];
	}
	else
	{
		returnCode |= I2C_PRESSURE_ERR;
	}

	return returnCode;
}

static gps_data_t GpsCheckAndProcess()
{
	static size_t old_pos;
	char GpsArray[GPS_RXBUF_SIZE];
	size_t pos;
	gps_data_t ret;
	/* Calculate current position in buffer */
	pos = ARRAY_LEN(GpsRxBuf) - DMA1_Channel5->CNDTR;
	if (pos != old_pos)
	{ /* Check change in received data */
		if (pos > old_pos)
		{ /* Current position is over previous one */
			/* We are in "linear" mode */
			/* Process data directly by subtracting "pointers" */
			memcpy(GpsArray, &GpsRxBuf[old_pos], pos - old_pos);
			//usart_process_data(&GpsRxBuf[old_pos], pos - old_pos);
		}
		else
		{
			/* We are in "overflow" mode */
			/* First process data to the end of buffer */
			memcpy(GpsArray, &GpsRxBuf[old_pos], ARRAY_LEN(GpsRxBuf) - old_pos);
			//usart_process_data(&GpsRxBuf[old_pos],
			//ARRAY_LEN(GpsRxBuf) - old_pos);
			/* Check and continue with beginning of buffer */
			if (pos)
			{
				memcpy(&GpsArray[pos], &GpsRxBuf[0], pos);
				//usart_process_data(&GpsRxBuf[0], pos);
			}
		}
	}
	old_pos = pos; /* Save current position as old */
	/* Check and manually update if we reached end of buffer */
	if (old_pos == ARRAY_LEN(GpsRxBuf))
	{
		old_pos = 0;
	}
	ret = GpsParse(GpsArray);
	memset(GpsArray, 0, sizeof(GpsArray));
	return ret;
}

static gps_data_t GpsParse(const char* data)
{
	static gps_data_t returnVal =
	{ 0, 0, 0,
	{ '0', '0', '0', '0', '0', '0', '.', '0', '0', '\0' }, 0x01, 'N', 'E', 0xFFFF };
	gps_data_t returnBuf =
	{ 0, 0, 0,
	{ '0', '0', '0', '0', '0', '0', '.', '0', '0', '\0' }, 0x01, 'N', 'E', 0xFFFF };;
	char* ptr;
	uint8_t GPSReadChecksum = 0;
	uint8_t GPSCalcChecksum = 1;
	char *schowek;
	ptr = strstr(data, "$GPGGA");
	if (ptr == NULL)
	{
		returnVal.status = 0x02;
		return returnVal;
	}
	GPSCalcChecksum = ptr[1];
	for (int i = 2; i <= GPS_RXBUF_SIZE; ++i)
	{
		if (ptr[i] == 0x2A)
			break;
		else
			GPSCalcChecksum ^= ptr[i];
	}
	strtok(&ptr[0], ",");
	schowek = strtok(NULL, ",");
	strcpy(returnBuf.czasGPS, schowek);
	schowek = strtok(NULL, ",");
	returnBuf.lat = (uint32_t) (strtof(schowek, NULL) * 100000);
	schowek = strtok(NULL, ",");
	returnBuf.LiterkaLat = *schowek;
	schowek = strtok(NULL, ",");
	returnBuf.lon = (uint32_t) (strtof(schowek, NULL) * 100000);
	schowek = strtok(NULL, ",");
	returnBuf.LiterkaLon = *schowek;
	schowek = strtok(NULL, ",");
	schowek = strtok(NULL, ",");
	schowek = strtok(NULL, ",");
	returnBuf.hdop = (int16_t) (strtol(schowek, NULL, 10) * 10.0f);
	schowek = strtok(NULL, ",");
	returnBuf.altitude = (int32_t) strtol(schowek, NULL, 10);
	schowek = strtok(NULL, "*");
	schowek = strtok(NULL, "\r");
	GPSReadChecksum = strtol(schowek, NULL, 16);
	returnBuf.czasGPS[10] = '\0';
	if (GPSCalcChecksum != GPSReadChecksum)
	{
		returnVal.hdop = 0xFF;
		returnVal.status = 0x01;
	}
	else
	{
		returnVal = returnBuf;
		returnVal.status = 0x00;
	}
	return returnVal;
}

/* USER CODE END 4 */

/* StartCommandTask function */
void StartCommandTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN 5 */
	const char initMessage[] = "\r\nRESET\r\n";
	gps_data_t gpsData =
	{ 0, 0, 0,
	{ '0', '0', '0', '0', '0', '0', '.', '0', '0', '\0' }, 0x01, 'N', 'E', 0xFFFF };
	wifi_set_t wifiData =
	{ 0x00, 0x00, 0x00,
	{ 0x0000, 0x0000, 0x0000}, 0x00, 0x00,
			0x00000000, 0x00000000, 0x00000000, 0x0000 };
	uint8_t dataToWatchdog = 0;
	uint8_t odcinaczFlag = 0;
	uint16_t LoraDnMsg = 0xFFFF;
	uint32_t tick = osKernelSysTick();
	uint8_t SDBuffer[64];
	uint8_t SDBufLen = 0;
	uint8_t RawDataBuffer[20];
	uint8_t PowerLEDFlagActivate = 0;
	uint8_t DNFlag = 0;
	loraUPframe_t DataToSend;
	uint16_t ErrBuff = 0;
	ds18b20_t tempbuf;
	uint8_t wifiStatus = 0;
	uint32_t CycleNb = 0;

	uint16_t ADC_ele = 0;
	uint16_t ADC_lora = 0;
	uint16_t ADC_thermal = 0;

	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);

	f_mount(&my_fatfs, SD_Path, 1);
	WriteToSD((uint8_t*) initMessage, sizeof(initMessage));
	memset(SDBuffer, 0, sizeof(SDBuffer));
/*
 *  Set RTC
	RawDataBuffer[0] = 0x00;
	RawDataBuffer[1] = 0x46;
	RawDataBuffer[2] = 0x21;
	if (HAL_I2C_Mem_Write(&hi2c2, DS3231Addr, 0x00, 1, RawDataBuffer, 3, 100)
			== HAL_OK)
	{

	}
*/

	/* Infinite loop */
	for (;;)
	{
		// flags to clear with each iteration
		ErrBuff &= ~(GPS_ERR | DS18_ERR);
		tick = osKernelSysTick();
		// --------------- I2C Sensors Readout -------------------------------
		ErrBuff |= ReadI2CSensors(RawDataBuffer);
		SDBufLen += sprintf((char*) SDBuffer, "%d:%d:%d,%d,", RawDataBuffer[0],
				RawDataBuffer[1], RawDataBuffer[2],
				(RawDataBuffer[3] << 8) + RawDataBuffer[4]);
		WriteToSD(SDBuffer, SDBufLen);
		wifiData.h = RawDataBuffer[0];
		wifiData.m = RawDataBuffer[1];
		wifiData.s = RawDataBuffer[2];
		wifiData.press = (RawDataBuffer[3] << 8) + RawDataBuffer[4];
		wifiData.hum = 0x0000;
		memset(SDBuffer, 0, sizeof(SDBuffer));
		SDBufLen = 0;

		// read ADCs
		ADC_ele = ReadADC(0);
		ADC_thermal = ReadADC(1);

		SDBufLen = sprintf((char*) SDBuffer, "%u,%u,%u,",
				ADC_ele, ADC_lora, ADC_thermal);
		ErrBuff |= WriteToSD(SDBuffer, SDBufLen);
		memset(SDBuffer, 0, sizeof(SDBuffer));
		SDBufLen = 0;

		// --------------- DS18B20 Readout ------------------------------------
		xQueueReceive(qFromDS18Handle, &tempbuf, 0);
		SDBufLen += sprintf((char*) SDBuffer, "%d,%d,%d,",
				tempbuf.t1, tempbuf.t2, tempbuf.t3);
		ErrBuff |= WriteToSD(SDBuffer, SDBufLen);
		if ((tempbuf.t1 == 4040) || (tempbuf.t2 == 4040)
				|| (tempbuf.t3 == 4040))
		{
			ErrBuff |= DS18_ERR;
		}
		wifiData.ds18[0] = tempbuf.t1;
		wifiData.ds18[1] = tempbuf.t2;
		wifiData.ds18[2] = tempbuf.t3;
		memset(SDBuffer, 0, sizeof(SDBuffer));
		SDBufLen = 0;
		// ---------------- GPS Readout ----------------------------------------

		if (xQueueReceive(qFromGPSHandle, &gpsData, 5) == pdTRUE)
		{
			// received GPS data
			gpsData.status = 0x00;
		}
		else
		{
			gpsData.status = 0x01;
			ErrBuff |= GPS_ERR;
		}
		xQueueReset(qFromGPSHandle);

		wifiData.alt = gpsData.altitude;
		wifiData.lat = gpsData.lat;
		wifiData.lon = gpsData.lon;
		wifiData.hdop = gpsData.hdop;
		SDBufLen = sprintf((char*) SDBuffer, "%s,%lu.%lu %c,%lu.%lu %c,%ld,%u, ",
				gpsData.czasGPS, (gpsData.lon / 100000), (gpsData.lon % 100000),
				gpsData.LiterkaLon, (gpsData.lat / 100000),
				(gpsData.lat % 100000), gpsData.LiterkaLat, gpsData.altitude, gpsData.hdop);
		WriteToSD(SDBuffer, SDBufLen);

		memset(SDBuffer, 0, sizeof(SDBuffer));
		SDBufLen = 0;
		//----------------- LORA Transmission ----------------------------------------
		DataToSend.alt = gpsData.altitude;
		DataToSend.lon = gpsData.lon;
		DataToSend.lat = gpsData.lat;
		DataToSend.hdop = gpsData.hdop;
		DataToSend.status = ErrBuff;
		DataToSend.statusKom = 0x0000;
		if (CycleNb % 200 == 0)
		{
			xQueueSend(qToLoraHandle, &DataToSend, 2);
		}
		//----------------- LORA Reception -------------------------------------
		if (xQueueReceive(qFromLoraHandle, &LoraDnMsg, 5) == pdTRUE)
		{
			DNFlag = 1;
			switch (LoraDnMsg)
			{
				case 0xFFFF:
					// this can happen when we receive alien frame
					// ignore
					//ErrBuff |= LORA_ERR;
					//HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
					//HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);
					break;
				case 0xFFFE:
					ErrBuff |= LORA_ERR;
					HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);
					break;
				case 0x0005:
					ErrBuff &= ~LORA_ERR;
					ErrBuff &= ~ODCINACZ_FLAG;
					odcinaczFlag = 0;
					HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
					xQueueSend(qToWifiHandle, &odcinaczFlag, 0);
					break;
				case 0x0002:
					ErrBuff &= ~LORA_ERR;
					ErrBuff |= ODCINACZ_FLAG;
					odcinaczFlag = 1;
					HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
					xQueueSend(qToWifiHandle, &odcinaczFlag, 0);
					break;
				case 0x0003:
				case 0x0004:
					ErrBuff &= ~LORA_ERR;
					//zapal diode
					PowerLEDFlagActivate = 1;
					HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
					break;
				case 0x0001:
					HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
					ErrBuff &= ~LORA_ERR;
					break;
				case 0x0006:
					// Start
					ErrBuff &= ~LORA_ERR;
					ErrBuff |= FLIGHT_IS_ON;
					break;
				case 0x0007:
					// Stop
					ErrBuff &= ~LORA_ERR;
					ErrBuff &= ~FLIGHT_IS_ON;
					break;
				case 0x0010:
					// Set stardust state to 0
					ErrBuff &= ~LORA_ERR;
					ErrBuff &= ~(STATUS_BYTE0|STATUS_BYTE1|STATUS_BYTE2);
					break;
				case 0x0011:
					// Set stardust state to 1
					ErrBuff &= ~LORA_ERR;
					ErrBuff &= ~(STATUS_BYTE0|STATUS_BYTE1|STATUS_BYTE2);
					ErrBuff |= STATUS_BYTE0;
					break;
				case 0x0012:
					// Set stardust state to 2
					ErrBuff &= ~LORA_ERR;
					ErrBuff &= ~(STATUS_BYTE0|STATUS_BYTE1|STATUS_BYTE2);
					ErrBuff |= STATUS_BYTE1;
					break;
				case 0x0013:
					// Set stardust state to 3
					ErrBuff &= ~LORA_ERR;
					ErrBuff &= ~(STATUS_BYTE0|STATUS_BYTE1|STATUS_BYTE2);
					ErrBuff |= STATUS_BYTE0 | STATUS_BYTE1;
					break;
				default:
					break;
			}
		}

		//--------------------WifiReception-------------------------------------------
		if (xQueueReceive(qFromWifiErrHandle, &wifiStatus, 0) == pdTRUE)
		{
			if (wifiStatus)
			{
				ErrBuff |= WIFI_ERR;
			}
			else
			{
				ErrBuff &= ~WIFI_ERR;
			}
		}
		// ------------------ some things -------------------------------------------

		if (DNFlag)
		{
			DNFlag = 0;
			SDBufLen = sprintf((char*) SDBuffer, "Received Downlink: %04x, ", LoraDnMsg);
			WriteToSD(SDBuffer, SDBufLen);
		}

		CycleNb++;

		for (int i = 0; i < 15; i++)
		{
			if ((ErrBuff & (1 << i)) != 0)
			{
				SDBufLen += sprintf((char*) SDBuffer, "%s,", error_strings[i]);
				WriteToSD(SDBuffer, SDBufLen);
				memset(SDBuffer, 0, sizeof(SDBuffer));
				SDBufLen = 0;
			}
		}
		WriteToSD((uint8_t *) "\r\n", 2);
		if (ErrBuff & SD_ERR)
		{
			dataToWatchdog = 0xFF;
			HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
		}
		else
		{
			if (PowerLEDFlagActivate)
			{
				PowerLEDFlagActivate = 0;
				if (LoraDnMsg == 0x0003)
				{
					dataToWatchdog = 1;
					HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
					ErrBuff |= POWER_LED_FLAG;
				}
				else
				{
					dataToWatchdog = 2;
					HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
					ErrBuff &= ~POWER_LED_FLAG;
				}
			}
			else
			{
				dataToWatchdog = 0;
			}
			HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
		}
		xQueueSend(qToWatchdogHandle, &dataToWatchdog, 10);

		wifiData.status = ErrBuff;

		if (CycleNb % 5 == 0)
		{
			xQueueSend(qToWifiSetValHandle, &wifiData, 10);
		}

		ErrBuff ^= RUNNING_FLAG;

		memset(RawDataBuffer, 0, sizeof(RawDataBuffer));
		RawDataBuffer[0] = (uint8_t)(~ErrBuff >> 8);
		RawDataBuffer[1] = (uint8_t)(~ErrBuff & 0xFF);
		HAL_I2C_Master_Transmit(&hi2c2, LEDDRIVERAddr, (uint8_t*) RawDataBuffer, 2, 100);

		HAL_GPIO_TogglePin(D0_GPIO_Port, D0_Pin);
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
	uint32_t send_tick = 0;
	uint32_t rx_tick = 0x8FFFFFFF;
	loraUPframe_t DataToSend;
	uint16_t ReceivedFrame = 0xFFFF;
	loraTempInfo_t tempInfo;
	uint8_t ReceivedNbOfBytes;
	uint8_t FifoCurrRxAddr;
	uint8_t ReceivedData[50];
	
	// wait for queues to be filled
	osDelay(2000);
	memset(&DataToSend, 0x00, sizeof(DataToSend));
	memset(&tempInfo, 0x00, sizeof(tempInfo));
	xQueueReceive(qToLoraHandle, &DataToSend, 0);
	xQueueReceive(qFromWifiHandle, &tempInfo, 0);
	send_tick = osKernelSysTick();
	LoRa_SendToGround(&DataToSend, &tempInfo, ReceivedFrame);

	/* Infinite loop */
	for (;;)
	{
		tick = osKernelSysTick();
		if (tick - send_tick > 200000)
		{
			xQueueReceive(qToLoraHandle, &DataToSend, 0);
			xQueueReceive(qFromWifiHandle, &tempInfo, 0);
			send_tick = osKernelSysTick();
			LoRa_SendToGround(&DataToSend, &tempInfo, ReceivedFrame);
			HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
			osDelay(100);
			HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
		}
		ReceivedNbOfBytes = LoraReceive();
		if (ReceivedNbOfBytes)
		{
			readLoraRegister(0x10, &FifoCurrRxAddr);
			writeLoraRegister(0x0D, FifoCurrRxAddr);
			memset(ReceivedData, 0, sizeof(ReceivedData));
			for (int i = 0; i < ReceivedNbOfBytes; i++)
			{
				readLoraRegister(0x00, &ReceivedData[i]);
			}
			if (LoraWANParseDN(ReceivedData, ReceivedNbOfBytes))
			{
				rx_tick = osKernelSysTick();
				ReceivedFrame = (ReceivedData[9] << 8) | (ReceivedData[10]);
			}
			else
			{
				ReceivedFrame = 0xFFFF;
			}
		}
		else
		{
			tick = osKernelSysTick();
			if (tick - rx_tick > 180000)
			{
				ReceivedFrame = 0xFFFE;
			}
			else
			{
				// rx still not timedout
				ReceivedFrame = 0x0001;
			}
		}
		xQueueSend(qFromLoraHandle, &ReceivedFrame, 5);
		osDelay(10);
	}
  /* USER CODE END StartLoraTask */
}

/* StartWIFITask function */
void StartWIFITask(void const * argument)
{
  /* USER CODE BEGIN StartWIFITask */
	const char CutCommand[] = "@MarcinOdcinaj!";
	const char GetCommand[] = "@MarcinGetValues!";
	const char Ok[] = "@MarcinOK:";
	char TxBuffer[192];
	char RxBuffer[75];
	uint8_t TxBufLen = 0;
	int16_t temps[30];
	uint8_t odcinaczFlag = 0;
	uint8_t error = 0;
	loraTempInfo_t tempInfo;
	uint32_t tick;
	int32_t meanCalc = 0;
	uint32_t tempStatusCalc = 0;
	wifi_set_t setData =
	{ 0x00, 0x00, 0x00,
	{ 0x0000, 0x0000, 0x0000}, 0x00, 0x00,
			0x00000000, 0x00000000, 0x00000000, 0x0000 };
	memset(TxBuffer, 0, sizeof(TxBuffer));
	memset(RxBuffer, 0, sizeof(RxBuffer));

	HAL_GPIO_WritePin(Wifi_RST_GPIO_Port, Wifi_RST_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Wifi_GPIO0_GPIO_Port, Wifi_GPIO0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Wifi_GPIO2_GPIO_Port, Wifi_GPIO2_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);


	//__HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);

	// to do: control ESP - pull gpios/rst pin appropriately
	// after it is soldered to main board

	/* Infinite loop */
	for (;;)
	{
		tick = osKernelSysTick();
		xQueueReceive(qToWifiSetValHandle, &setData, 1000);
		TxBufLen =
				sprintf((char*) TxBuffer,
						"@MarcinSetValues:%u,%u,%u,%d,%d,%d,%u,%u,%lu,%lu,%ld,%u,%u!\r\n",
						setData.h, setData.m, setData.s, setData.ds18[0],
						setData.ds18[1], setData.ds18[2],
						setData.hum, setData.press, setData.lat, setData.lon,
						setData.alt, setData.status, setData.hdop);
		HAL_UART_Transmit_DMA(&huart4, (uint8_t*) TxBuffer, TxBufLen);
		// wait for TX done
		xSemaphoreTake(WiFiTxDoneHandle, 500);
		strcpy(TxBuffer, GetCommand);
		// start listening first
		HAL_UART_Receive_DMA(&huart4, (uint8_t*) RxBuffer, sizeof(RxBuffer));
		// transmit data query command
		HAL_UART_Transmit_DMA(&huart4, (uint8_t*) TxBuffer, sizeof(GetCommand));
		// first, wait for TX to finish
		xSemaphoreTake(WiFiTxDoneHandle, 500);
		// now we wait for RX done
		if (xSemaphoreTake(WiFiRxDoneHandle, 1000) == pdTRUE)
		{
			// if data received, check if it has "MarcinOK" inside
 			if (strstr(RxBuffer, Ok) != NULL)
			{
				for (uint32_t i = 0; i < 30; i++)
				{
					temps[i] = (int16_t) (((uint16_t)RxBuffer[10 + (2 * i)] << 8) + (uint16_t)RxBuffer[11 + (2 * i)]);
				}
				tempInfo.statusKom = ((uint16_t)RxBuffer[70] << 8) + ((uint16_t)RxBuffer[71]);
				// temperature records are as follows:
				// upper sample: 1,2,3,4,5,6
				// Lower Sample: 1,2,3,4,5,6
				// Upper Heater (...)
				// Lower Heater
				// Ambient
				meanCalc = 0;
				tempStatusCalc = 0;
				for (uint32_t i = 0; i < 12; i++)
				{
					meanCalc += (int32_t)temps[i];
				}
				meanCalc /= 12L;
				tempInfo.mean = (int16_t)meanCalc;
				
				for (uint32_t i = 0; i < 12; i++)
				{
					uint8_t val = 0;
					if (temps[i] > 3700)
					{
						val = 1;
					}
					else if (temps[i] < 3500)
					{
						val = 2;
					}
					else
					{
						// temperature ok
						// do nothing cause val = 0
					}
					tempStatusCalc |=  (val << (2 * i));
				}
				tempInfo.highByte = (uint8_t)(tempStatusCalc >> 16);
				tempInfo.middleByte = (uint8_t)(tempStatusCalc >> 8);
				tempInfo.lowByte = (uint8_t)(tempStatusCalc);
			}
		}
		else
		{
			// no data received
			error = 1;
			// try to reset ESP?
			tempInfo.mean = 0xFFFF;
			tempInfo.highByte = 0xFF;
			tempInfo.middleByte = 0xFF;
			tempInfo.lowByte = 0xFF;
			tempInfo.statusKom = 0xFFFF;
		}
		xQueueSend(qFromWifiErrHandle, &error, 0);
		error = 0;
		xQueueReceive(qToWifiHandle, &odcinaczFlag, 0);
		if (odcinaczFlag == 1)
		{
			HAL_UART_Transmit(&huart4, (uint8_t*) CutCommand,
					sizeof(CutCommand), 100);
		}
		xQueueSend(qFromWifiHandle, &tempInfo, 0);
		osDelayUntil(&tick, 5000);
	}
  /* USER CODE END StartWIFITask */
}

/* StartWatchdogTask function */
void StartWatchdogTask(void const * argument)
{
  /* USER CODE BEGIN StartWatchdogTask */
	uint32_t tick = 0;
	uint8_t message = 0;
	uint8_t PowerLEDFlagActive = 0;
	uint8_t powerLEDstate = 0;


	RCC->CSR |= (1 << 0);                  // LSI enable, necessary for IWDG
	while ((RCC->CSR & (1 << 1)) == 0)
		;         // wait till LSI is ready
	IWDG->KR = 0x5555;
	IWDG->PR = 0x00000005;
	IWDG->RLR = 0x00000FFF;
	IWDG->KR = 0xCCCC;
	/* Infinite loop */
	for (;;)
	{
		tick = osKernelSysTick();
		xQueueReceive(qToWatchdogHandle, &message, 10);
		if (message == 0xFF)
		{
			osDelay(10000);
		}
		else
		{
			if (message == 1)
			{
				message = 0;
				if (!PowerLEDFlagActive)
				{
					PowerLEDFlagActive = 1;
					powerLEDstate = 0;
					HAL_GPIO_WritePin(Power_LED_GPIO_Port, Power_LED_Pin, GPIO_PIN_SET);
				}
			}
			else if (message == 2)
			{
				message = 0;
				if (PowerLEDFlagActive)
				{
					PowerLEDFlagActive = 0;
					powerLEDstate = 0;
					HAL_GPIO_WritePin(Power_LED_GPIO_Port, Power_LED_Pin, GPIO_PIN_SET);
				}
			}
			IWDG->KR = 0xAAAA;
		}

		if (PowerLEDFlagActive)
		{
			switch (powerLEDstate)
			{
			case 0:
				HAL_GPIO_WritePin(Power_LED_GPIO_Port, Power_LED_Pin, GPIO_PIN_RESET);
				powerLEDstate++;
				break;
			case 1:
				HAL_GPIO_WritePin(Power_LED_GPIO_Port, Power_LED_Pin, GPIO_PIN_SET);
				//allow to fall through
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
				powerLEDstate++;
				break;
			case 10:
				powerLEDstate = 0;
				break;
			default:
				powerLEDstate = 0;
				break;
			}
		}

		osDelayUntil(&tick, 100);
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
	HAL_GPIO_WritePin(OneWire_PULLUP_GPIO_Port, OneWire_PULLUP_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(Thermal_Signal_Internal_GPIO_Port, Thermal_Signal_Internal_Pin, GPIO_PIN_RESET);
	for (;;)
	{
		BeginTick = osKernelSysTick();
		DS18B20ConvertTemperature();
		DS18B20Temp.t1 = ReadDS18B20(DSAddr[0]);
		DS18B20Temp.t2 = ReadDS18B20(DSAddr[1]);
		DS18B20Temp.t3 = ReadDS18B20(DSAddr[2]);
		xQueueSend(qFromDS18Handle, &DS18B20Temp, 10);
		osDelayUntil(&BeginTick, 1000);
	}
  /* USER CODE END StartDS18Task */
}

/* StartGpsParserTask function */
void StartGpsParserTask(void const * argument)
{
  /* USER CODE BEGIN StartGpsParserTask */
	gps_data_t gps;
	UART_Init();
	/* Infinite loop */
	for (;;)
	{
		xSemaphoreTake(GpsBinarySemHandle, portMAX_DELAY);
		gps = GpsCheckAndProcess();
		if (gps.status == 0x00)
		{
			xQueueSend(qFromGPSHandle, &gps, 0);
		}
		osDelay(5);
	}
  /* USER CODE END StartGpsParserTask */
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
