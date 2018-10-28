/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define D5_Pin GPIO_PIN_13
#define D5_GPIO_Port GPIOC
#define D4_Pin GPIO_PIN_14
#define D4_GPIO_Port GPIOC
#define D3_Pin GPIO_PIN_15
#define D3_GPIO_Port GPIOC
#define ADC_Thermal_Pin GPIO_PIN_0
#define ADC_Thermal_GPIO_Port GPIOC
#define ADC_Ele_Pin GPIO_PIN_1
#define ADC_Ele_GPIO_Port GPIOC
#define ADC_LoRa_Pin GPIO_PIN_2
#define ADC_LoRa_GPIO_Port GPIOC
#define D2_Pin GPIO_PIN_3
#define D2_GPIO_Port GPIOC
#define D1_Pin GPIO_PIN_0
#define D1_GPIO_Port GPIOA
#define D0_Pin GPIO_PIN_1
#define D0_GPIO_Port GPIOA
#define LoRa_CS_Pin GPIO_PIN_4
#define LoRa_CS_GPIO_Port GPIOA
#define LoRa_SCK_Pin GPIO_PIN_5
#define LoRa_SCK_GPIO_Port GPIOA
#define LoRa_MISO_Pin GPIO_PIN_6
#define LoRa_MISO_GPIO_Port GPIOA
#define LoRa_MOSI_Pin GPIO_PIN_7
#define LoRa_MOSI_GPIO_Port GPIOA
#define LoRa_RST_Pin GPIO_PIN_4
#define LoRa_RST_GPIO_Port GPIOC
#define LoRa_RX_Pin GPIO_PIN_5
#define LoRa_RX_GPIO_Port GPIOC
#define LoRa_TX_Pin GPIO_PIN_0
#define LoRa_TX_GPIO_Port GPIOB
#define Thermal_Signal_Pin GPIO_PIN_1
#define Thermal_Signal_GPIO_Port GPIOB
#define Buzzer_Signal_Pin GPIO_PIN_2
#define Buzzer_Signal_GPIO_Port GPIOB
#define Sensors_SCL_Pin GPIO_PIN_10
#define Sensors_SCL_GPIO_Port GPIOB
#define Sensoors_SDA_Pin GPIO_PIN_11
#define Sensoors_SDA_GPIO_Port GPIOB
#define OneWire_Pin GPIO_PIN_12
#define OneWire_GPIO_Port GPIOB
#define RAD_SCK_Pin GPIO_PIN_13
#define RAD_SCK_GPIO_Port GPIOB
#define RAD_MISO_Pin GPIO_PIN_14
#define RAD_MISO_GPIO_Port GPIOB
#define RAD_MOSI_Pin GPIO_PIN_15
#define RAD_MOSI_GPIO_Port GPIOB
#define RAD_CS_Pin GPIO_PIN_6
#define RAD_CS_GPIO_Port GPIOC
#define Button_Pin GPIO_PIN_7
#define Button_GPIO_Port GPIOC
#define WIFI_GPIO0_Pin GPIO_PIN_9
#define WIFI_GPIO0_GPIO_Port GPIOC
#define WiFi_RST_Pin GPIO_PIN_8
#define WiFi_RST_GPIO_Port GPIOA
#define WiFi_TX_Pin GPIO_PIN_9
#define WiFi_TX_GPIO_Port GPIOA
#define WiFi_RX_Pin GPIO_PIN_10
#define WiFi_RX_GPIO_Port GPIOA
#define WiFi_CH_PD_Pin GPIO_PIN_11
#define WiFi_CH_PD_GPIO_Port GPIOA
#define WiFi_GPIO2_Pin GPIO_PIN_12
#define WiFi_GPIO2_GPIO_Port GPIOA
#define BLE_RST_Pin GPIO_PIN_15
#define BLE_RST_GPIO_Port GPIOA
#define BLE_TX_Pin GPIO_PIN_10
#define BLE_TX_GPIO_Port GPIOC
#define BLE_RX_Pin GPIO_PIN_11
#define BLE_RX_GPIO_Port GPIOC
#define BLE_KEY_Pin GPIO_PIN_3
#define BLE_KEY_GPIO_Port GPIOB
#define OneWire_PULLUP_Pin GPIO_PIN_4
#define OneWire_PULLUP_GPIO_Port GPIOB
#define Geiger_SCL_Pin GPIO_PIN_6
#define Geiger_SCL_GPIO_Port GPIOB
#define Geiger_SDA_Pin GPIO_PIN_7
#define Geiger_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define DATA_MODE 0x01
#define I2CERROR_MODE 0x02

#define I2C_PRESSURE_ERR 	(uint16_t)0x0001
#define I2C_RTC_ERR      	(uint16_t)0x0002
#define I2C_GEIGER_ERR   	(uint16_t)0x0004
#define I2C_MAX30205_ERR 	(uint16_t)0x0008
#define I2C_HDC1080_ERR  	(uint16_t)0x0010
#define I2C_INA3221_ERR  	(uint16_t)0x0020
#define I2C_GYRO_ERR     	(uint16_t)0x0040
#define I2C_ACC_ERR      	(uint16_t)0x0080
#define I2C_BARO_ERR     	(uint16_t)0x0100
#define DS18_ERR			(uint16_t)0x0200
#define SD_ERR				(uint16_t)0x0400
#define GPS_ERR				(uint16_t)0x0800
#define WIFI_ERR			(uint16_t)0x1000
#define BLE_ERR				(uint16_t)0x2000
#define LORA_ERR			(uint16_t)0x4000
#define UNDEF_ERR2			(uint16_t)0x8000
static const char error_strings[17][17] = {"I2C_PRESSURE_ERR", "I2C_RTC_ERR","I2C_GEIGER_ERR",
		"I2C_MAX30205_ERR","I2C_HDC1080_ERR","I2C_INA3221_ERR","I2C_GYRO_ERR",
		"I2C_ACC_ERR","I2C_BARO_ERR","DS18_ERR","SD_ERR",
		"GPS_ERR","WIFI_ERR","BLE_ERR","LORA_ERR",
		"UNDEF_ERR2",
};
/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
