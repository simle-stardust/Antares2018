#include "stm32l1xx_hal.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

void USART_IrqHandler (UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

void DMA_IrqHandler (DMA_HandleTypeDef *hdma, UART_HandleTypeDef *huart);

uint8_t check_for (char *string);
