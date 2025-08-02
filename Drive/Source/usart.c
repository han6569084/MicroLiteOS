#include "usart.h"
#include <stdio.h>
#include <string.h>

#define TX_BUF_SIZE (256)

// UART handles for USART1 and USART6
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

volatile uint8_t buf[TX_BUF_SIZE];
volatile uint16_t dma_tx_len = 0;
volatile uint8_t dma_busy = 0;

#ifndef STDOUT_FILENO
#define STDOUT_FILENO 1
#endif

#ifndef STDERR_FILENO
#define STDERR_FILENO 2
#endif

void  USART_GPIO_Config	(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	__HAL_RCC_GPIOC_CLK_ENABLE();  // USART6 TX/RX pins

	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_InitStructure.Pin = USART6_TX_PIN;
	GPIO_InitStructure.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(USART6_TX_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = USART6_RX_PIN;
	GPIO_InitStructure.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(USART6_RX_PORT, &GPIO_InitStructure);

}

void USART1_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();   // PA9, PA10
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure UART handle
    huart1.Instance = USART1;
    huart1.Init.BaudRate = baudrate;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.Mode = UART_MODE_TX | UART_MODE_RX;
    HAL_UART_Init(&huart1);
}

/*
void DMA_USART1_TX_Init(uint8_t *buf, uint16_t len)
{
    // TODO: Convert DMA configuration to HAL library
    // This function contains complex DMA setup that needs to be rewritten
    // For now, we'll use polling-based UART transmission
}
*/

void Usart_Config(void)
{
	__HAL_RCC_USART6_CLK_ENABLE();

	USART_GPIO_Config();

	// Configure UART6 handle
	huart6.Instance = USART6;
	huart6.Init.BaudRate = USART6_BaudRate;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_RX | UART_MODE_TX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;

	HAL_UART_Init(&huart6);
}

/*
void NVIC_Configuration(void)
{
    // TODO: Convert NVIC configuration to HAL library
    // This function needs to be rewritten using HAL_NVIC_SetPriority and HAL_NVIC_EnableIRQ
}
*/

/*
void DMA2_Stream7_IRQHandler(void)
{
    // TODO: Convert DMA interrupt handler to HAL library
}
*/

int usart_send_char(int c)
{
	// HAL_UART_Transmit(&huart6, (uint8_t*)&c, 1, HAL_MAX_DELAY);
	while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TXE) == RESET);

	return (c);
}

int _write(int fd, char *ptr, int len)
{
    for (int i = 0; i < len; i++) {
        usart_send_char(ptr[i]);
    }
    return len;
}

int fputc(int c, FILE *fp)
{
    // TODO: Implement HAL library UART transmission
    // HAL_UART_Transmit(&huart, (uint8_t*)&c, 1, HAL_MAX_DELAY);
    return c;
}

/*
int fputc_bak(int c, FILE *fp)
{
    // TODO: This function uses DMA variables that are not defined
    // Comment out for now until DMA implementation is complete
    return c;
}
*/
