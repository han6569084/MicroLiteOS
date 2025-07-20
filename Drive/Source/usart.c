#include "usart.h"
#include <stdio.h>
#include <string.h>

#define TX_BUF_SIZE (256)

volatile uint8_t dma_tx_buf[TX_BUF_SIZE];
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
	RCC_AHB1PeriphClockCmd ( USART6_TX_CLK|USART6_RX_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin = USART6_TX_PIN;
	GPIO_Init(USART6_TX_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = USART6_RX_PIN;
	GPIO_Init(USART6_RX_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(USART6_TX_PORT,USART6_TX_PinSource,GPIO_AF_USART6);
	GPIO_PinAFConfig(USART6_RX_PORT,USART6_RX_PinSource,GPIO_AF_USART6);
}

void USART1_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   // PA9, PA10
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); // TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // RX

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStruct);

    USART_Cmd(USART1, ENABLE);
}

void DMA_USART1_TX_Init(uint8_t *buf, uint16_t len)
{
    DMA_InitTypeDef DMA_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    DMA_DeInit(DMA2_Stream7);

    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);

    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)buf;
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStruct.DMA_BufferSize = len;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream7, &DMA_InitStruct);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA2_Stream7, ENABLE);
}

void Usart_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	USART_GPIO_Config();

	USART_InitStructure.USART_BaudRate 	 = USART6_BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits   = USART_StopBits_1;
	USART_InitStructure.USART_Parity     = USART_Parity_No ;
	USART_InitStructure.USART_Mode 	    = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init(USART6,&USART_InitStructure);
	USART_Cmd(USART6,ENABLE);
}

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream7_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void DMA2_Stream7_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7))
    {
        DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
        dma_busy = 0;
        DMA_Cmd(DMA2_Stream7, DISABLE);
    }
}

int usart_send_char(int c)
{
	USART_SendData( USART6,(u8)c );
	while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);

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
	USART_SendData( USART6,(u8)c );
	while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);

	return (c);
}

int fputc_bak(int c, FILE *fp)
{
	while (dma_busy);

    dma_tx_buf[dma_tx_len++] = c;
    if(c == '\n' || dma_tx_len >= TX_BUF_SIZE)
    {
        dma_busy = 1;
        DMA_USART1_TX_Init((uint8_t *)dma_tx_buf, dma_tx_len);
    }

	return (c);
}
