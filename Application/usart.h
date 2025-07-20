#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

/*----------------------USART���ú� ------------------------*/

#define  USART6_BaudRate  115200

#define  USART6_TX_PIN				GPIO_Pin_6					// TX ����
#define	USART6_TX_PORT				GPIOC							// TX ���Ŷ˿�
#define	USART6_TX_CLK				RCC_AHB1Periph_GPIOC		// TX ����ʱ��
#define  USART6_TX_PinSource     GPIO_PinSource6			// ����Դ

#define  USART6_RX_PIN				GPIO_Pin_7             // RX ����
#define	USART6_RX_PORT				GPIOC                   // RX ���Ŷ˿�
#define	USART6_RX_CLK				RCC_AHB1Periph_GPIOC    // RX ����ʱ��
#define  USART6_RX_PinSource     GPIO_PinSource7        // ����Դ


/*---------------------- �������� ----------------------------*/

void  Usart_Config (void);	// USART��ʼ������
void  USART_GPIO_Config	(void);
void  USART1_Init(uint32_t baudrate);
void  DMA_USART1_TX_Init(uint8_t *buf, uint16_t len);
void NVIC_Configuration(void);
#endif //__USART_H

