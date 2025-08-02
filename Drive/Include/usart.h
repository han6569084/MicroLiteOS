#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "stm32f4xx_hal.h"

#define  USART6_BaudRate  115200

#define  USART6_TX_PIN              GPIO_PIN_6
#define  USART6_TX_PORT             GPIOC

#define  USART6_RX_PIN              GPIO_PIN_7
#define	 USART6_RX_PORT             GPIOC

/*---------------------- USART6 ----------------------------*/

void  Usart_Config (void);
void  USART_GPIO_Config	(void);
void  USART1_Init(uint32_t baudrate);
void  DMA_USART1_TX_Init(uint8_t *buf, uint16_t len);
void  NVIC_Configuration(void);
#endif //__USART_H

