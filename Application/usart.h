#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"

/*----------------------USART配置宏 ------------------------*/

#define  USART6_BaudRate  115200

#define  USART6_TX_PIN				GPIO_Pin_6					// TX 引脚
#define	USART6_TX_PORT				GPIOC							// TX 引脚端口
#define	USART6_TX_CLK				RCC_AHB1Periph_GPIOC		// TX 引脚时钟
#define  USART6_TX_PinSource     GPIO_PinSource6			// 引脚源

#define  USART6_RX_PIN				GPIO_Pin_7             // RX 引脚
#define	USART6_RX_PORT				GPIOC                   // RX 引脚端口
#define	USART6_RX_CLK				RCC_AHB1Periph_GPIOC    // RX 引脚时钟
#define  USART6_RX_PinSource     GPIO_PinSource7        // 引脚源


/*---------------------- 函数声明 ----------------------------*/

void  Usart_Config (void);	// USART初始化函数
void  USART_GPIO_Config	(void);
void  USART1_Init(uint32_t baudrate);
void  DMA_USART1_TX_Init(uint8_t *buf, uint16_t len);
void NVIC_Configuration(void);
#endif //__USART_H

