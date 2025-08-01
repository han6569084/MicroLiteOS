#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"

// LED pin definitions
/*******************************************************/
// Red LED
#define LED1_PIN        GPIO_Pin_10
#define LED1_GPIO_PORT  GPIOH
#define LED1_GPIO_CLK   RCC_AHB1Periph_GPIOH

// Green LED
#define LED2_PIN        GPIO_Pin_11
#define LED2_GPIO_PORT  GPIOH
#define LED2_GPIO_CLK   RCC_AHB1Periph_GPIOH

// Blue LED
#define LED3_PIN        GPIO_Pin_12
#define LED3_GPIO_PORT  GPIOH
#define LED3_GPIO_CLK   RCC_AHB1Periph_GPIOH
/*******************************************************/

/**
 * LED logic level definition:
 * ON = 0, OFF = 1 for low-level active LEDs
 * If your LED is high-level active, you can set ON = 1, OFF = 0
 */
#define ON  0
#define OFF 1

/* Macro for direct LED control, can be used anywhere */
#define LED1(a)    do { if (a) GPIO_SetBits(LED1_GPIO_PORT, LED1_PIN); else GPIO_ResetBits(LED1_GPIO_PORT, LED1_PIN); } while(0)
#define LED2(a)    do { if (a) GPIO_SetBits(LED2_GPIO_PORT, LED2_PIN); else GPIO_ResetBits(LED2_GPIO_PORT, LED2_PIN); } while(0)
#define LED3(a)    do { if (a) GPIO_SetBits(LED3_GPIO_PORT, LED3_PIN); else GPIO_ResetBits(LED3_GPIO_PORT, LED3_PIN); } while(0)
#define LED4(a)    do { if (a) GPIO_SetBits(LED4_GPIO_PORT, LED4_PIN); else GPIO_ResetBits(LED4_GPIO_PORT, LED4_PIN); } while(0)

/* Direct register operation macros for IO */
#define digitalHi(p, i)         { (p)->BSRRL = (i); }    // Set pin high
#define digitalLo(p, i)         { (p)->BSRRH = (i); }    // Set pin low
#define digitalToggle(p, i)     { (p)->ODR ^= (i); }     // Toggle pin state

/* LED operation macros */
#define LED1_TOGGLE     digitalToggle(LED1_GPIO_PORT, LED1_PIN)
#define LED1_OFF        digitalHi(LED1_GPIO_PORT, LED1_PIN)
#define LED1_ON         digitalLo(LED1_GPIO_PORT, LED1_PIN)

#define LED2_TOGGLE     digitalToggle(LED2_GPIO_PORT, LED2_PIN)
#define LED2_OFF        digitalHi(LED2_GPIO_PORT, LED2_PIN)
#define LED2_ON         digitalLo(LED2_GPIO_PORT, LED2_PIN)

#define LED3_TOGGLE     digitalToggle(LED3_GPIO_PORT, LED3_PIN)
#define LED3_OFF        digitalHi(LED3_GPIO_PORT, LED3_PIN)
#define LED3_ON         digitalLo(LED3_GPIO_PORT, LED3_PIN)

/* RGB LED color macros, for full color mixing use PWM, these are for simple effects */

// Red
#define LED_RED      do { LED1_ON; LED2_OFF; LED3_OFF; } while(0)
// Green
#define LED_GREEN    do { LED1_OFF; LED2_ON; LED3_OFF; } while(0)
// Blue
#define LED_BLUE     do { LED1_OFF; LED2_OFF; LED3_ON; } while(0)
// Yellow (Red + Green)
#define LED_YELLOW   do { LED1_ON; LED2_ON; LED3_OFF; } while(0)
// Purple (Red + Blue)
#define LED_PURPLE   do { LED1_ON; LED2_OFF; LED3_ON; } while(0)
// Cyan (Green + Blue)
#define LED_CYAN     do { LED1_OFF; LED2_ON; LED3_ON; } while(0)
// White (Red + Green + Blue)
#define LED_WHITE    do { LED1_ON; LED2_ON; LED3_ON; } while(0)
// All off
#define LED_RGBOFF   do { LED1_OFF; LED2_OFF; LED3_OFF; } while(0)

void LED_GPIO_Config(void);
void KEY_GPIO_Config(void);

#endif /* __LED_H */
