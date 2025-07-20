
/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   LED application function interface
  ******************************************************************************
  * @attention
  *
  * Platform: Wildfire STM32 F429 development board
  * Forum   : http://www.firebbs.cn
  * Store   : https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */

#include "bsp_led.h"


/**
  * @brief  Initialize the IO for the onboard LEDs
  * @param  None
  * @retval None
  */
void LED_GPIO_Config(void)
{
    /* Create a GPIO_InitTypeDef structure */
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable the GPIO clock for the LEDs */
    RCC_AHB1PeriphClockCmd(LED1_GPIO_CLK |
                           LED2_GPIO_CLK |
                           LED3_GPIO_CLK, ENABLE);

    /* Select the GPIO pin to configure */
    GPIO_InitStructure.GPIO_Pin = LED1_PIN;

    /* Set the mode to output */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

    /* Set output type to push-pull */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

    /* Set pull-up mode */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    /* Set speed to 2MHz */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    /* Initialize GPIO for LED1 */
    GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);

    /* Initialize GPIO for LED2 */
    GPIO_InitStructure.GPIO_Pin = LED2_PIN;
    GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);

    /* Initialize GPIO for LED3 */
    GPIO_InitStructure.GPIO_Pin = LED3_PIN;
    GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);

    /* Turn off all RGB LEDs */
    LED_RGBOFF;
}

void KEY_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    /* Select the GPIO pin to configure */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;

    /* Set the mode to input */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;

    /* Set no pull-up or pull-down */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    /* Set speed to 2MHz */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    /* Initialize GPIO for KEY (should be GPIOC, not LED1_GPIO_PORT) */
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/*********************************************END OF FILE**********************/
