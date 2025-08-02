
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

    /* Enable the GPIO clock for the LEDs - HAL library manages clocks automatically */
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Select the GPIO pin to configure */
    GPIO_InitStructure.Pin = LED1_PIN;

    /* Set the mode to output push-pull */
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;

    /* Set pull-up mode */
    GPIO_InitStructure.Pull = GPIO_PULLUP;

    /* Set speed to low */
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;

    /* Initialize GPIO for LED1 */
    HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);

    /* Initialize GPIO for LED2 */
    GPIO_InitStructure.Pin = LED2_PIN;
    HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);

    /* Initialize GPIO for LED3 */
    GPIO_InitStructure.Pin = LED3_PIN;
    HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);

    /* Turn off all RGB LEDs */
    LED_RGBOFF;
}

void KEY_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOC clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Select the GPIO pin to configure */
    GPIO_InitStructure.Pin = GPIO_PIN_13;

    /* Set the mode to input */
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;

    /* Set no pull-up or pull-down */
    GPIO_InitStructure.Pull = GPIO_NOPULL;

    /* Set speed to low */
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;

    /* Initialize GPIO for KEY */
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/*********************************************END OF FILE**********************/
