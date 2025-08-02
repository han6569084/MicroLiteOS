#include "bsp_mpu_exti.h"

static void NVIC_Configuration(void)
{
  /* 设置NVIC优先级 */
  HAL_NVIC_SetPriority(MPU_INT_EXTI_IRQ, 1, 1);
  HAL_NVIC_EnableIRQ(MPU_INT_EXTI_IRQ);
}

void EXTI_MPU_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	__HAL_RCC_GPIOI_CLK_ENABLE();

  GPIO_InitStructure.Pin = MPU_INT_GPIO_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_INT_GPIO_PORT, &GPIO_InitStructure);

  /* 配置NVIC */
  NVIC_Configuration();
}

void EXTI_MPU_Disable(void)
{
  /* 禁用NVIC中断 */
  HAL_NVIC_DisableIRQ(MPU_INT_EXTI_IRQ);
}

extern void gyro_data_ready_cb(void);

void MPU_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(MPU_INT_GPIO_PIN) != RESET)
	{
		/* Handle new gyro*/
		gyro_data_ready_cb();

		__HAL_GPIO_EXTI_CLEAR_IT(MPU_INT_GPIO_PIN);
	}
}

/*********************************************END OF FILE**********************/
