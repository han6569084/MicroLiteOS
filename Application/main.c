#include "stm32f4xx_hal.h"
#include "bsp_led.h"
#include "i2c.h"
#include "bsp_mpu_exti.h"
#include "usart.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include "cmsis_os2.h"
#include "bsp_sdram.h"
#include "bsp_lcd.h"
#include <string.h>
#include "cm_backtrace.h"
#include "ui_app.h"

/* Standard includes. */
#include <stdio.h>

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

#if ( configCHECK_FOR_STACK_OVERFLOW > 0 )

    void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName )
    {
        /* Check pcTaskName for the name of the offending task,
         * or pxCurrentTCB if pcTaskName has itself been corrupted. */
        ( void ) xTask;
        ( void ) pcTaskName;
    }

#endif /* #if ( configCHECK_FOR_STACK_OVERFLOW > 0 ) */

void vApplicationMallocFailedHook( void )
{
    /* This function will be called if a call to pvPortMalloc() fails.
     * It is a hook function that can be defined by the application writer. */
    printf("Malloc failed!\r\n");
    for( ;; );
}
#if ( configUSE_IDLE_HOOK == 1 )
    void vApplicationIdleHook( void )
    {
        /* This function will be called on each cycle of the idle task.
         * It is a hook function that can be defined by the application writer. */
        /* Do nothing here, just to show that this hook can be used. */
        printf("Idle Hook called!\r\n");
    }
#endif

// void HardFault_Handler(void)
// {
//     __asm volatile
//     (
//         "TST lr, #4                        \n"
//         "ITE EQ                            \n"
//         "MRSEQ r0, MSP                     \n"
//         "MRSNE r0, PSP                     \n"
//         "B HardFault_Handler_C             \n"
//     );
// }

void HardFault_Handler_C(uint32_t *hardfault_args)
{
    uint32_t stacked_r0  = hardfault_args[0];
    uint32_t stacked_r1  = hardfault_args[1];
    uint32_t stacked_r2  = hardfault_args[2];
    uint32_t stacked_r3  = hardfault_args[3];
    uint32_t stacked_r12 = hardfault_args[4];
    uint32_t stacked_lr  = hardfault_args[5];
    uint32_t stacked_pc  = hardfault_args[6];
    uint32_t stacked_psr = hardfault_args[7];

    printf("HardFault!\r\n");
    printf("R0  = 0x%08lx\r\n", stacked_r0);
    printf("R1  = 0x%08lx\r\n", stacked_r1);
    printf("R2  = 0x%08lx\r\n", stacked_r2);
    printf("R3  = 0x%08lx\r\n", stacked_r3);
    printf("R12 = 0x%08lx\r\n", stacked_r12);
    printf("LR  = 0x%08lx\r\n", stacked_lr);
    printf("PC  = 0x%08lx\r\n", stacked_pc);
    printf("PSR = 0x%08lx\r\n", stacked_psr);

    while (1)
    {
        // 死循环，便于调试
    }
}

/*-----------------------------------------------------------*/
volatile uint8_t s_ledColor = 0;
volatile uint64_t s_timeLeft = 0;

// void EXTI15_10_IRQHandler(void)
// {
// 	EXTI_ClearFlag(EXTI_Line13);
// 	s_ledColor++;
// }

// void SysTick_Handler(void)
// {
//     if(s_timeLeft)
//     {
//         s_timeLeft--;
//     }
// }

void EXTI_Config()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}



void gyro_data_ready_cb(void)
{
    s_ledColor++;
}

void DMA2D_IRQHandler(void)
{
    printf("DMA2D_IRQHandler called\r\n");
    // printf("DMA2D_IRQHandler priority: %d\r\n", NVIC->IP[90]);
    // DMA2D_ClearFlag(DMA2D_FLAG_TC | DMA2D_FLAG_TW | DMA2D_FLAG_CE | DMA2D_FLAG_TE);
    // lv_draw_dma2d_transfer_complete_interrupt_handler();
    while (1)
    {

    }
}





float g_temp = 0.0f;
float g_humid = 0.0f;
int   g_posx = 0;
int   g_posy = 0;

static void sysCoreTask( void * argument )
{
    printf("sysCoreTask start\r\n");
    ui_app_init();
    ui_app_task_loop();
}

__attribute__((section(".sdram"))) uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];

void MPU_Config(void)
{
    MPU->CTRL = 0;

    // // SRAM: 0x20000000, 128KB
    // MPU->RNR = 0;
    // MPU->RBAR = 0x20000000;
    // MPU->RASR =
    //     (0x3 << 24) | // Full access
    //     (0x0 << 19) | // TEX=0
    //     (0x1 << 18) | // S=1
    //     (0x1 << 17) | // C=1
    //     (0x1 << 16) | // B=1
    //     (0x10 << 1) | // 128KB (2^(10+1))
    //     (1 << 0);

    // // FLASH: 0x08000000, 2MB
    // MPU->RNR = 1;
    // MPU->RBAR = 0x08000000;
    // MPU->RASR =
    //     (0x3 << 24) |
    //     (0x0 << 19) |
    //     (0x1 << 18) |
    //     (0x1 << 17) |
    //     (0x1 << 16) |
    //     (0x15 << 1) | // 2MB (2^(21+1))
    //     (1 << 0);

    // SDRAM: 0xD0000000, 32MB
    MPU->RNR = 0;
    MPU->RBAR = 0xD0000000;
    MPU->RASR =
        (0x3 << 24) | // Full access
        (0x1 << 19) | // TEX=1 (strongly recommended for external RAM)
        (0x1 << 18) | // S=1
        (0x1 << 17) | // C=1
        (0x1 << 16) | // B=1
        (0x18 << 1) | // 32MB (2^(24+1))
        (1 << 0);

    // // 禁止所有未配置区域（region 7，4GB）
    // MPU->RNR = 7;
    // MPU->RBAR = 0x00000000;
    // MPU->RASR =
    //     (0x0 << 24) | // No access
    //     (0x0 << 19) |
    //     (0x0 << 18) |
    //     (0x0 << 17) |
    //     (0x0 << 16) |
    //     (0x1F << 1) | // 4GB (2^(31+1))
    //     (1 << 0);

    // 使能MPU，关闭PRIVDEFENA
    MPU->CTRL = MPU_CTRL_ENABLE_Msk;

    __DSB();
    __ISB();
}

int main( void )
{
    SystemCoreClockUpdate();
    // 清除SDRAM所有数据，全部置为0

    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk; // 使能 UsageFault
    FPU->FPDSCR |= (1 << 8); // 使能分割异常（可选）
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk; // 使能整数除零陷阱（对浮点无效）
    LED_GPIO_Config();
    // KEY_GPIO_Config();
    SDRAM_Init();
    memset((void *)0xD0000000, 0, 32 * 1024 * 1024); // 32MB SDRAM全部清零
    NVIC_SetPriority(EXTI15_10_IRQn, 5);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriorityGrouping(0);
    NVIC_SetPriority(DMA2D_IRQn, 8);
    // EXTI_Config();
    EXTI_MPU_Config();
    // I2cMaster_Init();
    // SHT4x_DMA_Init();
    Usart_Config();
    // MPU_Config();
    cm_backtrace_init("STM32F429_Project", "v1.0.0", "v1.0.0");

    printf("DMA2D_IRQn = %d, IP = %ld\r\n", DMA2D_IRQn, NVIC_GetPriority(DMA2D_IRQn));
    ( void ) printf( "sysCoreTask FreeRTOS Project\r\n" );
    printf("SystemCoreClock = %lu\r\n", SystemCoreClock);

    osKernelInitialize();
    osThreadNew(sysCoreTask, NULL, NULL);
    osKernelStart();

    printf("Scheduler returned!\r\n");
    for( ; ; )
    {
        /* Should not reach here. */
    }
}
