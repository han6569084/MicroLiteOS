#include "stm32f4xx.h"
#include "./led/bsp_led.h"
#include "i2c.h"
#include "bsp_mpu_exti.h"
#include "usart.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include "cmsis_os2.h"

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

void HardFault_Handler(void)
{
    __asm volatile
    (
        "TST lr, #4                        \n"
        "ITE EQ                            \n"
        "MRSEQ r0, MSP                     \n"
        "MRSNE r0, PSP                     \n"
        "B HardFault_Handler_C             \n"
    );
}

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

void EXTI15_10_IRQHandler(void)
{
	EXTI_ClearFlag(EXTI_Line13);
	s_ledColor++;
}

// void SysTick_Handler(void)
// {
//     if(s_timeLeft)
//     {
//         s_timeLeft--;
//     }
// }

void EXTI_Config()
{
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Line = EXTI_Line13;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);
	EXTI_Init(&EXTI_InitStruct);
}

#define SHT4x_ADDR              0x88
#define HIGH_PRECISION_CMD      0xFD
#define MEDIUM_PRECISION_CMD    0xF6
#define LOW_PRECISION_CMD       0xE0
#define SOFT_RESET_CMD          0x94
#define SHT4x_I2C               I2C1

volatile uint8_t i2cRxBuffer[6] ;//__attribute__((aligned(4)));
volatile uint8_t dmaComplete = 0;

void DMA1_Stream0_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0)) {
        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
        dmaComplete = 1;
    }
}
static uint8_t SHT4x_CRC8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    uint8_t i, j;

    for(i = 0; i < len; i++) {
        crc ^= data[i];
        for(j = 0; j < 8; j++) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static ErrorStatus I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT, uint32_t timeout)
{
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT)) {
        if((timeout--) == 0) return ERROR;
    }
    return SUCCESS;
}

ErrorStatus SHT4x_ReadTempHumid(float *temperature, float *humidity, uint8_t precision_cmd)
{
    uint8_t cmd = precision_cmd;
    uint8_t data[6];
    uint8_t i = 0;
    uint8_t crc;
    uint16_t temp_raw;
    uint16_t humid_raw;

    I2C_GenerateSTART(SHT4x_I2C, ENABLE);
    if(I2C_WaitEvent(SHT4x_I2C, I2C_EVENT_MASTER_MODE_SELECT, 1000) == ERROR)
        goto ERROR_NACK;

    I2C_Send7bitAddress(SHT4x_I2C, SHT4x_ADDR, I2C_Direction_Transmitter);
    if(I2C_WaitEvent(SHT4x_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 1000) == ERROR)
        goto ERROR_NACK;

    I2C_SendData(SHT4x_I2C, cmd);
    if(I2C_WaitEvent(SHT4x_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED, 1000) == ERROR)
        goto ERROR_NACK;

    I2C_GenerateSTOP(SHT4x_I2C, ENABLE);

    switch(precision_cmd) {
        case HIGH_PRECISION_CMD: vTaskDelay(9); break;    //
        case MEDIUM_PRECISION_CMD: vTaskDelay(5); break;   //
        case LOW_PRECISION_CMD: vTaskDelay(2); break;      //
        default: vTaskDelay(10); break;
    }
    // vTaskDelay(8000);
    I2C_GenerateSTART(SHT4x_I2C, ENABLE);
    if(I2C_WaitEvent(SHT4x_I2C, I2C_EVENT_MASTER_MODE_SELECT, 1000) == ERROR)
        goto ERROR_NACK;

    I2C_Send7bitAddress(SHT4x_I2C, SHT4x_ADDR, I2C_Direction_Receiver);
    if(I2C_WaitEvent(SHT4x_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, 1000) == ERROR)
        goto ERROR_NACK;

    for(i = 0; i < 6; i++) {
        if(i == 5) {
            I2C_AcknowledgeConfig(SHT4x_I2C, DISABLE);
        }

        if(I2C_WaitEvent(SHT4x_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED, 1000) == ERROR) {
            I2C_AcknowledgeConfig(SHT4x_I2C, ENABLE);
            return ERROR;
        }

        data[i] = I2C_ReceiveData(SHT4x_I2C);
    }

    I2C_GenerateSTOP(SHT4x_I2C, ENABLE);
    I2C_AcknowledgeConfig(SHT4x_I2C, ENABLE);

    crc = SHT4x_CRC8(&data[0], 2);
    if (crc != data[2]) {
        printf("CRC Error (Temperature)!\r\n");
        return ERROR;
    }

    crc = SHT4x_CRC8(&data[3], 2);
    if (crc != data[5]) {
        printf("CRC Error (Humidity)!\r\n");
        return ERROR;
    }

    printf("data: %02x %02x %02x %02x %02x %02x \r\n", data[0], data[1], data[2], data[3],
            data[4], data[5]);

    temp_raw = (data[0] << 8) | data[1];
    humid_raw = (data[3] << 8) | data[4];

    *temperature = -45.0f + (175.0f * ((float)temp_raw / 65535.0f));
    *humidity = -6.0f + (125.0f * ((float)humid_raw / 65535.0f));

    if(*humidity > 100.0f) *humidity = 100.0f;
    if(*humidity < 0.0f) *humidity = 0.0f;
    return SUCCESS;
ERROR_NACK:

    I2C_GenerateSTOP(SHT4x_I2C, ENABLE);
    I2C_AcknowledgeConfig(SHT4x_I2C, ENABLE);
    if (I2C_GetFlagStatus(SHT4x_I2C, I2C_FLAG_AF)) {
        I2C_ClearFlag(SHT4x_I2C, I2C_FLAG_AF);
    }
    while (I2C_GetFlagStatus(SHT4x_I2C, I2C_FLAG_BUSY));
    return ERROR;
}

void gyro_data_ready_cb(void)
{
	s_ledColor++;
}

void SHT4x_DMA_Init(void)
{
    DMA_InitTypeDef dmaInit;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Stream0);
    dmaInit.DMA_Channel = DMA_Channel_1;
    dmaInit.DMA_PeripheralBaseAddr = (uint32_t)&(I2C1->DR);
    dmaInit.DMA_Memory0BaseAddr = (uint32_t)i2cRxBuffer;
    dmaInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dmaInit.DMA_BufferSize = 6;
    dmaInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInit.DMA_Mode = DMA_Mode_Normal;
    dmaInit.DMA_Priority = DMA_Priority_VeryHigh;
    dmaInit.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dmaInit.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    dmaInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dmaInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream0, &dmaInit);
    DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

ErrorStatus SHT4x_ReadTempHumid_DMA(float *temperature, float *humidity, uint8_t precision_cmd)
{
    uint8_t cmd = precision_cmd;
    uint8_t crc;
    uint16_t temp_raw, humid_raw;

    I2C_GenerateSTART(I2C1, ENABLE);
    if(I2C_WaitEvent(SHT4x_I2C, I2C_EVENT_MASTER_MODE_SELECT, 1000) == ERROR)
    {
        printf("I2C_WaitEvent I2C_EVENT_MASTER_MODE_SELECT faild\r\n");
        goto DMA_ERROR_NACK;
    }

    I2C_Send7bitAddress(I2C1, SHT4x_ADDR, I2C_Direction_Transmitter);
    if(I2C_WaitEvent(SHT4x_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, 1000) == ERROR)
    {
        printf("I2C_WaitEvent I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED faild\r\n");
        goto DMA_ERROR_NACK;
    }

    I2C_SendData(I2C1, cmd);
    if(I2C_WaitEvent(SHT4x_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED, 1000) == ERROR)
    {
        printf("I2C_WaitEvent I2C_EVENT_MASTER_BYTE_TRANSMITTED faild\r\n");
        goto DMA_ERROR_NACK;
    }

    I2C_GenerateSTOP(I2C1, ENABLE);

    switch (precision_cmd) {
        case HIGH_PRECISION_CMD: vTaskDelay(9); break;    // 8.2ms
        case MEDIUM_PRECISION_CMD: vTaskDelay(5); break;  // 4.5ms
        case LOW_PRECISION_CMD: vTaskDelay(2); break;     // 1.5ms
        default: vTaskDelay(10); break;
    }

    printf("I2C_DMA \r\n");
    DMA_Cmd(DMA1_Stream0, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Stream0, 6);
    DMA_Cmd(DMA1_Stream0, ENABLE);
    I2C_DMALastTransferCmd(I2C1, ENABLE);

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    I2C_GenerateSTART(I2C1, ENABLE);
    if(I2C_WaitEvent(SHT4x_I2C, I2C_EVENT_MASTER_MODE_SELECT, 1000) == ERROR)
    {
        printf("I2C_WaitEvent I2C_EVENT_MASTER_MODE_SELECT faild\r\n");
        goto DMA_ERROR_NACK;
    }

    I2C_Send7bitAddress(I2C1, SHT4x_ADDR, I2C_Direction_Receiver);
    if(I2C_WaitEvent(SHT4x_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, 1000) == ERROR)
    {
        printf("I2C_WaitEvent I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED faild\r\n");
        goto DMA_ERROR_NACK;
    }

    dmaComplete = 0;
    I2C_DMACmd(I2C1, ENABLE);

    while (!dmaComplete) {
        vTaskDelay(1);
    }

    I2C_DMACmd(I2C1, DISABLE);
    DMA_Cmd(DMA1_Stream0, DISABLE);

    if (!dmaComplete) {
        I2C_GenerateSTOP(I2C1, ENABLE);
        return ERROR;
    }

    I2C_GenerateSTOP(I2C1, ENABLE);
    DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);

    printf("i2cRxBuffer: %x %x %x %x %x %x \r\n", i2cRxBuffer[0], i2cRxBuffer[1], i2cRxBuffer[2], i2cRxBuffer[3],
            i2cRxBuffer[4], i2cRxBuffer[5]);

    crc = SHT4x_CRC8(&i2cRxBuffer[0], 2);
    if (crc != i2cRxBuffer[2]) return ERROR;

    crc = SHT4x_CRC8(&i2cRxBuffer[3], 2);
    if (crc != i2cRxBuffer[5]) return ERROR;

    temp_raw = (i2cRxBuffer[0] << 8) | i2cRxBuffer[1];
    humid_raw = (i2cRxBuffer[3] << 8) | i2cRxBuffer[4];

    *temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    *humidity = -6.0f + 125.0f * ((float)humid_raw / 65535.0f);

    if (*humidity > 100.0f) *humidity = 100.0f;
    if (*humidity < 0.0f) *humidity = 0.0f;

    return SUCCESS;
DMA_ERROR_NACK:
    printf("I2C ERROR\r\n");
    I2C_GenerateSTOP(SHT4x_I2C, ENABLE);
    I2C_AcknowledgeConfig(SHT4x_I2C, ENABLE);
    if (I2C_GetFlagStatus(SHT4x_I2C, I2C_FLAG_AF)) {
        I2C_ClearFlag(SHT4x_I2C, I2C_FLAG_AF);
    }
    while (I2C_GetFlagStatus(SHT4x_I2C, I2C_FLAG_BUSY));
    return ERROR;
}

ErrorStatus SHT4x_ReadHighPrecision(float *temperature, float *humidity)
{
    return SHT4x_ReadTempHumid_DMA(temperature, humidity, HIGH_PRECISION_CMD);
    // return SHT4x_ReadTempHumid(temperature, humidity, HIGH_PRECISION_CMD);
}

// int main(void)
// {
//     float temp, humid;
// 	LED_GPIO_Config();
// 	KEY_GPIO_Config();
// 	NVIC_SetPriority(EXTI15_10_IRQn, 0);
// 	NVIC_EnableIRQ(EXTI15_10_IRQn);
// 	SysTick->LOAD = (SystemCoreClock / 1000) - 1;
//     SysTick->VAL = 0;
//     SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
//                 SysTick_CTRL_TICKINT_Msk |
//                 SysTick_CTRL_ENABLE_Msk;

// 	EXTI_Config();
//     EXTI_MPU_Config();
//     I2cMaster_Init();
//     SHT4x_DMA_Init();
//     Usart_Config();
//     vTaskDelay(1000);

// }

// void vTaskDelay(__IO uint32_t nCount)
// {
//     s_timeLeft = nCount;
// 	for(; s_timeLeft != 0; );
// }


static void sysCoreTask( void * argument );

/*-----------------------------------------------------------*/

static void sysCoreTask( void * argument )
{
    float temp, humid;
    printf("sysCoreTask start\r\n");
    printf("start measure\r\n");
    while (1)
    {
        if(SHT4x_ReadHighPrecision(&temp, &humid) == SUCCESS) {
            printf("temp:%lf, humid:%lf \r\n", temp, humid);
            LED_GREEN;
        } else {
            printf("mesure faild\r\n");
            LED_BLUE;
        }
        printf("delay 1000ms\r\n");
        osDelay(1000);
        printf("sysCoreTask running\r\n");
    }
}

/*-----------------------------------------------------------*/

int main( void )
{
    SystemCoreClockUpdate();
    LED_GPIO_Config();
    KEY_GPIO_Config();
    NVIC_SetPriority(EXTI15_10_IRQn, 5);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    EXTI_Config();
    EXTI_MPU_Config();
    I2cMaster_Init();
    SHT4x_DMA_Init();
    Usart_Config();

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
