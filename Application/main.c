#include "stm32f4xx.h"
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
#include "lvgl.h"
#include "bsp_sdram.h"
#include "bsp_lcd.h"
#include <string.h>
#include "../lvgl/demos/widgets/lv_demo_widgets.h"
#include "bsp_touch_gtxx.h"
#include "stm32f4xx_dma2d.h"
#include "misc.h"
#include "../lvgl/src/draw/dma2d/lv_draw_dma2d.h"
#include "cm_backtrace.h"

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
        case HIGH_PRECISION_CMD: osDelay(9); break;    //
        case MEDIUM_PRECISION_CMD: osDelay(5); break;   //
        case LOW_PRECISION_CMD: osDelay(2); break;      //
        default: osDelay(10); break;
    }
    // osDelay(8000);
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
        case HIGH_PRECISION_CMD: osDelay(9); break;    // 8.2ms
        case MEDIUM_PRECISION_CMD: osDelay(5); break;  // 4.5ms
        case LOW_PRECISION_CMD: osDelay(2); break;     // 1.5ms
        default: osDelay(10); break;
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
        osDelay(1);
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


static lv_draw_buf_t draw_buf;
// 真双缓冲 FrameBuffer 地址由驱动统一定义和管理，直接通过 getter 获取
// FrameBuffer1/2 地址定义见 bsp_lcd.c
static lv_color_t *lvgl_buf1 = (lv_color_t *)0xd0000000; // 备份/渲染 Buffer
static lv_color_t *lvgl_buf2 = (lv_color_t *)0xd0200000; // 显示 Buffer

// 交替渲染用静态变量
static uint8_t s_fb_index = 0;
volatile uint8_t s_lcd_busy = 0;
/*
 * 真正无撕裂双缓冲 LVGL flush_cb 实现：
 * 1. LVGL 渲染始终写入当前“备份”Buffer（lvgl_buf1），显示端始终显示“主”Buffer（lvgl_buf2）。
 * 2. 每次 flush_cb 被 LVGL 调用时，交替切换 lvgl_buf1/lvgl_buf2 指针（即 FrameBuffer1/2 角色），
 *    并调用 LCD_SetFrameBuffer() 通知驱动切换物理地址，等待 LTDC 帧中断完成物理切换。
 * 3. flush_cb 只需调用 lv_display_flush_ready()，无需 memcpy，驱动帧中断自动切换显示。
 * 4. 需保证 FrameBuffer1/2 地址与驱动一致，且 LCD_SetFrameBuffer/FrameCallback 正确实现。
 * 5. 典型流程：
 *    - LVGL 渲染到“备份”Buffer（lvgl_buf1）
 *    - flush_cb 交替切换 lvgl_buf1/lvgl_buf2，调用 LCD_SetFrameBuffer(显示 Buffer)
 *    - LTDC 帧中断回调 FrameCallback，通知 LVGL 渲染下一帧
 */
static void my_flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * color_p)
{
    // if(area->x1 == 0 && area->y1 == 0 && area->x2 == LCD_PIXEL_WIDTH-1 && area->y2 == LCD_PIXEL_HEIGHT-1) {
    // }
    // else {
    //     return;
    // }
    while(s_lcd_busy) { osDelay(1); }
    s_lcd_busy = 1;
    // 只支持全屏刷新，area 检查可选
    // 交替切换 FrameBuffer1/2
    s_fb_index ^= 1;
    extern uint32_t LCD_GetFrameBuffer1(void);
    extern uint32_t LCD_GetFrameBuffer2(void);
    extern void LCD_SetFrameBuffer(uint32_t fb_addr);
    if(s_fb_index == 0) {
        lvgl_buf1 = (lv_color_t *)LCD_GetFrameBuffer1();
        lvgl_buf2 = (lv_color_t *)LCD_GetFrameBuffer2();
    } else {
        lvgl_buf1 = (lv_color_t *)LCD_GetFrameBuffer2();
        lvgl_buf2 = (lv_color_t *)LCD_GetFrameBuffer1();
    }
    memcpy(lvgl_buf1, lvgl_buf2, LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT * sizeof(lv_color16_t));
    // 每次切换后，重新设置 LVGL 渲染 Buffer，确保双缓冲生效
    lv_display_set_buffers(disp, lvgl_buf1, NULL, LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT * sizeof(lv_color16_t), LV_DISPLAY_RENDER_MODE_DIRECT);
    // 通知驱动切换显示 Buffer
    LCD_SetFrameBuffer((uint32_t)lvgl_buf2);
    // LVGL 渲染下一帧前，flush_ready
    lv_display_flush_ready(disp);
}

static void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    volatile int x = 0, y = 0;
    // printf("my_touchpad_read called\r\n");
    GTP_Execu(&x, &y);
    if (x > 0 && y > 0) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
    // printf("Touch at (%d, %d), state: %d\r\n", data->point.x, data->point.y, data->state);
}

static void hal_init(void)
{
    // 使用双FrameBuffer，lvgl_buf1为渲染Buffer，lvgl_buf2为显示Buffer
    lv_draw_buf_init(
        &draw_buf,
        LCD_PIXEL_WIDTH,
        LCD_PIXEL_HEIGHT,
        LV_COLOR_FORMAT_NATIVE,
        LCD_PIXEL_WIDTH,
        lvgl_buf1,
        LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT * sizeof(lv_color16_t)
    );

    lv_display_t *disp = lv_display_create(LCD_PIXEL_WIDTH, LCD_PIXEL_HEIGHT);
    lv_display_set_default(disp);
    lv_display_set_flush_cb(disp, my_flush_cb);
    lv_display_set_buffers(disp, lvgl_buf1, NULL, LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT * sizeof(lv_color16_t), LV_DISPLAY_RENDER_MODE_DIRECT);
    lv_display_set_render_mode(disp, LV_DISPLAY_RENDER_MODE_DIRECT);

    // lv_indev_t *touchpad_indev = lv_indev_create();
    // if (touchpad_indev == NULL) {
    //     printf("lv_indev_create failed\r\n");
    //     return;
    // }

    // lv_indev_set_type(touchpad_indev, LV_INDEV_TYPE_POINTER);
    // lv_indev_set_read_cb(touchpad_indev, my_touchpad_read);

    // lv_indev_set_mode(touchpad_indev, LV_INDEV_MODE_TIMER);
    // lv_indev_enable(touchpad_indev, true);
    printf("lv_indev_create success\r\n");
}

void lvglTimerCb (void *argument)
{
    lv_tick_inc(10);
}

lv_obj_t *label = NULL;
// Timer callback to update label text and red area color
static void lvgl_update_cb(lv_timer_t * timer)
{
    static uint32_t cnt = 0;
    static uint32_t color = 0xFF0000;
    lv_obj_t *red_area = lv_obj_get_parent(label);
    char buf[32];
    cnt++;
    // Cycle color: red -> green -> blue -> red ...
    uint32_t color_tmp = 0;
    switch(cnt / 20 % 3) {
        case 0: color_tmp = 0xFF0000; break;
        case 1: color_tmp = 0x00FF00; break;
        case 2: color_tmp = 0x0000FF; break;
    }
    // color_tmp = 0xFFFFFF;
    if (color_tmp != color) {
        color = color_tmp;
        lv_obj_set_style_bg_color(red_area, lv_color_hex(color), 0);
    }

    snprintf(buf, sizeof(buf), "LVGL Timer: %lu", (unsigned long)cnt);
    lv_label_set_text(label, buf);
    lv_obj_center(label);

    LCD_SetBacklight(cnt % 10 * 10); // 模拟背光亮度变化
}


static void lvglGuiTask( void * argument )
{
    LCD_Init(LCD_BUFFER, 0, LTDC_Pixelformat_RGB565);
    GTP_Init_Panel();

    LCD_BackLed_Control(ENABLE);
    /*Initialize LittlevGL*/
    lv_init();

    /*Initialize the HAL (display, input devices, tick) for LittlevGL*/
    hal_init();

    // 创建800x400红色区域
    lv_obj_t *red_area = lv_obj_create(lv_scr_act());
    lv_obj_set_size(red_area, 800, 480);
    lv_obj_set_style_bg_color(red_area, lv_color_hex(0xFF0000), 0);
    lv_obj_center(red_area);

    // 创建label并居中于红色区域
    label = lv_label_create(red_area);
    lv_label_set_text(label, "Hello LVGL!");
    lv_obj_center(label);

    // 创建LVGL定时器，每秒更新label和颜色
    lv_timer_t *timer = lv_timer_create(lvgl_update_cb, 1000, label);
    if(timer == NULL) {
        printf("lv_timer_create failed\r\n");
    }

    // lv_demo_widgets();

    printf("lvglGuiTask start\r\n");
    while (1)
    {
        /*Call the lv_task handler periodically*/
        lv_timer_handler();
        osDelay(10);
    }
}
float g_temp = 0.0f;
float g_humid = 0.0f;
int   g_posx = 0;
int   g_posy = 0;
static void sysCoreTask( void * argument )
{
    float temp, humid;
    printf("sysCoreTask start\r\n");

    osThreadAttr_t attr_thread = {
        .name = "lvglGuiTask",
        .priority = osPriorityHigh,
        .stack_size = 40*1024
    };
    osThreadNew(lvglGuiTask, NULL, &attr_thread);

    osTimerAttr_t attr_timer = {
        .name = "lvglTimer",
    };
    osTimerId_t lvglTimer = osTimerNew(lvglTimerCb, osTimerPeriodic, NULL, &attr_timer);
    if (lvglTimer == NULL) {
        printf("osTimerNew failed\r\n");
    } else {
        osTimerStart(lvglTimer, 10);
        printf("osTimerNew success\r\n");
    }

    printf("start measure\r\n");
    while (1)
    {
        // if(SHT4x_ReadHighPrecision(&temp, &humid) == SUCCESS) {
        //     printf("temp:%lf, humid:%lf \r\n", temp, humid);
        //     g_temp = temp;
        //     g_humid = humid;
        //     LED_GREEN;
        // } else {
        //     printf("mesure faild\r\n");
        //     LED_BLUE;
        // }
        // int x = 0, y = 0;
        // GTP_Execu(&x, &y);
        // g_posx = x;
        // g_posy = y;
        // if (x > 0 && y > 0) {
        //     printf("touch at (%d, %d)\r\n", x, y);
        // }
        // printf("delay 50ms\r\n");
        osDelay(50);
        // printf("sysCoreTask running\r\n");
    }
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

    printf("DMA2D_IRQn = %d, IP = %d\r\n", DMA2D_IRQn, NVIC_GetPriority(DMA2D_IRQn));
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
