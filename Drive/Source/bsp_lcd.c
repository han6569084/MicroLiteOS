/**
  ******************************************************************************
  * @file    bsp_lcd.c
  * @author  fire
  * @version V1.1
  * @date    2015-xx-xx
  * @brief   LCD驱动函数接口
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F429 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */


/**
  ******************************************************************************
  * @file    bsp_lcd.c
  * @author  fire
  * @version V1.1
  * @date    2015-xx-xx
  * @brief   LCD驱动函数接口
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 F429 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */

/*
 * ================== main.c flush_cb 配合说明 ==================
 * 1. LVGL 渲染始终写入“后备”Buffer（如 FRAMEBUFFER2_ADDR）。
 * 2. flush_cb 渲染完成后调用 LCD_SetFrameBuffer(后备Buffer地址)，通知LTDC下帧切换。
 * 3. LTDC 帧中断回调 LCD_FrameCallback()，LTDC 显示Buffer切换到后备Buffer。
 * 4. 下次LVGL渲染写入另一个Buffer，flush_cb后再切回。
 * 5. 如此循环，实现真正的无撕裂双缓冲。
 *
 * main.c 示例：
 *   static uint8_t cur_buf = 0;
 *   static void my_flush_cb(...) {
 *       uint32_t draw_addr = cur_buf ? LCD_GetFrameBuffer2() : LCD_GetFrameBuffer1();
 *       // LVGL 渲染到 draw_addr ...
 *       LCD_SetFrameBuffer(draw_addr);
 *       cur_buf = !cur_buf;
 *       lv_display_flush_ready(...);
 *   }
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_lcd.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_ltdc.h"
#include "stm32f4xx_hal_rcc_ex.h"
#include "stdio.h"

#define FRAMEBUFFER1_ADDR  ((uint32_t)0xD0000000)
#define FRAMEBUFFER2_ADDR  ((uint32_t)0xD0200000)

/* LTDC句柄 */
LTDC_HandleTypeDef hltdc;

extern volatile uint8_t s_lcd_busy;
// 获取FrameBuffer1/2地址，供应用层/flush_cb使用
uint32_t LCD_GetFrameBuffer1(void) { return FRAMEBUFFER1_ADDR; }
uint32_t LCD_GetFrameBuffer2(void) { return FRAMEBUFFER2_ADDR; }

// ================== LTDC_IRQHandler 示例实现 ==================
void LCD_FrameCallback(void);
// LTDC全局中断处理函数（建议放到 stm32f4xx_it.c 并在启动文件中声明）
void LTDC_IRQHandler(void)
{
    // 检查帧中断标志（Frame Interrupt）
    if(LTDC->ISR & LTDC_ISR_LIF) {
        LTDC->ICR = LTDC_ICR_CFUIF; // 清除帧中断标志
        s_lcd_busy = 0;
        LCD_FrameCallback();        // 帧切换
    }
    // 可选：处理其他中断标志
    if(LTDC->ISR & LTDC_ISR_FUIF) {
        LTDC->ICR = LTDC_ICR_CFUIF;
    }
    if(LTDC->ISR & LTDC_ISR_TERRIF) {
        LTDC->ICR = LTDC_ICR_CLIF;
    }
    if(LTDC->ISR & LTDC_ISR_LIF) {
        LTDC->ICR = LTDC_ICR_CLIF;
    }
}

/* 不同液晶屏的参数 */
const LCD_PARAM_TypeDef lcd_param[LCD_TYPE_NUM]={

  /* 5寸屏参数 */
  {
    /*根据液晶数据手册的参数配置*/
    .hbp = 46,  //HSYNC后的无效像素
    .vbp = 23,  //VSYNC后的无效行数

    .hsw = 1,  	//HSYNC宽度
    .vsw = 1,   //VSYNC宽度

    .hfp = 22,  	//HSYNC前的无效像素
    .vfp = 22,  	//VSYNC前的无效行数

    .comment_clock_2byte = 33, //rgb565/argb4444等双字节像素时推荐使用的液晶时钟频率
    .comment_clock_4byte = 21, //Argb8888等四字节像素时推荐使用的液晶时钟频率


    .lcd_pixel_width = LCD_MAX_PIXEL_WIDTH,//液晶分辨率，宽
    .lcd_pixel_height = LCD_MAX_PIXEL_HEIGHT,//液晶分辨率，高
  },

   /* 7寸屏参数（与5寸一样） */
  {
    /*根据液晶数据手册的参数配置*/
    .hbp = 46,  //HSYNC后的无效像素
    .vbp = 23,  //VSYNC后的无效行数

    .hsw = 1,  	//HSYNC宽度
    .vsw = 1,   //VSYNC宽度

    .hfp = 22,  	//HSYNC前的无效像素
    .vfp = 22,  	//VSYNC前的无效行数

    .comment_clock_2byte = 33, //rgb565/argb4444等双字节像素时推荐使用的液晶时钟频率
    .comment_clock_4byte = 21, //Argb8888等四字节像素时推荐使用的液晶时钟频率


    .lcd_pixel_width = LCD_MAX_PIXEL_WIDTH,//液晶分辨率，宽
    .lcd_pixel_height = LCD_MAX_PIXEL_HEIGHT,//液晶分辨率，高
  },

  /* 4.3寸屏参数 */
  {
      /*根据液晶数据手册的参数配置*/
    .hbp = 8,  //HSYNC后的无效像素
    .vbp = 2,  //VSYNC后的无效行数

    .hsw = 41,  	//HSYNC宽度
    .vsw = 10,   //VSYNC宽度

    .hfp = 4,  	//HSYNC前的无效像素
    .vfp = 4,  	//VSYNC前的无效行数

    .comment_clock_2byte = 15, //rgb565/argb4444等双字节像素时推荐使用的液晶时钟频率
    .comment_clock_4byte = 15, //Argb8888等四字节像素时推荐使用的液晶时钟频率

    .lcd_pixel_width = 480,//液晶分辨率，宽
    .lcd_pixel_height = 272,//液晶分辨率，高
  }
};

/* 当前使用的LCD，默认为5/7寸屏
  * 在触摸驱动初始化时可根据触摸芯片的型号驱分不同的LCD
*/
LCD_TypeDef cur_lcd = INCH_5;

/* 每个像素点占多少个字节
ARGB8888/RGB888/RGB565/ARGB1555/ARGB4444/L8/AL44/AL88
使用宏LTDC_PIXEL_FORMAT_ARGB8888作为索引即可
*/
const uint8_t PIXEL_BPP[]={4,3,2,2,2,1,1,2};

/**
  * @brief  初始化LCD的GPIO，RGB888及背光、DISP
  * @param  无
  * @retval 无
  */
void LCD_SetBacklight(uint8_t percent);

static void LCD_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 使能LCD使用到的引脚时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOJ_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();


/* GPIO配置 */

/* 红色数据线 */
    GPIO_InitStruct.Pin = LTDC_R0_GPIO_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = LTDC_R0_AF;

    HAL_GPIO_Init(LTDC_R0_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_R1_GPIO_PIN;
    HAL_GPIO_Init(LTDC_R1_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_R2_GPIO_PIN;
    HAL_GPIO_Init(LTDC_R2_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_R3_GPIO_PIN;
    HAL_GPIO_Init(LTDC_R3_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_R4_GPIO_PIN;
    HAL_GPIO_Init(LTDC_R4_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_R5_GPIO_PIN;
    HAL_GPIO_Init(LTDC_R5_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_R6_GPIO_PIN;
    HAL_GPIO_Init(LTDC_R6_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_R7_GPIO_PIN;
    HAL_GPIO_Init(LTDC_R7_GPIO_PORT, &GPIO_InitStruct);

    //绿色数据线
    GPIO_InitStruct.Pin = LTDC_G0_GPIO_PIN;
    HAL_GPIO_Init(LTDC_G0_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_G1_GPIO_PIN;
    HAL_GPIO_Init(LTDC_G1_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_G2_GPIO_PIN;
    HAL_GPIO_Init(LTDC_G2_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_G3_GPIO_PIN;
    HAL_GPIO_Init(LTDC_G3_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_G4_GPIO_PIN;
    HAL_GPIO_Init(LTDC_G4_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_G5_GPIO_PIN;
    HAL_GPIO_Init(LTDC_G5_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_G6_GPIO_PIN;
    HAL_GPIO_Init(LTDC_G6_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_G7_GPIO_PIN;
    HAL_GPIO_Init(LTDC_G7_GPIO_PORT, &GPIO_InitStruct);

    //蓝色数据线
    GPIO_InitStruct.Pin = LTDC_B0_GPIO_PIN;
    HAL_GPIO_Init(LTDC_B0_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_B1_GPIO_PIN;
    HAL_GPIO_Init(LTDC_B1_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_B2_GPIO_PIN;
    HAL_GPIO_Init(LTDC_B2_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_B3_GPIO_PIN;
    HAL_GPIO_Init(LTDC_B3_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_B4_GPIO_PIN;
    HAL_GPIO_Init(LTDC_B4_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_B5_GPIO_PIN;
    HAL_GPIO_Init(LTDC_B5_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_B6_GPIO_PIN;
    HAL_GPIO_Init(LTDC_B6_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_B7_GPIO_PIN;
    HAL_GPIO_Init(LTDC_B7_GPIO_PORT, &GPIO_InitStruct);

    //控制信号线
    GPIO_InitStruct.Pin = LTDC_CLK_GPIO_PIN;
    HAL_GPIO_Init(LTDC_CLK_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_HSYNC_GPIO_PIN;
    HAL_GPIO_Init(LTDC_HSYNC_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_VSYNC_GPIO_PIN;
    HAL_GPIO_Init(LTDC_VSYNC_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LTDC_DE_GPIO_PIN;
    HAL_GPIO_Init(LTDC_DE_GPIO_PORT, &GPIO_InitStruct);

    //BL DISP
    GPIO_InitStruct.Pin = LTDC_DISP_GPIO_PIN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    HAL_GPIO_Init(LTDC_DISP_GPIO_PORT, &GPIO_InitStruct);


    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

#if USE_TIM2_PWM_BACKLIGHT
    // 配置PB3为TIM2_CH2复用功能，输出PWM
    GPIO_InitStruct.Pin = LTDC_BL_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_Mode_AF;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LTDC_BL_GPIO_PORT, &GPIO_InitStruct);
    LCD_PWM_Backlight_Init();
#endif

    //拉高使能lcd
    HAL_GPIO_WritePin(LTDC_DISP_GPIO_PORT, LTDC_DISP_GPIO_PIN, GPIO_PIN_SET);
    // 背光初始由PWM控制

}

/**
 * @brief  背光LED控制
 * @param  on 1为亮，其余值为灭
 * @retval 无
 */
// 兼容旧接口，on=1为100%，on=0为0%
void LCD_BackLed_Control ( int on )
{
    if (on)
        LCD_SetBacklight(50);
    else
        LCD_SetBacklight(0);
}


/*
// TIM2_CH2 (PB3) 作为PWM背光输出
#define LCD_PWM_TIM                TIM2
#define LCD_PWM_TIM_CLK            RCC_APB1Periph_TIM2
#define LCD_PWM_TIM_CLK_INIT       RCC_APB1PeriphClockCmd
#define LCD_PWM_PERIOD             999
#define LCD_PWM_PRESCALER          83  // 84MHz/84=1MHz, 1MHz/1000=1kHz
#define LCD_PWM_CHANNEL            TIM_Channel_2

static void LCD_PWM_Backlight_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    LCD_PWM_TIM_CLK_INIT(LCD_PWM_TIM_CLK, ENABLE);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = LCD_PWM_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = LCD_PWM_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(LCD_PWM_TIM, &TIM_TimeBaseStructure);

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 50; // 默认50%亮度
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC2Init(LCD_PWM_TIM, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(LCD_PWM_TIM, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(LCD_PWM_TIM, ENABLE);
    TIM_Cmd(LCD_PWM_TIM, ENABLE);
}
*/

// 设置背光亮度百分比（0~100）
void LCD_SetBacklight(uint8_t percent)
{
#if USE_TIM2_PWM_BACKLIGHT
    if(percent > 100) percent = 100;
    uint16_t ccr = (LCD_PWM_PERIOD + 1) * percent / 100;
    TIM_SetCompare2(LCD_PWM_TIM, ccr);
#else
    // 直接控制GPIO输出高低电平
    if(percent > 0) {
        HAL_GPIO_WritePin(LTDC_BL_GPIO_PORT, LTDC_BL_GPIO_PIN, GPIO_PIN_SET); // 背光亮
    } else {
        HAL_GPIO_WritePin(LTDC_BL_GPIO_PORT, LTDC_BL_GPIO_PIN, GPIO_PIN_RESET); // 背光灭
    }
#endif
}

/**
  * @brief 初始化LTDC的 层 参数
  * @param fb_addr 显存首地址
  * @param pixel_format 像素格式，
           如LTDC_PIXEL_FORMAT_ARGB8888 、LTDC_PIXEL_FORMAT_RGB565等
  * @retval None
  */
static uint32_t s_lcd_fb_addr = 0; // 当前显示的FrameBuffer地址
static uint32_t s_lcd_fb_addr_next = 0; // 下一个待切换的FrameBuffer地址

// 切换FrameBuffer地址，下一帧生效
void LCD_SetFrameBuffer(uint32_t fb_addr)
{
    s_lcd_fb_addr_next = fb_addr;
}

// 在LTDC帧中断中调用，完成FrameBuffer切换
void LCD_FrameCallback(void)
{
    // printf("LCD_FrameCallback\r\n");
    // 如果有下一个待
    if(s_lcd_fb_addr_next && s_lcd_fb_addr_next != s_lcd_fb_addr) {
        LTDC_Layer1->CFBAR = s_lcd_fb_addr_next;
        HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_VERTICAL_BLANKING); // VBlank期间切换
        s_lcd_fb_addr = s_lcd_fb_addr_next;
    }
}

void LCD_LayerInit(uint32_t fb_addr, uint32_t pixel_format )
{
    LTDC_LayerCfgTypeDef LTDC_Layer_InitStruct;

    /* 层窗口配置 */
    /* 配置本层的窗口边界，注意这些参数是包含HBP HSW VBP VSW的 */
    //一行的第一个起始像素，该成员值应用为 (LTDC_InitStruct.LTDC_AccumulatedHBP+1)的值
    LTDC_Layer_InitStruct.WindowX0 = HBP + HSW;
    //一行的最后一个像素，该成员值应用为 (LTDC_InitStruct.LTDC_AccumulatedActiveW)的值
    LTDC_Layer_InitStruct.WindowX1 = HSW+HBP+LCD_PIXEL_WIDTH-1;
    //一列的第一个起始像素，该成员值应用为 (LTDC_InitStruct.LTDC_AccumulatedVBP+1)的值
    LTDC_Layer_InitStruct.WindowY0 =  VBP + VSW;
    //一列的最后一个像素，该成员值应用为 (LTDC_InitStruct.LTDC_AccumulatedActiveH)的值
    LTDC_Layer_InitStruct.WindowY1 = VSW+VBP+LCD_PIXEL_HEIGHT-1;

    /* 像素格式配置*/
    LTDC_Layer_InitStruct.PixelFormat = pixel_format;
    /* 恒定Alpha值配置，0-255 */
    LTDC_Layer_InitStruct.Alpha = 255;
    /* 默认背景颜色，该颜色在定义的层窗口外或在层禁止时使用。 */
    LTDC_Layer_InitStruct.Backcolor.Blue = 0xFF;
    LTDC_Layer_InitStruct.Backcolor.Green = 0xFF;
    LTDC_Layer_InitStruct.Backcolor.Red = 0xFF;
    LTDC_Layer_InitStruct.Alpha0 = 0xFF;
    /* 配置混合因子 CA表示使用恒定Alpha值，PAxCA表示使用像素Alpha x 恒定Alpha值 */
    LTDC_Layer_InitStruct.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    LTDC_Layer_InitStruct.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;

    /* 配置有效的行数 */
    LTDC_Layer_InitStruct.ImageHeight = LCD_PIXEL_HEIGHT;
    LTDC_Layer_InitStruct.ImageWidth = LCD_PIXEL_WIDTH;

    /* 配置本层的显存首地址 */
    LTDC_Layer_InitStruct.FBStartAdress = fb_addr;
    s_lcd_fb_addr = fb_addr;
    s_lcd_fb_addr_next = fb_addr;

    /* 以上面的配置初始化第 1 层*/
    HAL_LTDC_ConfigLayer(&hltdc, &LTDC_Layer_InitStruct, 0);

    /* 立即重载配置 */
    HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);

  /*只使用一层 */
    __HAL_LTDC_LAYER_ENABLE(&hltdc, 0);
    __HAL_LTDC_LAYER_DISABLE(&hltdc, 1);

    /* 立即重载配置 */
    HAL_LTDC_Reload(&hltdc, LTDC_RELOAD_IMMEDIATE);
}

/**
  * @brief LCD初始化
  * @param fb_addr 显存首地址
  * @param  lcd_clk_mhz 像素时钟频率，为 0 时直接使用推荐时钟频率
            RGB565格式推荐为30~33，
            XRGB8888格式推荐为20~22
            极限范围为15~52，其余值会超出LTDC时钟分频配置范围
  * @param pixel_format 像素格式，如LTDC_PIXEL_FORMAT_ARGB8888 、LTDC_PIXEL_FORMAT_RGB565等
  * @retval  None
  */
// LCD初始化，fb_addr为初始FrameBuffer
void LCD_Init(uint32_t fb_addr, int lcd_clk_mhz, uint32_t pixel_format )
// 用户需在中断文件 stm32f4xx_it.c 中的 LTDC_IRQHandler 或 DMA2D/LTDC 帧中断中调用 LCD_FrameCallback()
{
  uint32_t div;

  /* lcd_clk_mhz为0时使用推荐时钟频率 */
  if(lcd_clk_mhz == 0)
  {
    if(pixel_format == LTDC_PIXEL_FORMAT_RGB565||
        pixel_format == LTDC_PIXEL_FORMAT_ARGB1555||
        pixel_format == LTDC_PIXEL_FORMAT_ARGB4444||
        pixel_format == LTDC_PIXEL_FORMAT_L8||
        pixel_format == LTDC_PIXEL_FORMAT_AL88)
    {
      lcd_clk_mhz = lcd_param[cur_lcd].comment_clock_2byte;
    }
    else if(pixel_format == LTDC_PIXEL_FORMAT_ARGB8888||
              pixel_format == LTDC_PIXEL_FORMAT_RGB888)
    {
      lcd_clk_mhz = lcd_param[cur_lcd].comment_clock_4byte;
    }
  }

  /* 使能LTDC外设时钟 */
  __HAL_RCC_LTDC_CLK_ENABLE();

  /* 使能DMA2D时钟 */
  __HAL_RCC_DMA2D_CLK_ENABLE();

  /* 初始化LCD的控制引脚 */
  LCD_GPIO_Config();

  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  /* LCD时钟配置 */
  /* PLLSAI_VCO 输入时钟 = HSE_VALUE/PLLM = 1 Mhz */
  /* PLLSAI_VCO 输出时钟 = PLLSAI_VCO输入 * PLLSAIN = 420 Mhz */
  /* PLLLCDCLK = PLLSAI_VCO 输出/PLLSAIR = 420/div  Mhz */
  /* LTDC 时钟频率 = PLLLCDCLK / DIV = 420/div/4 = PIXEL_CLK_MHZ Mhz */
  /* LTDC时钟太高会导花屏，若对刷屏速度要求不高，降低时钟频率可减少花屏现象*/
  div = 420/4/lcd_clk_mhz;

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 420;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = div;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* LTDC配置*********************************************************/
  hltdc.Instance = LTDC;

  /*信号极性配置*/
  /* 行同步信号极性 */
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  /* 垂直同步信号极性 */
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  /* 数据使能信号极性 */
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  /* 像素同步时钟极性 */
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;

  /* 配置LCD背景颜色 */
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;

  /* 时间参数配置 */
  /* 配置行同步信号宽度(HSW-1) */
  hltdc.Init.HorizontalSync = HSW-1;
  /* 配置垂直同步信号宽度(VSW-1) */
  hltdc.Init.VerticalSync = VSW-1;
  /* 配置(HSW+HBP-1) */
  hltdc.Init.AccumulatedHBP = HSW+HBP-1;
  /* 配置(VSW+VBP-1) */
  hltdc.Init.AccumulatedVBP = VSW+VBP-1;
  /* 配置(HSW+HBP+有效像素宽度-1) */
  hltdc.Init.AccumulatedActiveW = HSW+HBP+LCD_PIXEL_WIDTH-1;
  /* 配置(VSW+VBP+有效像素高度-1) */
  hltdc.Init.AccumulatedActiveH = VSW+VBP+LCD_PIXEL_HEIGHT-1;
  /* 配置总宽度(HSW+HBP+有效像素宽度+HFP-1) */
  hltdc.Init.TotalWidth = HSW+ HBP+LCD_PIXEL_WIDTH  + HFP-1;
  /* 配置总高度(VSW+VBP+有效像素高度+VFP-1) */
  hltdc.Init.TotalHeigh = VSW+ VBP+LCD_PIXEL_HEIGHT  + VFP-1;

  HAL_LTDC_Init(&hltdc);

  LCD_LayerInit(fb_addr, pixel_format);

  // 使能LTDC帧中断（LIF）
  LTDC->IER |= LTDC_IER_LIE;
  NVIC_EnableIRQ(LTDC_IRQn);
}

/*
void LCD_LayerCamInit(uint32_t Addr, uint32_t width, uint32_t high)
{
    // TODO: Convert this function to HAL library
    // This function uses LTDC_Layer_InitTypeDef which is deprecated in HAL library
    // Need to rewrite using HAL_LTDC_ConfigLayer and LTDC_LayerCfgTypeDef
}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
