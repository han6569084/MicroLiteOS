#include "ui_app.h"
#include "lvgl.h"
#include <stdio.h>
#include "stdlib.h"
#include "string.h"
#include "bsp_lcd.h"
#include "bsp_touch_gtxx.h"
#include "cmsis_os2.h"

static lv_draw_buf_t draw_buf;
static lv_color_t *lvgl_buf1 = (lv_color_t *)0xd0000000;
static lv_color_t *lvgl_buf2 = (lv_color_t *)0xd0200000;
static uint8_t s_fb_index = 0;
volatile uint8_t s_lcd_busy = 0;
lv_obj_t *label = NULL;

static void my_flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * color_p);
static void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data);
static void hal_init(void);
static void lvgl_update_cb(lv_timer_t * timer);

static void lvgl_tick_handler(void * arg)
{
    lv_tick_inc(5);
}

void ui_app_init(void)
{
    LCD_Init(LCD_BUFFER, 0, LTDC_PIXEL_FORMAT_RGB565);
    GTP_Init_Panel();
    LCD_BackLed_Control(ENABLE);
    lv_init();
    hal_init();

    osTimerAttr_t timer_attr = {
        .name = "lvgl_update_timer"
    };
    osTimerId_t timer_id = osTimerNew(lvgl_tick_handler, osTimerPeriodic, NULL, &timer_attr);
    osTimerStart(timer_id, 5);

    lv_obj_t *red_area = lv_obj_create(lv_scr_act());
    lv_obj_set_size(red_area, 800, 480);
    lv_obj_set_style_bg_color(red_area, lv_color_hex(0xFF0000), 0);
    lv_obj_center(red_area);

    label = lv_label_create(red_area);
    lv_label_set_text(label, "Hello LVGL!");
    lv_obj_center(label);

    lv_timer_t *timer = lv_timer_create(lvgl_update_cb, 1000, label);
    if(timer == NULL) {
        printf("lv_timer_create failed\r\n");
    }
    printf("ui_app_init done\r\n");
}

void ui_app_task_loop(void)
{
    while (1)
    {
        lv_timer_handler();
        osDelay(5);
    }
}

static void my_flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * color_p)
{
    while(s_lcd_busy) { osDelay(1); }
    s_lcd_busy = 1;
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
    lv_display_set_buffers(disp, lvgl_buf1, NULL, LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT * sizeof(lv_color16_t), LV_DISPLAY_RENDER_MODE_DIRECT);
    LCD_SetFrameBuffer((uint32_t)lvgl_buf2);
    lv_display_flush_ready(disp);
}

static void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    int x = 0, y = 0;
    GTP_Execu(&x, &y);
    if (x > 0 && y > 0) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void hal_init(void)
{
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

    lv_indev_t *touchpad_indev = lv_indev_create();
    if (touchpad_indev == NULL) {
        printf("lv_indev_create failed\r\n");
        return;
    }
    lv_indev_set_type(touchpad_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touchpad_indev, my_touchpad_read);
    lv_indev_set_mode(touchpad_indev, LV_INDEV_MODE_TIMER);
    lv_indev_enable(touchpad_indev, true);
    printf("lv_indev_create success\r\n");
}

static void lvgl_update_cb(lv_timer_t * timer)
{
    static uint32_t cnt = 0;
    static uint32_t color = 0xFF0000;
    lv_obj_t *red_area = lv_obj_get_parent(label);
    char buf[32];
    cnt++;
    uint32_t color_tmp = 0;
    switch(cnt / 20 % 3) {
        case 0: color_tmp = 0xFF0000; break;
        case 1: color_tmp = 0x00FF00; break;
        case 2: color_tmp = 0x0000FF; break;
    }
    if (color_tmp != color) {
        color = color_tmp;
        lv_obj_set_style_bg_color(red_area, lv_color_hex(color), 0);
    }
    snprintf(buf, sizeof(buf), "LVGL Timer: %lu", (unsigned long)cnt);
    lv_label_set_text(label, buf);
    lv_obj_center(label);
    LCD_SetBacklight(cnt % 10 * 10);
}
