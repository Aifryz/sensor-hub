#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include <lvgl.h>

#include <drivers/mcu/spi.hpp>
#include <bsp/board_gpio.hpp>
#include <safebits.hpp>
#include <drivers/peripheral/ili9341.hpp>
#include <atomic>
#include <array>
#include <src/hal/lv_hal_tick.h>
#include <src/misc/lv_color.h>
#include <src/misc/lv_timer.h>

static void lcd_demo()
{
    while(true)
    {
        
        ILI9341_Clear(0xFFFF); // White
        vTaskDelay(1000);
        ILI9341_Clear(0xF800); // Red
        vTaskDelay(1000);
        ILI9341_Clear(0x07E0); // Green
        vTaskDelay(1000);
        ILI9341_Clear(0x001F); // Blue
        vTaskDelay(1000);
    }
}

void lv_example_get_started_1(void)
{
    //Change the active screen's background color
    //lv_scr_act();
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x003a57), LV_PART_MAIN);

    //Create a white label, set its text and align it to the center
    lv_obj_t * label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello world");
    lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

#define LCD_W 240
#define LCD_H 320

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LCD_W * 10];
//static lv_color_t buf_2[LCD_W * 10];

/*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL instead buf_2 */

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one
     *`put_px` is just an example, it needs to implemented by you.*/
    /*int32_t x, y;
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
            //put_px(x, y, *color_p);
            ILI9341_WrPixel(x, y, *color_p);
            color_p++;
        }
    }
        */
    ILI9341_SetAddr(area->x1, area->y1, area->x2, area->y2);
    uint32_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    ILI9341_SendData(reinterpret_cast<uint8_t*>(color_p), size * 2);    

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}


void lcd_task([[maybe_unused]] void* arg )
{
    ILI9341_Init();
    lv_init();
    //pwm :)
    //lcd_demo();
    lv_disp_draw_buf_init(&disp_buf, buf_1, nullptr, LCD_W * 10);

    static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
    lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
    disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
    disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
    disp_drv.hor_res = LCD_W;                 /*Set the horizontal resolution in pixels*/
    disp_drv.ver_res = LCD_H;                 /*Set the vertical resolution in pixels*/

    disp_drv.color_format = LV_COLOR_FORMAT_NATIVE_REVERSE; // ?

    lv_disp_t * disp;
    disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/

    

    lv_example_get_started_1();

    while(1) {
        vTaskDelay(1);;
        lv_tick_inc(1);
        lv_timer_handler(); // ?? 
    };
    
}

TaskHandle_t lcd_task_handle;
void init_lcd_display()
{
	BaseType_t ret = xTaskCreate(&lcd_task, "radio", 512, NULL, 7, &lcd_task_handle);
}