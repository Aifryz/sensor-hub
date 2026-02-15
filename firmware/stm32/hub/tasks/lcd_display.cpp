#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "sensors/sensors_const.hpp"
#include "task.h"

#include <lvgl.h>

#include <drivers/mcu/spi.hpp>
#include <bsp/board_gpio.hpp>
#include <bsp/board_config.hpp>
#include <safebits.hpp>
#include <drivers/peripheral/ili9341.hpp>
#include <sensors/sensors_db.hpp>
#include <atomic>
#include <array>
#include <src/core/lv_obj.h>
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

void lv_example_create_table(void)
{
    //Create a table
    lv_obj_t * table = lv_table_create(lv_scr_act());
    lv_obj_set_size(table, 320, 240);
    lv_obj_center(table);

    //Set the number of rows and columns
    lv_table_set_row_cnt(table, 3);
    lv_table_set_col_cnt(table, 3);

    //Fill the first row
    lv_table_set_cell_value(table, 0, 0, "R1C1");
    lv_table_set_cell_value(table, 0, 1, "R1C2");
    lv_table_set_cell_value(table, 0, 2, "R1C3");

    //Fill the second row
    lv_table_set_cell_value(table, 1, 0, "R2C1");
    lv_table_set_cell_value(table, 1, 1, "R2C2");
    lv_table_set_cell_value(table, 1, 2, "R2C3");

    //Fill the third row
    lv_table_set_cell_value(table, 2, 0, "R3C1");
    lv_table_set_cell_value(table, 2, 1, "R3C2");
    lv_table_set_cell_value(table, 2, 2, "R3C3");
}

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
constexpr size_t lvgl_color_buf_size = LCD_WIDTH * 10; // Buffer for 10 lines of the display
static lv_color_t lvgl_color_buf[lvgl_color_buf_size];

/*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL instead buf_2 */

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    ILI9341_SetAddr(area->x1, area->y1, area->x2, area->y2);
    uint32_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    ILI9341_SendData(reinterpret_cast<uint8_t*>(color_p), size * 2);    

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

lv_obj_t* table;

static void create_table()
{
    table = lv_table_create(lv_scr_act());
    lv_obj_set_pos(table, 0, 25);
    lv_table_set_row_cnt(table, 5);
    lv_table_set_col_cnt(table, 6);
    lv_table_set_col_width(table, 0, 60);
    lv_table_set_col_width(table, 1, 60);
    lv_table_set_col_width(table, 2, 60);
    lv_table_set_col_width(table, 3, 40);
    lv_table_set_col_width(table, 4, 60);
    lv_table_set_col_width(table, 5, 40);

    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_width(cont, 320);
    lv_obj_set_height(cont, 25);

    static lv_style_t style_cont;
    lv_style_init(&style_cont);

    //lv_style_set_bg_color(&style_cont, white);
    lv_style_set_border_width(&style_cont, 1);
    //lv_style_set_border_color(&style_cont, gray);
    lv_style_set_pad_all(&style_cont, 2);
    lv_obj_add_style(cont, &style_cont, 0);

    static lv_style_t style_table_pad;
    lv_style_init(&style_table_pad);
    lv_style_set_pad_left(&style_table_pad, 2);
    lv_style_set_pad_right(&style_table_pad, 2);
    lv_obj_add_style(table, &style_table_pad, LV_PART_ITEMS);

    lv_obj_t *time_label = lv_label_create(cont);

    lv_label_set_text(time_label, "Time:");

    static const char *const locations[5] = {"Okno", "Dwor", "Balkon", "Pokoj",
                                             "Strych"};

    for (int i = 0; i < 5; i++)
    {
        lv_table_set_cell_value(table, i, 0, locations[i]);
    }
}

void format100(char* buf, int16_t val, const char* tag)
{
    uint32_t aval = abs(val);
    if (val >= 0)
    {
        sprintf(buf, "%d.%d%s", aval / 100, aval % 100, tag);
    }
    else
    {
        sprintf(buf, "-%d.%d%s", aval / 100, aval % 100, tag);
    }
}

void format1000(char *buf, int16_t val, const char* tag)
{
    uint32_t aval = abs(val);
    if (val >= 0)
    {
        sprintf(buf, "%d.%d%s", aval / 1000, aval % 1000, tag);
    }
    else
    {
        sprintf(buf, "-%d.%d%s", aval / 1000, aval % 1000, tag);
    }
}

void updateTableData(lv_obj_t *table)
{
    char str[32];
    for (int col = 0; col < 5; col++)
    {
        int node = col + 1;

        auto& sensor_db = GetSensorDB();

        auto temperature = sensor_db.GetLastMeasurement(node, MeasurementType::Temperature);
        auto humidity = sensor_db.GetLastMeasurement(node, MeasurementType::Humidity);
        auto light = sensor_db.GetLastMeasurement(node, MeasurementType::Light);
        auto battery = sensor_db.GetLastMeasurement(node, MeasurementType::Battery);


        if(temperature.has_value())
        {
            format100(str, temperature.value(), "C");
        }
        else {
            sprintf(str, "---");
        }
        lv_table_set_cell_value(table, col, 1, str);

        if(humidity.has_value())
        {
            format100(str, humidity.value(), "%");
        }
        else {
            sprintf(str, "---");
        }
        lv_table_set_cell_value(table, col, 2, str);
        
        if (light.has_value())
        {
           sprintf(str,"%d", light.value());
        }
        else
        {
            sprintf(str, "---");
        }
        lv_table_set_cell_value(table, col, 3, str);

        if (battery.has_value())
        {
            format1000(str, battery.value(), "V");
        }
        else
        {
            sprintf(str, "---");
        }
        lv_table_set_cell_value(table, col, 4, str);

    }
}

void lcd_task([[maybe_unused]] void* arg )
{
    ILI9341_Init();
    lv_init();
    //pwm :)
    //lcd_demo();
    lv_disp_draw_buf_init(&disp_buf, lvgl_color_buf, nullptr, lvgl_color_buf_size);

    static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
    lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
    disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
    disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
    disp_drv.hor_res = LCD_WIDTH;                 /*Set the horizontal resolution in pixels*/
    disp_drv.ver_res = LCD_HEIGHT;                 /*Set the vertical resolution in pixels*/
    disp_drv.color_format = LV_COLOR_FORMAT_NATIVE_REVERSE; // Reverse LSB/MSB order

    lv_disp_t * disp;
    disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/

    // update?
    

    //lv_example_get_started_1();
    //lv_example_create_table();

    create_table();

    

    const size_t delay = 20;
    while(1) {
        updateTableData(table);
        vTaskDelay(delay);
        // Update LVGL tick
        lv_tick_inc(delay);
        lv_timer_handler();
    };
    
}

TaskHandle_t lcd_task_handle;
void init_lcd_display()
{
	xTaskCreate(&lcd_task, "radio", 4096, NULL, 7, &lcd_task_handle);
}