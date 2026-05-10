#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "sensors/sensors_const.hpp"
#include "task.h"

#include "table_screen.hpp"

#include <lvgl.h>
#include <log.hpp>
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

class lcd_spi_driver : public ili9341_io_driver
{
    constexpr static size_t BLOCK_SIZE = 256;
  public:
    void send_cmd(uint8_t *cmd, uint32_t len) override
    {
        lcd_dcrs_pin::clear();
        lcd_cs_pin::clear();

        uint32_t rem = len;
        uint8_t *ptr = cmd;

        while (rem > BLOCK_SIZE)
        {
            lcd_spi.send(ptr, BLOCK_SIZE);
            rem -= BLOCK_SIZE;
            ptr += BLOCK_SIZE;
        }

        lcd_spi.send(ptr, rem);

        lcd_cs_pin::set();
    }

    void send_data(uint8_t *data, uint32_t len) override
    {
        lcd_dcrs_pin::set();
        lcd_cs_pin::clear();
        uint32_t rem = len;
        uint8_t *ptr = data;

        while (rem > BLOCK_SIZE)
        {
            lcd_spi.send(ptr, BLOCK_SIZE);
            rem -= BLOCK_SIZE;
            ptr += BLOCK_SIZE;
        }

        lcd_spi.send(ptr, rem);

        lcd_cs_pin::set();
    }
};

lcd_spi_driver lcd_io_driver;
ili9341_lcd_driver lcd_driver{lcd_io_driver};


/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
constexpr size_t lvgl_color_buf_size = LCD_WIDTH * 10; // Buffer for 10 lines of the display
static lv_color_t lvgl_color_buf[lvgl_color_buf_size];

/*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL instead buf_2 */

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    lcd_driver.set_addr(area->x1, area->y1, area->x2, area->y2);
    uint32_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    lcd_driver.send_data(reinterpret_cast<uint8_t*>(color_p), size * 2);

    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

static void lcd_demo()
{
    while(true)
    {
        
        lcd_driver.clear(0xFFFF); // White
        vTaskDelay(1000);
        lcd_driver.clear(0xF800); // Red
        vTaskDelay(1000);
        lcd_driver.clear(0x07E0); // Green
        vTaskDelay(1000);
        lcd_driver.clear(0x001F); // Blue
        vTaskDelay(1000);
    }
}

void lcd_task([[maybe_unused]] void* arg )
{
    vTaskDelay(100);
    logging::log("Starting LCD task \r\n");
    // Not sure if it helps, but maybe fast init causes radio to not work?
    vTaskDelay(5000);
    lcd_driver.init();
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
        
        vTaskDelay(delay);
        logging::log("Updating table data\r\n");
        updateTableData();
        // Update LVGL tick
        lv_tick_inc(delay);
        lv_timer_handler();
        logging::log("LVGL done\r\n");
    };
    
}

TaskHandle_t lcd_task_handle;
void init_lcd_display()
{
	xTaskCreate(&lcd_task, "lcd", 4096, NULL, 7, &lcd_task_handle);
}