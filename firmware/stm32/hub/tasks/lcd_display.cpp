#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include <drivers/mcu/spi.hpp>
#include <bsp/board_gpio.hpp>
#include <safebits.hpp>
#include <drivers/peripheral/ili9341.hpp>
#include <atomic>
#include <array>

void lcd_task([[maybe_unused]] void* arg )
{
    ILI9341_Init();
    //pwm :)

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

TaskHandle_t lcd_task_handle;
void init_lcd_display()
{
	BaseType_t ret = xTaskCreate(&lcd_task, "radio", 512, NULL, 7, &lcd_task_handle);
}