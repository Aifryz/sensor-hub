#include "FreeRTOS.h"
#include "io_expander.hpp"

//#include "stm32f4xx_hal_def.h"
#include "task.h"
#include <log.hpp>
#include "i2c.h"
#include "stm32f4xx_hal_i2c.h"

void io_expander_task([[maybe_unused]] void* arg)
{
    vTaskDelay(100);
    logging::log("Starting IO Expander task \r\n");
    
    // For now, simple readout to make sure that everything on the board works ok,
    // Ideally this would be called exactly every 1ms, e.g. via timer
    // But we can improve this later

    uint8_t buf[8];
    
    
    const size_t delay = 10;
    uint8_t last_value = 0;
    while(1) {
        // Add IO Expander operations here

        uint8_t pcal_address = 0x40;
        uint8_t input_port = 0x00;
        HAL_StatusTypeDef result = HAL_OK;
        result = HAL_I2C_Mem_Read(&hi2c1, pcal_address, input_port, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
        if(result != HAL_OK)
        {
            logging::log("Error reading from IO expander: {}\r\n", result);
        }
        else if(buf[0] != last_value)
        {
            logging::log("PCAL input changed: {}\r\n", (int)buf[0]);
            last_value = buf[0];
        }
        
        vTaskDelay(delay);
    }
}

TaskHandle_t io_expander_task_handle;
void init_io_expander()
{
    // Need to have higher priority, since it expects to be called periodically
    xTaskCreate(&io_expander_task, "io_expander", 2048, NULL, 8, &io_expander_task_handle);
}