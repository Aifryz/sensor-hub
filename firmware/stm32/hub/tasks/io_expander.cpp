#include "FreeRTOS.h"
#include "io_expander.hpp"

//#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_tim.h"
#include "task.h"
#include <log.hpp>
#include <drivers/mcu/i2c.hpp>
#include "stm32f4xx_hal_i2c.h"
#include "tim.h"

#include <drivers/peripheral/encoder.hpp>

namespace logging
{
    namespace impl
    {
        template<>
		inline const char* log_var(const char* spec, HAL_StatusTypeDef var)
		{
            
            switch(var)
            {
                case HAL_OK:
                    log_part("HAL_OK", 0, 6);
                    break;
                case HAL_ERROR:
                    log_part("HAL_ERROR", 0, 9);
                    break;
                case HAL_BUSY:
                    log_part("HAL_BUSY", 0, 8);
                    break;
                case HAL_TIMEOUT:
                    log_part("HAL_TIMEOUT", 0, 11);
                    break;
                default:
                    log_part("UNKNOWN", 0, 7);
            }

			const char* fmt_end = spec;
			while (*fmt_end != '\0')
			{
				if(*fmt_end == '}' && *(fmt_end+1) != '}'){
					fmt_end++;
					break;
				}
				fmt_end++;
			}
			return fmt_end;
            
		}
    }

}

uint32_t enc_up_tick;
uint32_t last_update_tick;
void io_expander_task([[maybe_unused]] void* arg)
{
    vTaskDelay(1000);
    logging::log("Starting IO Expander task \r\n");
    
    // For now, simple readout to make sure that everything on the board works ok,
    // Ideally this would be called exactly every 1ms, e.g. via timer
    // But we can improve this later

    

    uint8_t buf[8];
    
    
    const size_t delay = 1;
    uint8_t last_value = 0;
    while(1) {
        // Add IO Expander operations here

        uint8_t pcal_address = 0x40;
        uint8_t input_port = 0x00;
        HAL_StatusTypeDef result = HAL_OK;
        enc_up_tick = HAL_GetTick()%4096;

        auto i2c_session = board_i2c.start_session();
        auto i2c_result = i2c_session.mem_read(pcal_address, input_port, buf, 1);
        if(i2c_result != i2c_error::None)
        {
            // for some reason, the PCAL gives error after some time
            // so it works, but after some time we get 2 fails
            // then it again works, probably initialization issue, let's ignore it for now
            logging::log("Error reading from IO expander: {}\r\n", (int)i2c_result);
        }        
        else if(buf[0] != last_value)
        {
            //logging::log("PCAL input changed: {}\r\n", (int)buf[0]);
            last_value = buf[0];
            uint8_t ab_state = buf[0] >> 1;
            ab_state = ab_state&0x03;

            bool a = (ab_state & 0x01) != 0;
            bool b = (ab_state & 0x02) != 0;
            bool btn = (buf[0] & 0x01) != 0;

            last_update_tick = HAL_GetTick();
            
            // 0-3 GPIO, a,b, BUTTON, spare
            // 4 - card det
            // 5,6,7 - led
            
            //logging::log("PCAL input changed: A {}, B {}\r\n", (int)a, (int)b);
            
            //logging::log("PCAL input changed: {}\r\n", (int)ab_state);

            encoder::InputState state = static_cast<encoder::InputState>(ab_state); // Assuming A is bit 0 and B is bit 1
            enc.update_state(state);
            logging::log("Encoder ticks: {}, full ticks: {}\r\n", (int)enc.get_ticks(), (int)enc.get_full_ticks());
            
        }

        {
            uint32_t now = HAL_GetTick();
            uint32_t delay_ms = 5000; // 5 seconds of inactivity before reset
            if(now - last_update_tick > delay_ms)
            {
                //htim2.Instance->CCR3 = 50; // Set to low brightness to indicate inactivity
            }
            else
            {
                htim2.Instance->CCR3 = 1000; // Set to high brightness to indicate activity
            }
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