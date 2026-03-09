#include "FreeRTOS.h"
#include "io_expander.hpp"

//#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include <log.hpp>
#include "i2c.h"
#include "stm32f4xx_hal_i2c.h"

class encoder
{
    public:

    enum class InputState: uint8_t
    {
        A0B0 = 0b00,
        A0B1 = 0b01,
        A1B0 = 0b10,
        A1B1 = 0b11,
        None = 0xFF
    };

    int32_t get_ticks()
    {
        return m_ticks/4;
    }

    int32_t get_full_ticks()
    {
        return m_ticks;
    }

    void update_state(InputState state)
    {
        if(state == m_last_state)
            return;

        if(state == InputState::None)
        {
            m_last_state = state;
            return;
        }
        else
        {
            // State transition table for quadrature encoder
            // Each state is represented as a 2-bit value (A and B)
            // The table indicates the change in ticks for each transition
            static const int8_t transition_table[4][4] = {
                { 0, -1,  1,  0}, // From A0B0
                { 1,  0,  0, -1}, // From A0B1
                {-1,  0,  0,  1}, // From A1B0
                { 0,  1, -1,  0}  // From A1B1
            };

            uint8_t last_state_index = static_cast<uint8_t>(m_last_state);
            uint8_t new_state_index = static_cast<uint8_t>(state);

            m_ticks += transition_table[last_state_index][new_state_index];
            m_last_state = state;
        }
    }

    private:
    InputState m_last_state = InputState::None;
    int32_t m_ticks;
    
};
encoder enc;
uint32_t enc_up_tick;
void io_expander_task([[maybe_unused]] void* arg)
{
    vTaskDelay(100);
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
        result = HAL_I2C_Mem_Read(&hi2c1, pcal_address, input_port, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
        if(result != HAL_OK)
        {
            logging::log("Error reading from IO expander: {}\r\n", result);
        }
        else if(buf[0] != last_value)
        {
            //logging::log("PCAL input changed: {}\r\n", (int)buf[0]);
            last_value = buf[0];
            uint8_t ab_state = buf[0] >> 1;
            ab_state = ab_state&0x03;

            bool a = (ab_state & 0x01) != 0;
            bool b = (ab_state & 0x02) != 0;
            //logging::log("PCAL input changed: A {}, B {}\r\n", (int)a, (int)b);
            
            //logging::log("PCAL input changed: {}\r\n", (int)ab_state);

            encoder::InputState state = static_cast<encoder::InputState>(ab_state); // Assuming A is bit 0 and B is bit 1
            enc.update_state(state);
            logging::log("Encoder ticks: {}, full ticks: {}\r\n", (int)enc.get_ticks(), (int)enc.get_full_ticks());
            
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