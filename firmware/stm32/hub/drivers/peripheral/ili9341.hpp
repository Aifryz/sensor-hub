#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#include <cstdint>

class ili9341_io_driver
{
    public:
    virtual void send_cmd(uint8_t* cmd, uint32_t len) = 0;
    virtual void send_data(uint8_t* data, uint32_t len) = 0;
};

class ili9341_lcd_driver
{
    public:
    ili9341_lcd_driver(ili9341_io_driver& driver);

    void init();
    void clear(uint16_t colour);
    void send_data(uint8_t* data, uint32_t len);
    void set_addr(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);

    private:
    inline void send_cmd_byte(uint8_t cmd)
    {
        m_io_driver.send_cmd(&cmd, 1);
    }
    ili9341_io_driver& m_io_driver;
};

#endif /* INC_ILI9341_H_ */
