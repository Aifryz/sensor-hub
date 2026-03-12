#include "ili9341.hpp"

#include <bsp/board_gpio.hpp>
#include <bsp/board_config.hpp>
#include <drivers/mcu/spi.hpp>

ili9341_lcd_driver::ili9341_lcd_driver(ili9341_io_driver& driver) 
  : m_io_driver(driver) {}

void ili9341_lcd_driver::init()
{
    //gpio_set_level(LCD_CS_PIN, LOW);
	lcd_cs_pin::clear();
	
	//gpio_set_level(RESET_PIN, LOW); // Pull reset low
	sys_reset_pin::clear();
	vTaskDelay(5/ portTICK_PERIOD_MS);
	//gpio_set_level(RESET_PIN, HIGH); // Pull reset low
	sys_reset_pin::set();
	vTaskDelay(150/ portTICK_PERIOD_MS);

	uint8_t tx[16];
	send_cmd_byte(0x01);//soft reset
	vTaskDelay(1000/ portTICK_PERIOD_MS);

	//power control A
	send_cmd_byte(0xCB);
	tx[0] = 0x39;
	tx[1] = 0x2C;
	tx[2] = 0x00;
	tx[3] = 0x34;
	tx[4] = 0x02;
	m_io_driver.send_data(tx,5);

	//power control B
	send_cmd_byte(0xCF);
	tx[0] = 0x00;
	tx[1] = 0xC1;
	tx[2] = 0x30;
	m_io_driver.send_data(tx,3);

	//driver timing control A
	send_cmd_byte(0xE8);
	tx[0] = 0x85;
	tx[1] = 0x00;
	tx[2] = 0x78;
	m_io_driver.send_data(tx,3);

	//driver timing control B
	send_cmd_byte(0xEA);
	tx[0] = 0x00;
	tx[1] = 0x00;
	m_io_driver.send_data(tx,2);

	//power on sequence control
	send_cmd_byte(0xED);
	tx[0]=0x64;
	tx[1]=0x03;
	tx[2]=0x12;
	tx[3]=0x81;
	m_io_driver.send_data(tx,4);

	//pump ratio control
	send_cmd_byte(0xF7);
	tx[0]=0x20;
	m_io_driver.send_data(tx, 1);

	//power control,VRH[5:0]
	send_cmd_byte(0xC0);
	tx[0]=0x23;
	m_io_driver.send_data(tx, 1);

	//Power control,SAP[2:0];BT[3:0]
	send_cmd_byte(0xC1);
	tx[0]=0x10;
	m_io_driver.send_data(tx, 1);

	//vcm control
	send_cmd_byte(0xC5);
	tx[0]=0x3E;
	tx[1]=0x28;
	m_io_driver.send_data(tx, 2);

	//vcm control 2
	send_cmd_byte(0xC7);
	tx[0]=0x86;
	m_io_driver.send_data(tx, 1);

	//memory access control
	send_cmd_byte(0x36);
	//tx[0]=0x28; // works for small display
	tx[0]=0x08 | 0x40; // | 0x40; // works for large display
	m_io_driver.send_data(tx, 1);

	//pixel format
	send_cmd_byte(0x3A);
	tx[0]=0x55;
	m_io_driver.send_data(tx, 1);

	//frameration control,normal mode full colours
	send_cmd_byte(0xB1);
	tx[0]=0x00;
	tx[1]=0x18;
	m_io_driver.send_data(tx, 2);

	//display function control
	send_cmd_byte(0xB6);
	tx[0]=0x08;
	tx[1]=0x82;
	tx[2]=0x27;
	m_io_driver.send_data(tx, 3);

	//3gamma function disable
	send_cmd_byte(0xF2);
	tx[0]=0x00;
	m_io_driver.send_data(tx, 1);

	//gamma curve selected
	send_cmd_byte(0x26);
	tx[0]=0x01;
	m_io_driver.send_data(tx, 1);

	//set positive gamma correction
	send_cmd_byte(0xE0);
	tx[0]=0x0F;
	tx[1]=0x31;
	tx[2]=0x2B;
	tx[3]=0x0C;
	tx[4]=0x0E;
	tx[5]=0x08;
	tx[6]=0x4E;
	tx[7]=0xF1;
	tx[8]=0x37;
	tx[9]=0x07;
	tx[10]=0x10;
	tx[11]=0x03;
	tx[12]=0x0E;
	tx[13]=0x09;
	tx[14]=0x00;
	m_io_driver.send_data(tx, 15);

	//set negative gamma correction
	send_cmd_byte(0xE1);
	tx[0]=0x00;
	tx[1]=0x0E;
	tx[2]=0x14;
	tx[3]=0x03;
	tx[4]=0x11;
	tx[5]=0x07;
	tx[6]=0x31;
	tx[7]=0xC1;
	tx[8]=0x48;
	tx[9]=0x08;
	tx[10]=0x0F;
	tx[11]=0x0C;
	tx[12]=0x31;
	tx[13]=0x36;
	tx[14]=0x0F;
	m_io_driver.send_data(tx, 15);

	//exit sleep
	send_cmd_byte(0x11);
	vTaskDelay(120/ portTICK_PERIOD_MS);
	//display on
	send_cmd_byte(0x29);
}

void ili9341_lcd_driver::clear(uint16_t colour)
{
    uint16_t i,j;
	set_addr(0,0,LCD_WIDTH-1,LCD_HEIGHT-1);

	uint8_t buf[LCD_WIDTH*2];

	for(i=0;i<LCD_WIDTH;i++)
	{
		for(j=0;j<LCD_HEIGHT;j++)
		{
			buf[j*2+0] = colour>>8;
			buf[j*2+1] = colour;
		}
		m_io_driver.send_data(buf, LCD_HEIGHT * 2);
	}
}

void ili9341_lcd_driver::send_data(uint8_t* data, uint32_t len)
{
    m_io_driver.send_data(data, len);
}

void ili9341_lcd_driver::set_addr(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint8_t buf[16];
	send_cmd_byte(0x2A);
	buf[0] = x1>>8;
	buf[1] = x1;
	buf[2] = x2>>8;
	buf[3] = x2;
	m_io_driver.send_data(buf, 4);

	send_cmd_byte(0x2B);
	buf[0] = y1>>8;
	buf[1] = y1;
	buf[2] = y2;
	buf[3] = y2;
	m_io_driver.send_data(buf, 4);

	send_cmd_byte(0x2C);//meory write
}