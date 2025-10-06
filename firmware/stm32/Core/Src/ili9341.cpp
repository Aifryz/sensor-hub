#include "ili9341.hpp"

#include "bsp/board_gpio.hpp"
#include "drv/spi.hpp"


#define HIGH 1
#define LOW 0

#define LCD_W 240
#define LCD_H 320

//extern spi_device_handle_t lcdSpidev;

#define BLOCK_SIZE 256
void LcdSpiTransmit(uint8_t* data, uint16_t len)
{
	lcd_cs_pin::clear();
	uint16_t rem = len;
	uint8_t* ptr = data;
	//spi_device_acquire_bus(lcdSpidev, portMAX_DELAY);
	
	while(rem > BLOCK_SIZE)
	{
		lcd_spi.send(ptr, BLOCK_SIZE);
		rem -= BLOCK_SIZE;
		ptr += BLOCK_SIZE;
	}

	lcd_spi.send(ptr, rem);

	lcd_cs_pin::set();

		//spi_device_release_bus(lcdSpidev);
  
}

void LcdSpiTransmitReceive(uint8_t* tx_data, uint8_t* rx_data,  uint16_t len)
{
	/*
  spi_transaction_t t = {
        .rxlength = 8*len,
        .length = 8*len,
        .rx_buffer = rx_data,
        .tx_buffer = tx_data
    };
    esp_err_t ret = spi_device_polling_transmit(lcdSpidev, &t);
    ESP_ERROR_CHECK(ret);
	*/
	lcd_cs_pin::clear();
	lcd_spi.transfer(tx_data, rx_data, len);
	lcd_cs_pin::set();
}



void ILI9341_SendCmd(uint8_t* cmd, uint32_t len)
{
	//gpio_set_level(LCD_DCRS_PIN, LOW);
	lcd_dcrs_pin::clear();
	LcdSpiTransmit(cmd, len);
}
void ILI9341_SendCmd8(uint8_t cmd)
{
	uint8_t buf = cmd;
	lcd_dcrs_pin::clear();
	LcdSpiTransmit(&buf, 1);
}
void ILI9341_SendData(uint8_t* data, uint32_t len)
{
	lcd_dcrs_pin::set();
	LcdSpiTransmit(data, len);
}
void ILI9341_ReadData(uint8_t* data_tx,uint8_t* data_rx , uint32_t len)
{
	lcd_dcrs_pin::set();
	LcdSpiTransmitReceive(data_tx, data_rx, len);
}


void ILI9341_SetAddr(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)//set coordinate for print or other function
{
	uint8_t buf[16];
	ILI9341_SendCmd8(0x2A);
	buf[0] = x1>>8;
	buf[1] = x1;
	buf[2] = x2>>8;
	buf[3] = x2;
	ILI9341_SendData(buf, 4);

	ILI9341_SendCmd8(0x2B);
	buf[0] = y1>>8;
	buf[1] = y1;
	buf[2] = y2;
	buf[3] = y2;
	ILI9341_SendData(buf, 4);

	ILI9341_SendCmd8(0x2C);//meory write
}



void ILI9341_Clear(uint16_t colour)
{
	uint16_t i,j;
	ILI9341_SetAddr(0,0,LCD_W-1,LCD_H-1);

	uint8_t buf[320*2];

	for(i=0;i<LCD_W;i++)
	{
		for(j=0;j<LCD_H;j++)
		{
			buf[j*2+0] = colour>>8;
			buf[j*2+1] = colour;
		}
		ILI9341_SendData(buf,LCD_H*2);
	}
}

void ILI9341_Init(void)
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


	ILI9341_SendCmd8(0x01);//soft reset
	vTaskDelay(1000/ portTICK_PERIOD_MS);

	//power control A
	ILI9341_SendCmd8(0xCB);
	tx[0] = 0x39;
	tx[1] = 0x2C;
	tx[2] = 0x00;
	tx[3] = 0x34;
	tx[4] = 0x02;
	ILI9341_SendData(tx,5);

	//power control B
	ILI9341_SendCmd8(0xCF);
	tx[0] = 0x00;
	tx[1] = 0xC1;
	tx[2] = 0x30;
	ILI9341_SendData(tx,3);

	//driver timing control A
	ILI9341_SendCmd8(0xE8);
	tx[0] = 0x85;
	tx[1] = 0x00;
	tx[2] = 0x78;
	ILI9341_SendData(tx,3);

	//driver timing control B
	ILI9341_SendCmd8(0xEA);
	tx[0] = 0x00;
	tx[1] = 0x00;
	ILI9341_SendData(tx,2);

	//power on sequence control
	ILI9341_SendCmd8(0xED);
	tx[0]=0x64;
	tx[1]=0x03;
	tx[2]=0x12;
	tx[3]=0x81;
	ILI9341_SendData(tx,4);

	//pump ratio control
	ILI9341_SendCmd8(0xF7);
	tx[0]=0x20;
	ILI9341_SendData(tx, 1);

	//power control,VRH[5:0]
	ILI9341_SendCmd8(0xC0);
	tx[0]=0x23;
	ILI9341_SendData(tx, 1);

	//Power control,SAP[2:0];BT[3:0]
	ILI9341_SendCmd8(0xC1);
	tx[0]=0x10;
	ILI9341_SendData(tx, 1);

	//vcm control
	ILI9341_SendCmd8(0xC5);
	tx[0]=0x3E;
	tx[1]=0x28;
	ILI9341_SendData(tx, 2);

	//vcm control 2
	ILI9341_SendCmd8(0xC7);
	tx[0]=0x86;
	ILI9341_SendData(tx, 1);

	//memory access control
	ILI9341_SendCmd8(0x36);
	tx[0]=0x48;
	ILI9341_SendData(tx, 1);

	//pixel format
	ILI9341_SendCmd8(0x3A);
	tx[0]=0x55;
	ILI9341_SendData(tx, 1);

	//frameration control,normal mode full colours
	ILI9341_SendCmd8(0xB1);
	tx[0]=0x00;
	tx[1]=0x18;
	ILI9341_SendData(tx, 2);

	//display function control
	ILI9341_SendCmd8(0xB6);
	tx[0]=0x08;
	tx[1]=0x82;
	tx[2]=0x27;
	ILI9341_SendData(tx, 3);

	//3gamma function disable
	ILI9341_SendCmd8(0xF2);
	tx[0]=0x00;
	ILI9341_SendData(tx, 1);

	//gamma curve selected
	ILI9341_SendCmd8(0x26);
	tx[0]=0x01;
	ILI9341_SendData(tx, 1);

	//set positive gamma correction
	ILI9341_SendCmd8(0xE0);
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
	ILI9341_SendData(tx, 15);

	//set negative gamma correction
	ILI9341_SendCmd8(0xE1);
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
	ILI9341_SendData(tx, 15);

	//exit sleep
	ILI9341_SendCmd8(0x11);
	vTaskDelay(120/ portTICK_PERIOD_MS);
	//display on
	ILI9341_SendCmd8(0x29);


	const uint16_t a = 0xF800; // Top 5 bits set (red)
	const uint16_t b = 0x07E0; // mid 6 bits set // green?
	const uint16_t c = 0x001F; // lower 5 bits set (blue)
	uint16_t col = a; //green
	//col = 0;







}
