/*
 * nrf_radio.cpp
 *
 *  Created on: Apr 3, 2023
 *      Author: robal
 */

#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include "drv/spi.hpp"
#include "bsp/board_gpio.hpp"
#include <atomic>

namespace nrf24_regs
{
namespace
{
	template<uint8_t reg_addr>
	struct reg_check
	{
		static_assert(reg_addr<=31);
		static constexpr uint8_t value = reg_addr;
	};
	template<uint8_t pipe>
	struct pipe_check
	{
		static_assert(pipe<=7);
		static constexpr uint8_t value = pipe;
	};
}
template<uint8_t reg_addr>
constexpr uint8_t r_register = reg_check<reg_addr>::value;

template<uint8_t reg_addr>
constexpr uint8_t w_register = 0x20 | reg_check<reg_addr>::value;

constexpr uint8_t r_rx_payload = 0x61;
constexpr uint8_t w_tx_payload = 0xA0;
constexpr uint8_t flush_tx = 0xE1;
constexpr uint8_t flush_rx = 0xE2;
constexpr uint8_t reuse_tx_pl = 0xE3;
constexpr uint8_t r_rx_pl_wid = 0x60;

template<uint8_t pipe_addr>
constexpr uint8_t w_ack_payload = 0xA8 | pipe_check<pipe_addr>::value;

constexpr uint8_t w_tx_payload_no_ack = 0xB0;
constexpr uint8_t nop = 0xFF;

//Registers

constexpr uint8_t config = 0x00;
constexpr uint8_t en_aa = 0x01;
constexpr uint8_t en_rxaddr = 0x02;
constexpr uint8_t setup_aw = 0x03;
constexpr uint8_t setup_retr = 0x04;
constexpr uint8_t rf_ch = 0x05;
constexpr uint8_t rf_setup = 0x06;
constexpr uint8_t status = 0x07;
constexpr uint8_t observe_tx = 0x08;
constexpr uint8_t rpd = 0x09;
constexpr uint8_t rx_addr_p0 = 0x0a;
constexpr uint8_t rx_addr_p1 = 0x0b;
constexpr uint8_t rx_addr_p2 = 0x0c;
constexpr uint8_t rx_addr_p3 = 0x0d;
constexpr uint8_t rx_addr_p4 = 0x0e;
constexpr uint8_t rx_addr_p5 = 0x0f;
constexpr uint8_t tx_addr = 0x10;
constexpr uint8_t rx_pw_p0 = 0x11;
constexpr uint8_t rx_pw_p1 = 0x12;
constexpr uint8_t rx_pw_p2 = 0x13;
constexpr uint8_t rx_pw_p3 = 0x14;
constexpr uint8_t rx_pw_p4 = 0x15;
constexpr uint8_t rx_pw_p5 = 0x16;
constexpr uint8_t fifo_status = 0x17;
constexpr uint8_t dynpd = 0x1c;
constexpr uint8_t feature = 0x1d;


}

class nrf24_radio
{
public:
private:
	uint8_t last_status;
};

void nrf_task(void* arg)
{
	nrf_cs_pin::set();
	vTaskDelay(100);
	std::byte rx_buf[4];
	std::byte tx_buf[4];
	std::memset(rx_buf, 0, 4);
	std::memset(tx_buf, 0, 4);
	nrf_cs_pin::clear();
	nrf_spi.transfer(tx_buf, rx_buf, 4);
	nrf_cs_pin::set();
	user_led_pin::set();
	//uint8_t xtmp = nrf24_regs::w_register<-1>;

	std::atomic<uint32_t> x;
	x += 1;
	//xtmp = nrf24_regs::r_register<24>;
	//xtmp = nrf24_regs::w_ack_payload<50>;
	while(1)
	{
		std::printf("Hi nrf\r\n");
		//user_led_pin::toggle();

		vTaskDelay(500);
	}
}


TaskHandle_t nrf_task_handle;
void init_nrf_radio()
{
	BaseType_t ret = xTaskCreate(&nrf_task, "radio", 512, NULL, 7, &nrf_task_handle);
}
