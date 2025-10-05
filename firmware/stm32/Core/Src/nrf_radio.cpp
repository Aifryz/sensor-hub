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
#include "safebits.hpp"
#include <atomic>
#include <array>


struct nrf24_tag{};
struct stm32_tag{};

using nrf_reg0 = safebits::reg<uint8_t, 0, nrf24_tag>;
using stm32_reg0 = safebits::reg<uint8_t, 0, stm32_tag>;

constexpr nrf_reg0::bits<0x01, 0x01> first_on;
constexpr nrf_reg0::bits<0x01, 0x00> first_off;

constexpr nrf_reg0::bits<0x02, 0x01> second_on;

constexpr nrf_reg0::bits<0x03, 0x03> second_first_on;

constexpr stm32_reg0::bits<0x02, 0x01> second_on_st;

//constexpr auto res1 = first_on|first_off;
//constexpr auto res2 = first_on|second_on;
//constexpr auto res3 = first_on|second_on_st;

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
//Commands
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



constexpr nrf_reg0::bits<0x03, 0x03> second_first_on;

}

namespace nrf24
{
	namespace regs
	{
		namespace raw
		{
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
		
		struct config : public safebits::reg<uint8_t, raw::config, nrf24_tag>
		{
			static constexpr bits<0x40, 0x40> mask_rx_dr = {};
			static constexpr bits<0x40, 0x40> disable_rx_dr_irq = {};
			static constexpr bits<0x40, 0x00> enable_rx_dr_irq = {};

			static constexpr bits<0x20, 0x20> mask_tx_ds = {};
			static constexpr bits<0x20, 0x20> disable_tx_ds_irq = {};
			static constexpr bits<0x20, 0x00> enable_tx_ds_irq = {};

			static constexpr bits<0x10, 0x10> mask_max_rt = {};
			static constexpr bits<0x10, 0x10> disable_max_rt_irq = {};
			static constexpr bits<0x10, 0x00> enable_max_rt_irq = {};

			static constexpr bits<0x08, 0x08> enable_crc = {};
			static constexpr bits<0x08, 0x00> disable_crc = {};

			static constexpr bits<0x04, 0x04> crc_2byte = {};
			static constexpr bits<0x04, 0x00> crc_1byte = {};

			static constexpr bits<0x02, 0x02> power_up = {};
			static constexpr bits<0x02, 0x00> power_down = {};

			static constexpr bits<0x01, 0x01> primary_rx = {};
			static constexpr bits<0x01, 0x00> primary_tx = {};
		};

		struct en_aa : public safebits::reg<uint8_t, raw::en_aa, nrf24_tag>
		{
			static constexpr bits<0x20, 0x20> enable_auto_ack_pipe5 = {};
			static constexpr bits<0x20, 0x00> disable_auto_ack_pipe5 = {};

			static constexpr bits<0x10, 0x10> enable_auto_ack_pipe4 = {};
			static constexpr bits<0x10, 0x00> disable_auto_ack_pipe4 = {};

			static constexpr bits<0x08, 0x08> enable_auto_ack_pipe3 = {};
			static constexpr bits<0x08, 0x00> disable_auto_ack_pipe3 = {};

			static constexpr bits<0x04, 0x04> enable_auto_ack_pipe2 = {};
			static constexpr bits<0x04, 0x00> disable_auto_ack_pipe2 = {};

			static constexpr bits<0x02, 0x02> enable_auto_ack_pipe1 = {};
			static constexpr bits<0x02, 0x00> disable_auto_ack_pipe1 = {};

			static constexpr bits<0x01, 0x01> enable_auto_ack_pipe0 = {};
			static constexpr bits<0x01, 0x00> disable_auto_ack_pipe0 = {};
		};

		struct en_rxaddr : public safebits::reg<uint8_t, raw::en_rxaddr, nrf24_tag>
		{
			static constexpr bits<0x20, 0x20> enable_rx_pipe5 = {};
			static constexpr bits<0x20, 0x00> disable_rx_pipe5 = {};

			static constexpr bits<0x10, 0x10> enable_rx_pipe4 = {};
			static constexpr bits<0x10, 0x00> disable_rx_pipe4 = {};

			static constexpr bits<0x08, 0x08> enable_rx_pipe3 = {};
			static constexpr bits<0x08, 0x00> disable_rx_pipe3 = {};

			static constexpr bits<0x04, 0x04> enable_rx_pipe2 = {};
			static constexpr bits<0x04, 0x00> disable_rx_pipe2 = {};

			static constexpr bits<0x02, 0x02> enable_rx_pipe1 = {};
			static constexpr bits<0x02, 0x00> disable_rx_pipe1 = {};

			static constexpr bits<0x01, 0x01> enable_rx_pipe0 = {};
			static constexpr bits<0x01, 0x00> disable_rx_pipe0 = {};
		};

		struct setup_aw : public safebits::reg<uint8_t, raw::setup_aw, nrf24_tag>
		{
			static constexpr bits<0x03, 0x01> addr_width_3bytes = {};
			static constexpr bits<0x03, 0x02> addr_width_4bytes = {};
			static constexpr bits<0x03, 0x03> addr_width_5bytes = {};
		};

		struct setup_retr : public safebits::reg<uint8_t, raw::setup_retr, nrf24_tag>
		{
			static constexpr bits<0xF0, 0x00> auto_retransmit_wait_250us = {};
			static constexpr bits<0xF0, 0x10> auto_retransmit_wait_500us = {};
			static constexpr bits<0xF0, 0x20> auto_retransmit_wait_750us = {};
			static constexpr bits<0xF0, 0x30> auto_retransmit_wait_1000us = {};
			static constexpr bits<0xF0, 0x40> auto_retransmit_wait_1250us = {};
			static constexpr bits<0xF0, 0x50> auto_retransmit_wait_1500us = {};
			static constexpr bits<0xF0, 0x60> auto_retransmit_wait_1750us = {};
			static constexpr bits<0xF0, 0x70> auto_retransmit_wait_2000us = {};
			static constexpr bits<0xF0, 0x80> auto_retransmit_wait_2250us = {};
			static constexpr bits<0xF0, 0x90> auto_retransmit_wait_2500us = {};
			static constexpr bits<0xF0, 0xA0> auto_retransmit_wait_2750us = {};
			static constexpr bits<0xF0, 0xB0> auto_retransmit_wait_3000us = {};
			static constexpr bits<0xF0, 0xC0> auto_retransmit_wait_3250us = {};
			static constexpr bits<0xF0, 0xD0> auto_retransmit_wait_3500us = {};
			static constexpr bits<0xF0, 0xE0> auto_retransmit_wait_3750us = {};
			static constexpr bits<0xF0, 0xF0> auto_retransmit_wait_4000us = {};

			static constexpr bits<0x0F, 0x00> retransmit_disabled = {};
			static constexpr bits<0x0F, 0x01> retransmit_1 = {};
			static constexpr bits<0x0F, 0x02> retransmit_2 = {};
			static constexpr bits<0x0F, 0x03> retransmit_3 = {};
			static constexpr bits<0x0F, 0x04> retransmit_4 = {};
			static constexpr bits<0x0F, 0x05> retransmit_5 = {};
			static constexpr bits<0x0F, 0x06> retransmit_6 = {};
			static constexpr bits<0x0F, 0x07> retransmit_7 = {};
			static constexpr bits<0x0F, 0x08> retransmit_8 = {};
			static constexpr bits<0x0F, 0x09> retransmit_9 = {};
			static constexpr bits<0x0F, 0x0A> retransmit_10 = {};
			static constexpr bits<0x0F, 0x0B> retransmit_11 = {};
			static constexpr bits<0x0F, 0x0C> retransmit_12 = {};
			static constexpr bits<0x0F, 0x0D> retransmit_13 = {};
			static constexpr bits<0x0F, 0x0E> retransmit_14 = {};
			static constexpr bits<0x0F, 0x0F> retransmit_15 = {};
		};

		struct rf_ch : public safebits::reg<uint8_t, raw::rf_ch, nrf24_tag>
		{
			template<uint8_t rf_ch>
			static constexpr bits_range<0xFF, rf_ch, 0, 0x7F> rf_channel = {};
		};

		struct rf_setup : public safebits::reg<uint8_t, raw::rf_setup, nrf24_tag>
		{
			static constexpr bits<0x80, 0x80> cont_wave_enable = {};
			static constexpr bits<0x80, 0x00> cont_wave_disable = {};

			static constexpr bits<0x28, 0x20> data_rate_250_kbps = {};
			static constexpr bits<0x28, 0x00> data_rate_1000_kbps = {};
			static constexpr bits<0x28, 0x08> data_rate_2000_kbps = {};

			static constexpr bits<0x10, 0x10> force_pll_lock = {};

			static constexpr bits<0x06, 0x00> rf_power_n18dbm = {};
			static constexpr bits<0x06, 0x02> rf_power_n12dbm = {};
			static constexpr bits<0x06, 0x04> rf_power_n6dbm = {};
			static constexpr bits<0x06, 0x06> rf_power_0dbm = {};
		};
		//primary read only
		struct status : public safebits::reg<uint8_t, raw::status, nrf24_tag>
		{
			static constexpr bits<0x40, 0x40> rx_data_ready_irq = {};
			static constexpr bits<0x20, 0x20> tx_data_sent_irq = {};
			static constexpr bits<0x10, 0x10> max_retransmit_irq = {};

			template<uint8_t num>
			static constexpr bits_range<0x0E, num, 0, 0x0E> rx_pipe_no = {};

			static constexpr bits<0x01, 0x01> tx_full = {};
		};
		//primary read only
		struct observe_tx : public safebits::reg<uint8_t, raw::observe_tx, nrf24_tag>
		{
			template<uint8_t num>
			static constexpr bits_range<0x0E, num, 0, 0x0E> rx_pipe_no = {};
			
		};






		
		
	}
}

class nrf24_radio
{
public:
	void init()
	{

		//hacked together just for now
		uint8_t config_value = (nrf24::regs::config::enable_crc | 
		  nrf24::regs::config::primary_rx | 
		  nrf24::regs::config::crc_2byte).value;
		write_register_unsafe(nrf24::regs::raw::config, config_value);

		// enable pipe 0
		write_register_unsafe(nrf24::regs::raw::en_rxaddr, nrf24::regs::en_rxaddr::enable_rx_pipe0.value);

		// disable retransmit
		write_register_unsafe(nrf24::regs::raw::setup_retr, 0);

		//disable auto ack one every pipe
		write_register_unsafe(nrf24::regs::raw::en_aa, 0);

		// setup 5 byte address
		write_register_unsafe(nrf24::regs::raw::setup_aw, nrf24::regs::setup_aw::addr_width_5bytes.value);

		// rf setup 250kbps, 0dbm
		write_register_unsafe(nrf24::regs::raw::rf_setup, (nrf24::regs::rf_setup::data_rate_250_kbps | nrf24::regs::rf_setup::rf_power_0dbm).value);

		//datawidth setup
		write_register_unsafe(nrf24::regs::raw::rx_pw_p0, 5);

	}

	void setup_rx_addr()
	{
		//setup rx address pipe 0
		nrf_cs_pin::clear();
		std::array<uint8_t, 5> addr = {0x76, 0x03, 0xC9, 0x63, 0x0C};
		m_tx_buf[0] = nrf24_regs::w_register<nrf24::regs::raw::rx_addr_p0>;
		for (size_t i = 0; i < addr.size(); ++i) {
			m_tx_buf[i + 1] = addr[i];
		}
		nrf_spi.transfer(m_tx_buf.data(), m_rx_buf.data(), 6);
		nrf_cs_pin::set();
		m_last_status = m_rx_buf[0];
	}

	void power_up()
	{
		m_tx_buf[0] = nrf24_regs::r_register<nrf24::regs::raw::config>;
		m_tx_buf[1] = 0xFF; // nop
		nrf_cs_pin::clear();
		nrf_spi.transfer(m_tx_buf.data(), m_rx_buf.data(), 2);	
		nrf_cs_pin::set();
		uint8_t config_value = m_rx_buf[1] | nrf24::regs::config::power_up.value;

		m_tx_buf[0] = nrf24_regs::w_register<nrf24::regs::raw::config>;
		m_tx_buf[1] = config_value;
		nrf_cs_pin::clear();
		nrf_spi.transfer(m_tx_buf.data(), m_rx_buf.data(), 2);
		nrf_cs_pin::set();
		nrf_ce_pin::set();
 	}

	void power_down()
	{
		m_tx_buf[0] = nrf24_regs::r_register<nrf24::regs::raw::config>;
		m_tx_buf[1] = 0xFF; // nop
		nrf_cs_pin::clear();
		nrf_spi.transfer(m_tx_buf.data(), m_rx_buf.data(), 2);	
		nrf_cs_pin::set();
		uint8_t config_value = m_rx_buf[1] & ~nrf24::regs::config::power_up.value;

		m_tx_buf[0] = nrf24_regs::w_register<nrf24::regs::raw::config>;
		m_tx_buf[1] = config_value;
		nrf_cs_pin::clear();
		nrf_spi.transfer(m_tx_buf.data(), m_rx_buf.data(), 2);
		nrf_cs_pin::set();
		nrf_ce_pin::clear();
	}

	uint8_t get_last_status() const
	{
		return m_last_status;
	}

	uint8_t read_status()
	{
		m_tx_buf[0] = nrf24_regs::nop;
		nrf_cs_pin::clear();
		nrf_spi.transfer(m_tx_buf.data(), m_rx_buf.data(), 1);
		nrf_cs_pin::set();
		m_last_status = m_rx_buf[0];
		return m_last_status;
	}

	struct rx_data
	{
		uint8_t node_id;
		uint8_t seq;
		uint8_t tag;
		int16_t data;
	};

	rx_data receive()
	{
		rx_data ret;
		nrf_ce_pin::clear();
		m_tx_buf[0] = nrf24_regs::r_rx_payload;
		nrf_cs_pin::clear();
		nrf_spi.transfer(m_tx_buf.data(), m_rx_buf.data(), 6);
		nrf_cs_pin::set();
		ret.node_id = m_rx_buf[1];
		ret.seq = m_rx_buf[2];
		ret.tag = m_rx_buf[3];
		int16_t data = (((uint16_t)(m_rx_buf[4])) << 8) | (uint16_t)(m_rx_buf[5]);
		ret.data = data;
		nrf_ce_pin::set();
		return ret;
	}



	template<uint8_t reg_addr>
	void read_register(uint8_t len);
	template<uint8_t reg_addr>
	void write_register(uint8_t len);
	void read_rx_payload(uint8_t len);
	void write_tx_payload(uint8_t len);
	void flush_tx();
	void flush_rx();
	void reuse_tx_payload();
	void activate();
	void read_fifo_rx_payload();
	template<uint8_t pipe_addr>
	void write_ack_payload(uint8_t len);
	void write_tx_payload_no_ack(uint8_t len);
	void nop();

	void write_register_unsafe(uint8_t reg, uint8_t data)
	{
		nrf_cs_pin::clear();
		m_tx_buf[0] = 0x20 | (reg & 0x1F); //todo fixme
		m_tx_buf[1] = data;
		nrf_spi.transfer(m_tx_buf.data(), m_rx_buf.data(), 2);
		nrf_cs_pin::set();
		m_last_status = m_rx_buf[0];
	}
private:
	// RX buffer - status + upto 32 bytes of data
	std::array<uint8_t, 33> m_rx_buf;
	// TX buffer - cmd + upto 32 bytes of data
	std::array<uint8_t, 33> m_tx_buf;
	uint8_t m_last_status;
};

nrf24_radio nrf24_device;

void nrf_task(void* arg)
{
	nrf_cs_pin::set();
	nrf_ce_pin::clear();
	vTaskDelay(500); //wait for the radio to power up	

	nrf24_device.init();

	//   NRF24_SetChannel(&hNrf, channel);
	uint8_t channel = 53;
	nrf24_device.write_register_unsafe(nrf24::regs::raw::rf_ch, channel);

   //NRF24_SetRXPipeAddr(&hNrf, addr, 0);
   nrf24_device.setup_rx_addr();
   



   //NRF24_PowerUp(&hNrf);
   nrf24_device.power_up();
   vTaskDelay(5); //wait for the radio to power up	



	//nrf_cs_pin::set();
	//vTaskDelay(100);
	//std::byte rx_buf[4];
	//std::byte tx_buf[4];
	//std::memset(rx_buf, 0, 4);
	//std::memset(tx_buf, 0, 4);
	//nrf_cs_pin::clear();
	//nrf_spi.transfer(tx_buf, rx_buf, 4);
	//nrf_cs_pin::set();
	//sys_led_pin::set();
	//uint8_t xtmp = nrf24_regs::w_register<-1>;

	//std::atomic<uint32_t> x;
	//x += 1;
	//xtmp = nrf24_regs::r_register<24>;
	//xtmp = nrf24_regs::w_ack_payload<50>;
	while(1)
	{
		//std::printf("Hi nrf\r\n");
		//user_led_pin::toggle();
		sys_led_pin::toggle();
		
		nrf24_device.read_status();
		uint8_t status = nrf24_device.get_last_status();
		//std::printf("nrf status: 0x%02X\r\n", status);
		uint8_t pipe = (status >> 1) &0x07;
		if(pipe == 0)
		{
			auto data = nrf24_device.receive();
			std::printf("Data: ID, %02X seq %02X tag %02x, data %d \r\n",
				 data.node_id, data.seq, data.tag, data.data);
			

		}

		vTaskDelay(100);

	}
}


TaskHandle_t nrf_task_handle;
void init_nrf_radio()
{
	BaseType_t ret = xTaskCreate(&nrf_task, "radio", 512, NULL, 7, &nrf_task_handle);
}
