#pragma once

#include <cstddef>
#include <system_error>

#include "FreeRTOS.h"
#include "semphr.h"

#include "i2c.h"
#include "stm32f4xx_hal_i2c.h"

class i2c;


enum class i2c_error
{
    None = 0, // No error
    BusError, // General hardware bus error, 
    AcknowledgeFailure, // No ACK
    Other // Other errors
};

class i2c_bus_session
{
public:
    ~i2c_bus_session();

    i2c_error read(uint8_t device_address, uint8_t* data, size_t length);
    i2c_error write(uint8_t device_address, const uint8_t* data, size_t length);

    i2c_error mem_read(uint8_t device_address, uint8_t mem_address, uint8_t* data, size_t length);
    i2c_error mem_write(uint8_t device_address, uint8_t mem_address, const uint8_t* data, size_t length);

private:
    friend class i2c;
    i2c_bus_session(i2c& bus);
    i2c& m_bus;
};


class i2c
{
public:
    i2c(I2C_HandleTypeDef* handle);

    i2c_bus_session start_session();

private:
    void lock_bus();
    void unlock_bus();

    I2C_HandleTypeDef* m_handle;
    // Mutex to protect access to the bus
    SemaphoreHandle_t m_bus_mutex;
    // Semaphore to signal completion of a transaction
    SemaphoreHandle_t m_transaction_mutex;
    friend class i2c_bus_session;
};


static i2c board_i2c{&hi2c1};