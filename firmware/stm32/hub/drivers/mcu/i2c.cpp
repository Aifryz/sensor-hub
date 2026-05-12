#include "i2c.hpp"

namespace {
constexpr uint32_t kI2cTimeoutMs = 100;

static i2c_error translate_hal_i2c_status(I2C_HandleTypeDef* handle, HAL_StatusTypeDef status)
{
    if(status == HAL_OK)
    {
        return i2c_error::None;
    }

    if(status == HAL_TIMEOUT)
    {
        return i2c_error::AcknowledgeFailure;
    }

    if(status == HAL_ERROR && handle != nullptr)
    {
        if(handle->ErrorCode & HAL_I2C_ERROR_AF)
        {
            return i2c_error::AcknowledgeFailure;
        }
        return i2c_error::BusError;
    }

    return i2c_error::Other;
}
}

static void ensure_semaphore(SemaphoreHandle_t semaphore)
{
    configASSERT(semaphore != nullptr);
}

static void ensure_hal_ok(HAL_StatusTypeDef status)
{
    configASSERT(status == HAL_OK);
}


i2c::i2c(I2C_HandleTypeDef* handle)
    : m_handle(handle)
    , m_bus_mutex(xSemaphoreCreateMutex())
    , m_transaction_mutex(xSemaphoreCreateMutex())
{
    configASSERT(m_handle != nullptr);
    ensure_semaphore(m_bus_mutex);
    ensure_semaphore(m_transaction_mutex);
}


i2c_bus_session::i2c_bus_session(i2c& bus)
    : m_bus(bus)
{
}


i2c_bus_session::~i2c_bus_session()
{
    m_bus.unlock_bus();
}


i2c_bus_session i2c::start_session()
{
    lock_bus();
    return i2c_bus_session{*this};
}

void i2c::lock_bus()
{
    ensure_semaphore(m_bus_mutex);
    configASSERT(xSemaphoreTake(m_bus_mutex, portMAX_DELAY) == pdTRUE);
}

void i2c::unlock_bus()
{
    ensure_semaphore(m_bus_mutex);
    configASSERT(xSemaphoreGive(m_bus_mutex) == pdTRUE);
}

i2c_error i2c_bus_session::read(uint8_t device_address, uint8_t* data, size_t length)
{
    ensure_semaphore(m_bus.m_transaction_mutex);
    configASSERT(data != nullptr || length == 0);
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(m_bus.m_handle,
                                                      device_address,
                                                      data,
                                                      static_cast<uint16_t>(length),
                                                      kI2cTimeoutMs);
    return translate_hal_i2c_status(m_bus.m_handle, status);
}

i2c_error i2c_bus_session::write(uint8_t device_address, const uint8_t* data, size_t length)
{
    ensure_semaphore(m_bus.m_transaction_mutex);
    configASSERT(data != nullptr || length == 0);
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(m_bus.m_handle,
                                                       device_address,
                                                       const_cast<uint8_t*>(data),
                                                       static_cast<uint16_t>(length),
                                                       kI2cTimeoutMs);
    return translate_hal_i2c_status(m_bus.m_handle, status);
}

i2c_error i2c_bus_session::mem_read(uint8_t device_address,
                               uint8_t mem_address,
                               uint8_t* data,
                               size_t length)
{
    ensure_semaphore(m_bus.m_transaction_mutex);
    configASSERT(data != nullptr || length == 0);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(m_bus.m_handle,
                                                device_address,
                                                mem_address,
                                                I2C_MEMADD_SIZE_8BIT,
                                                data,
                                                static_cast<uint16_t>(length),
                                                kI2cTimeoutMs);
    return translate_hal_i2c_status(m_bus.m_handle, status);
}

i2c_error i2c_bus_session::mem_write(uint8_t device_address,
                                     uint8_t mem_address,
                                     const uint8_t* data,
                                     size_t length)
{
    ensure_semaphore(m_bus.m_transaction_mutex);
    configASSERT(data != nullptr || length == 0);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(m_bus.m_handle,
                                                 device_address,
                                                 mem_address,
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 const_cast<uint8_t*>(data),
                                                 static_cast<uint16_t>(length),
                                                 kI2cTimeoutMs);
    return translate_hal_i2c_status(m_bus.m_handle, status);
}
