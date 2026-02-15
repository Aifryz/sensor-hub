#include "sensors_db.hpp"
#include <cstdio>
#include <cstdint>
#include <limits>

namespace 
{
    static SensorDB sensor_db{};
};

SensorDB::SensorDB()
{
    for(auto& node : m_data)
    {
        //node.temperature = std::numeric_limits<int16_t>::min();
        //node.humidity  = std::numeric_limits<int16_t>::min();
        //node.light  = std::numeric_limits<int16_t>::min();
        //node.battery  = std::numeric_limits<int16_t>::min();
        node.temperature = 500;
        node.humidity = 500;
        node.light = 500;
        node.battery = 500;
    }
}

void SensorDB::AddMeasurement(SensorLocation location, MeasurementType type, int16_t value)
{
    switch(type)
    {
        case MeasurementType::Temperature:
            m_data[location].temperature = value;
            break;
        case MeasurementType::Humidity:
            m_data[location].humidity = value;
            break;
        case MeasurementType::Light:
            m_data[location].light = value;
            break;
        case MeasurementType::Battery:
            m_data[location].battery = value;
            break;
        default:
            break;
    }

    std::printf("Added mes %d, %d, %d, %d, %d ", location, m_data[location].temperature, m_data[location].humidity, m_data[location].light, m_data[location].battery);
}

std::optional<int16_t> SensorDB::GetLastMeasurement(SensorLocation location, MeasurementType type) const
{
    int16_t value = std::numeric_limits<uint16_t>::min();
    switch(type)
    {
        case MeasurementType::Temperature:
            value = m_data[location].temperature;
            break;
        case MeasurementType::Humidity:
            value = m_data[location].humidity;
            break;
        case MeasurementType::Light:
            value = m_data[location].light;
            break;
        case MeasurementType::Battery:
            value = m_data[location].battery;
            break;
        default:
            return std::nullopt;
    }

    if(value == std::numeric_limits<int16_t>::min())
    {
        return std::nullopt;
    }
    else
    {
        return value;
    }
}

SensorDB& GetSensorDB()
{
    return sensor_db;
}