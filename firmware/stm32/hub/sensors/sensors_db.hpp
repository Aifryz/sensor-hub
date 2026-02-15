#pragma once

#include "sensors_const.hpp"
#include <cstdint>
#include <array>
#include <optional>


class SensorDB
{
    public:
    SensorDB();
    //void AddMeasurement(SensorLocation location, MeasurementType type, int32_t value);
    void AddMeasurement(SensorLocation location, MeasurementType type, int16_t value);
    //void AddMeasurement(SensorLocation location, MeasurementType type, int8_t value);

    std::optional<int16_t> GetLastMeasurement(SensorLocation location, MeasurementType type) const;

    private:

    struct NodeData
    {
        int16_t temperature;
        int16_t humidity;
        int16_t light;
        int16_t battery;
    };

    std::array<NodeData, 256> m_data;

};

SensorDB& GetSensorDB();