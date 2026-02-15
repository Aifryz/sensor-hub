#pragma once

#include <cstdint>

enum class MeasurementType: uint8_t
{
    Status = 0x00,
    Temperature = 0x01,
    Humidity = 0x02,
    Light = 0x03,
    Battery = 0x04,
};

enum class UnitType: uint8_t
{
    Units = 0x00,
    Tens = 0x01,
    Hundreds = 0x02,
    Thousands = 0x03,
};

using SensorLocation = uint8_t;
