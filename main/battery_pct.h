#pragma once


#include <cstdint>


enum class BatteryType
{
    LiIon1S,
    Alkaline2S,
};


uint8_t battery_mv_to_zigbee_percent(uint16_t mv, BatteryType type);