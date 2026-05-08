#pragma once


#include <cstdint>


namespace zigbee
{
    void startTask();
    void updateTempHum(int16_t temp, uint16_t hum);
    void updateIlluminance(uint32_t ill_mLx);
}