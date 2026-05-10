#pragma once

#include <cstdint>


namespace onboard_led
{
    void init();
    void on();
    void off();
    void blinkOn(uint32_t onTimeMs, uint32_t offTimeMs);
    void blinkOff();
}
