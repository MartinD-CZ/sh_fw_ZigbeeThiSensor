#include "battery_pct.h"

#include <cstddef>


struct BatteryCurvePoint
{
    uint16_t mv;
    uint8_t percent; // 0..100
};


static uint8_t interpolate_percent_from_curve(uint16_t mv, const BatteryCurvePoint* curve, size_t count)
{
    if (count == 0) {
        return 0xFF; // unknown
    }

    // Curves are ordered from high voltage to low voltage.
    if (mv >= curve[0].mv) {
        return curve[0].percent;
    }

    if (mv <= curve[count - 1].mv) {
        return curve[count - 1].percent;
    }

    for (size_t i = 0; i < count - 1; ++i) {
        const BatteryCurvePoint high = curve[i];
        const BatteryCurvePoint low = curve[i + 1];

        if (mv <= high.mv && mv >= low.mv) {
            const uint32_t voltage_span = high.mv - low.mv;
            const uint32_t percent_span = high.percent - low.percent;
            const uint32_t voltage_above_low = mv - low.mv;

            return static_cast<uint8_t>(
                low.percent +
                ((voltage_above_low * percent_span) + voltage_span / 2) / voltage_span
            );
        }
    }

    return 0;
}


uint8_t battery_mv_to_zigbee_percent(uint16_t mv, BatteryType type)
{
    // Approximate 1-cell Li-ion / LiPo open-circuit curve.
    // Tune this for your actual cell and cutoff voltage.
    static constexpr BatteryCurvePoint li_ion_1s[] = {
        {4200, 100},
        {4150, 95},
        {4100, 90},
        {4050, 85},
        {4000, 80},
        {3900, 65},
        {3800, 50},
        {3700, 35},
        {3600, 20},
        {3500, 10},
        {3300, 5},
        {3000, 0},
    };

    // Approximate 2x alkaline AA/AAA curve, total pack voltage.
    // Very load-dependent; tune using your device current profile.
    static constexpr BatteryCurvePoint alkaline_2s[] = {
        {3200, 100},
        {3100, 95},
        {3000, 90},
        {2900, 75},
        {2800, 60},
        {2700, 45},
        {2600, 30},
        {2400, 15},
        {2200, 5},
        {2000, 0},
    };

    uint8_t percent = 0xFF;

    switch (type) {
    case BatteryType::LiIon1S:
        percent = interpolate_percent_from_curve(mv, li_ion_1s, sizeof(li_ion_1s) / sizeof(li_ion_1s[0]));
        break;
    case BatteryType::Alkaline2S:
        percent = interpolate_percent_from_curve(mv, alkaline_2s, sizeof(alkaline_2s) / sizeof(alkaline_2s[0]));
        break;

    default:
        return 0xFF; // unknown
    }

    return (percent > 100) ? 100 : percent;
}