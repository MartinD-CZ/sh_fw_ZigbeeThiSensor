#include "sensors.h"
#include "zigbee.h"

#include "mal_gpio.h"
#include "mal_tick.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "nvs_flash.h"


const Gpio led{GPIO_NUM_12};

static const char *TAG = "main";


extern "C" void app_main(void)
{
    sensors::initI2c();
    sensors::initThermHum();
    sensors::initIllum();
    sensors::initVbatMeasurement();
        
    led.initOutput(Gpio::Speed::SLOW, Gpio::Output::HIGH, Gpio::Type::OPEN_DRAIN);
    led.setLow();
    tick::delay(200);
    led.setHigh();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition("zb_storage"));

    esp_pm_config_t pm_config = 
    {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    zigbee::startTask();

    while (true)
    {
        const auto th = sensors::measureThermHum();
        ESP_LOGI(TAG, "Temperature %li.%02li °C, humidity %li.%02li %%", th.first / 100, th.first % 100, th.second / 100, th.second % 100);
        zigbee::updateTempHum(th.first, th.second);

        sensors::startIllumMeasurement();
        const auto ill = sensors::getIllum();
        ESP_LOGI(TAG, "Illuminance %lu.%03lu lx", ill / 1000, ill % 1000);
        zigbee::updateIlluminance(ill);

        const auto voltage = sensors::measureBattery();
        ESP_LOGI(TAG, "Battery voltage %u mV", static_cast<unsigned>(voltage));
        zigbee::updateBattery(voltage);

        tick::delay(60'000);
    }
}
