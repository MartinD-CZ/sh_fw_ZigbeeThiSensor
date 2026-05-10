#include "sensors.h"
#include "zigbee.h"
#include "battery_pct.h"
#include "onboard_led.h"

#include "mal_tick.h"

#include "etl/vector.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char *TAG = "main";

static void batteryTask(void* param);


extern "C" void app_main(void)
{
    sensors::initI2c();
    sensors::initThermHum();
    sensors::initIllum();
    sensors::initVbatMeasurement();
    
    ESP_LOGI(TAG, "Startup\n");

    ESP_ERROR_CHECK(nvs_flash_init());

    esp_pm_config_t pm_config = 
    {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .light_sleep_enable = true
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    onboard_led::init();

    zigbee::startTask();
    xTaskCreate(batteryTask, "batt_tsk", 2048, NULL, 3, NULL);

    while (true)
    {
        const auto th = sensors::measureThermHum();
        ESP_LOGI(TAG, "Temperature %li.%02li °C, humidity %li.%02li %%", th.first / 100, th.first % 100, th.second / 100, th.second % 100);
        zigbee::updateTempHum(th.first, th.second);

        sensors::startIllumMeasurement();
        const auto ill = sensors::getIllum();
        ESP_LOGI(TAG, "Illuminance %lu.%03lu lx", ill / 1000, ill % 1000);
        zigbee::updateIlluminance(ill);

        tick::delay(20'000);
    }
}


void batteryTask(void* param)
{
    while (true)
    {
        const auto vbat = sensors::measureBattery(16);
        const auto vbat_pct = battery_mv_to_zigbee_percent(vbat, BatteryType::LiIon1S);
        ESP_LOGI(TAG, "Battery voltage %lu mV (%u %%)", vbat, vbat_pct);
        zigbee::updateVbat(vbat, vbat_pct);

        tick::delay(15 * 60 * 1000);
    }
}
