/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */


#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "sensors.h"
#include "zigbee.h"

#include "mal_gpio.h"
#include "mal_tick.h"


const Gpio led{GPIO_NUM_12};

static const char *TAG = "main";


extern "C" void app_main(void)
{
    sensors::initI2c();
    sensors::initThermHum();
    sensors::initIllum();
    
    ESP_LOGI(TAG, "Startup\n");
    
    led.initOutput(Gpio::Speed::SLOW, Gpio::Output::HIGH, Gpio::Type::OPEN_DRAIN);

    ESP_ERROR_CHECK(nvs_flash_init());
    zigbee::startTask();

    while (true)
    {
        //led.setLow();

        const auto th = sensors::measureThermHum();
        ESP_LOGI(TAG, "Temperature %li.%02li °C, humidity %li.%02li %%", th.first / 100, th.first % 100, th.second / 100, th.second % 100);
        zigbee::updateTempHum(th.first, th.second);
        /*sensors::startIllumMeasurement();
        const auto ill = sensors::getIllum();
        ESP_LOGI(TAG, "Illuminance %lu.%03lu lx", ill / 1000, ill % 1000);*/

        //led.setHigh();

        tick::delay(60'000);
    }
}
