#include "onboard_led.h"

#include "mal_gpio.h"

#include "esp_check.h"
#include "esp_pm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"


namespace
{
    const Gpio led{GPIO_NUM_12};
    TimerHandle_t blinkTimer = nullptr;
    SemaphoreHandle_t mutex = nullptr;
    esp_pm_lock_handle_t pmLock = nullptr;

    TickType_t onTicks = 1;
    TickType_t offTicks = 1;
    bool blinking = false;
    bool ledOn = false;
    bool pmLockHeld = false;


    TickType_t msToTicks(uint32_t ms)
    {
        const TickType_t ticks = pdMS_TO_TICKS(ms);
        return ticks == 0 ? 1 : ticks;
    }


    void acquirePmLock()
    {
        if (!pmLockHeld)
        {
            ESP_ERROR_CHECK(esp_pm_lock_acquire(pmLock));
            pmLockHeld = true;
        }
    }


    void releasePmLock()
    {
        if (pmLockHeld)
        {
            ESP_ERROR_CHECK(esp_pm_lock_release(pmLock));
            pmLockHeld = false;
        }
    }


    void setLedOn()
    {
        acquirePmLock();
        led.setLow();
        ledOn = true;
    }


    void setLedOff()
    {
        led.setHigh();
        ledOn = false;
        releasePmLock();
    }


    void timerCallback(TimerHandle_t timer)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);

        if (blinking)
        {
            if (ledOn)
            {
                setLedOff();
                ESP_ERROR_CHECK(xTimerChangePeriod(timer, offTicks, 0) == pdPASS ? ESP_OK : ESP_FAIL);
            }
            else
            {
                setLedOn();
                ESP_ERROR_CHECK(xTimerChangePeriod(timer, onTicks, 0) == pdPASS ? ESP_OK : ESP_FAIL);
            }
        }

        xSemaphoreGive(mutex);
    }
}


void onboard_led::init()
{
    if (mutex == nullptr)
        mutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(mutex != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    if (blinkTimer == nullptr)
        blinkTimer = xTimerCreate("led_blink", 1, pdFALSE, nullptr, timerCallback);

    ESP_ERROR_CHECK(blinkTimer != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    if (pmLock == nullptr)
        ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "onboard_led", &pmLock));

    xSemaphoreTake(mutex, portMAX_DELAY);
    led.initOutput(Gpio::Speed::SLOW, Gpio::Output::HIGH, Gpio::Type::OPEN_DRAIN);
    blinking = false;
    ledOn = false;
    releasePmLock();
    xSemaphoreGive(mutex);
}


void onboard_led::on()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    blinking = false;
    ESP_ERROR_CHECK(xTimerStop(blinkTimer, 0) == pdPASS ? ESP_OK : ESP_FAIL);
    setLedOn();
    xSemaphoreGive(mutex);
}


void onboard_led::off()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    blinking = false;
    ESP_ERROR_CHECK(xTimerStop(blinkTimer, 0) == pdPASS ? ESP_OK : ESP_FAIL);
    setLedOff();
    xSemaphoreGive(mutex);
}


void onboard_led::blinkOn(uint32_t onTimeMs, uint32_t offTimeMs)
{
    xSemaphoreTake(mutex, portMAX_DELAY);

    onTicks = msToTicks(onTimeMs);
    offTicks = msToTicks(offTimeMs);
    blinking = true;
    setLedOn();
    ESP_ERROR_CHECK(xTimerChangePeriod(blinkTimer, onTicks, 0) == pdPASS ? ESP_OK : ESP_FAIL);

    xSemaphoreGive(mutex);
}


void onboard_led::blinkOff()
{
    off();
}
