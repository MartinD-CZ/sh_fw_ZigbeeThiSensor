#include "sensors.h"

#include "SHT4x.h"
#include "LTR308.h"

#include "mal_assert.h"
#include "mal_tick.h"

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


const Gpio 					sda{GPIO_NUM_10}, scl{GPIO_NUM_11}, ill_int{GPIO_NUM_22}, vbat_en{GPIO_NUM_0};
I2cMaster 					i2c1{I2C_NUM_1, sda, scl};
SHT4x						sht4x{i2c1};
LTR308						ltr308{i2c1};
adc_oneshot_unit_handle_t 	adc1;
adc_cali_handle_t 			adc1_cali;

constexpr static auto VBAT_ADC_CHANNEL = ADC_CHANNEL_0;
constexpr static auto VBAT_ADC_ATTEN = ADC_ATTEN_DB_12;
constexpr static auto VBAT_ADC_WIDTH = ADC_BITWIDTH_DEFAULT;
bool adc1_cali_ready = false;

static const char* TAG = "sensors";


void sensors::initI2c()
{
	i2c1.init(400'000);
}


void sensors::initThermHum()
{
	ASSERT(sht4x.init());
}


/**
 * 
 * @note It is recommended to call this even if illuminance measurement is not needed, because this enforces sensor power down.
 */
void sensors::initIllum()
{
	ASSERT(ltr308.init());			//sensor disabled by default
	ltr308.restart();				//we disable the sensor on first run of the main loop
}


void sensors::initVbatMeasurement()
{
	vbat_en.initOutput();

	adc_oneshot_unit_init_cfg_t init_config1 = {
    	.unit_id = ADC_UNIT_1,
    	.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1));

	adc_oneshot_chan_cfg_t config = {
		.atten = VBAT_ADC_ATTEN,
		.bitwidth = VBAT_ADC_WIDTH,
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, VBAT_ADC_CHANNEL, &config));

	adc_cali_curve_fitting_config_t cali_config = {};
    cali_config.unit_id = ADC_UNIT_1;
    cali_config.chan = VBAT_ADC_CHANNEL;
    cali_config.atten = VBAT_ADC_ATTEN;
    cali_config.bitwidth = VBAT_ADC_WIDTH;

    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali);
    if (err == ESP_OK)
        adc1_cali_ready = true;
    else
	{
        adc1_cali_ready = false;
        ESP_LOGW(TAG, "ADC calibration not available: %s", esp_err_to_name(err));
	}
}


etl::pair<int32_t, int32_t> sensors::measureThermHum()
{
	return sht4x.readInt();
}


void sensors::startIllumMeasurement()
{
	ltr308.enable();
}


uint32_t sensors::getIllum()
{
	while (!ltr308.isDataReady())
		tick::delay(5);
	ltr308.disable();
	
	return ltr308.readMilliLux();
}


uint16_t sensors::measureBattery(size_t numSamples)
{
	vbat_en.setHigh();
	tick::delay(5);

	//throw away first measurement
	int raw, raw_acc = 0;
	adc_oneshot_read(adc1, VBAT_ADC_CHANNEL, &raw);

	for (size_t i = 0; i < numSamples; i++)
	{
		adc_oneshot_read(adc1, VBAT_ADC_CHANNEL, &raw);
		raw_acc += raw;
	}
	raw_acc /= numSamples;
	vbat_en.setLow();
	
	// Convert raw ADC value to millivolts
	 int adc_pin_mv = 0;
	if (adc1_cali_ready)
		ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali, raw_acc, &adc_pin_mv));
	else
        adc_pin_mv = (raw_acc * 1100 * 4) / 4095;

	const uint16_t vbat = adc_pin_mv * 2;
	return vbat;
}