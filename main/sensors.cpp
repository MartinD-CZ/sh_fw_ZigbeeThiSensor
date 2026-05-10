#include "sensors.h"

#include "SHT4x.h"
#include "LTR308.h"

#include "mal_assert.h"
#include "mal_tick.h"

#include "esp_adc/adc_oneshot.h"


const Gpio 					sda{GPIO_NUM_10}, scl{GPIO_NUM_11}, ill_int{GPIO_NUM_22}, vbat_en{GPIO_NUM_0};
I2cMaster 					i2c1{I2C_NUM_1, sda, scl};
SHT4x						sht4x{i2c1};
LTR308						ltr308{i2c1};
adc_oneshot_unit_handle_t 	adc1;


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
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, ADC_CHANNEL_0, &config));
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


uint16_t sensors::measureBattery()
{
	vbat_en.setHigh();
	tick::delay(5);

	int raw;
	adc_oneshot_read(adc1, ADC_CHANNEL_0, &raw);
	vbat_en.setLow();
	
	// Convert raw ADC value to millivolts
	uint16_t voltage = (uint16_t)((raw * 1100 * 4 * 2) / 4095);
	return voltage;
}