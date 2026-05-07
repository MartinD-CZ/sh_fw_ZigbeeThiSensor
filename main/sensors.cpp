#include "sensors.h"

#include "SHT4x.h"
#include "LTR308.h"

#include "mal_assert.h"
#include "mal_tick.h"


const Gpio 		sda{GPIO_NUM_10}, scl{GPIO_NUM_11};
I2cMaster 		i2c1{I2C_NUM_1, sda, scl};
SHT4x			sht4x{i2c1};
LTR308			ltr308{i2c1};


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


etl::pair<int32_t, int32_t> sensors::measureThermHum()
{
	return sht4x.readInt();
		
	/*DataTypeTemperature temp{(int16_t)th.first};
	msg.addData(&temp);
	DataTypeHumidity rh{(uint16_t)(th.second / 10)};
	msg.addData(&rh);*/

	//LOGT("Temperature %u.%02u °C, humidity %u.%02u %%\n", th.first / 100, th.first % 100, th.second / 100, th.second % 100);
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
	/*DataTypeIllum illuminance{ill};
	msg.addData(&illuminance);*/

	//LOGT("Illuminance %lu.%03lu lx\n", ill / 1000, ill % 1000);
}


void sensors::measureBattery()
{
	

	//LOGT("Battery voltage %u mV\n", bat.data);
}