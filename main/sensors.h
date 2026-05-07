#pragma once


#include "etl/utility.h"


namespace sensors
{
	void initI2c();
	void initThermHum();
	void initIllum();

	etl::pair<int32_t, int32_t> measureThermHum();
	void startIllumMeasurement();
	uint32_t getIllum();

	void measureBattery(); 
}