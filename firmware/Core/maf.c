/*
 * maf.c
 *
 *  Created on: Nov 20, 2023
 *      Author: Heinrich
 */


#include <math.h>
#include "maf.h"
#include "ts_config.h"

extern TS_Config ts_config;

uint16_t interpolate_voltage2airMass (uint16_t inputValue)
{
	uint16_t retVal = 0;
	uint8_t counter = 0;
	uint8_t lastIndex = sizeof(ts_config.hfmIN_airmass)/2 - 1;
	float gain, offset;

	// check agains both ends

	if (inputValue <= ts_config.hfmIN_voltage[0])
	{
		retVal = ts_config.hfmIN_airmass[0];
	}

	if (inputValue >= ts_config.hfmIN_voltage[lastIndex])
	{
		retVal = ts_config.hfmIN_airmass[lastIndex];
	}

	while (retVal == 0 && counter < lastIndex)
	{
		if (ts_config.hfmIN_voltage[counter] == inputValue)
		{
			retVal = ts_config.hfmIN_airmass[counter];
		}

		else if (ts_config.hfmIN_voltage[counter] <= inputValue && inputValue <= ts_config.hfmIN_voltage[counter+1])
		{
			gain = (float) (ts_config.hfmIN_airmass[counter+1] - ts_config.hfmIN_airmass[counter]) / (ts_config.hfmIN_voltage[counter + 1] - ts_config.hfmIN_voltage[counter]);
			offset = (ts_config.hfmIN_airmass[counter+1] - (ts_config.hfmIN_voltage[counter + 1] * gain));
			retVal = (uint16_t) ((inputValue * gain) + offset);
		}

		counter++;
	}

	return retVal;
}

uint16_t interpolate_airMass2voltage (uint16_t inputValue)
{
	uint16_t retVal = 0;
	uint8_t counter = 0;
	uint8_t lastIndex = sizeof(ts_config.hfmOUT_airmass)/2 - 1;
	float gain, offset;


	// check agains both ends

	if (inputValue <= ts_config.hfmOUT_airmass[0])
	{
		retVal = ts_config.hfmOUT_voltage[0];
	}

	if (inputValue >= ts_config.hfmOUT_airmass[lastIndex])
	{
		retVal = ts_config.hfmOUT_voltage[lastIndex];
	}

	while (retVal == 0 && counter < lastIndex)
	{
		if (ts_config.hfmOUT_airmass[counter] == inputValue)
		{
			retVal = ts_config.hfmOUT_voltage[counter];
		}

		else if (ts_config.hfmOUT_airmass[counter] <= inputValue && inputValue <= ts_config.hfmOUT_airmass[counter+1])
		{
			gain = (float) (ts_config.hfmOUT_voltage[counter+1] - ts_config.hfmOUT_voltage[counter]) / (ts_config.hfmOUT_airmass[counter + 1] -  ts_config.hfmOUT_airmass[counter]);
			offset = (ts_config.hfmOUT_voltage[counter+1] - (ts_config.hfmOUT_airmass[counter + 1] * gain));
			retVal = (uint16_t) ((inputValue * gain) + offset);
		}

		counter++;
	}

	return retVal;
}

uint16_t calculate_dac_value_for_voltage (uint16_t voltage)
{
	float real_voltage = (float) voltage/ 1000 / 1.5151515151;
	float dac_value = roundf(real_voltage*(4096) / 3.3);

	return (uint16_t)dac_value;
}
