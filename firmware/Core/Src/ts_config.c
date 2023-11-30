/*
 * ts_config.c
 *
 *  Created on: Nov 9, 2023
 *      Author: heinrich
 */

#include <string.h>
#include "ts_config.h"

TS_Config ts_config;
TS_Signature ts_signature;


/**
  * @brief  creates a initial config for TunerStudio
  * @param  config Pointer to a TS_Config structure that contains
  *                		  the configuration information
  * @retval void
  */
void create_initial_config (TS_Config *config, TS_Signature *signature)
{
	// memset(page_data,0xff,eeprom_device.at24cxx_page_size);
	memcpy(signature->version, "001", sizeof(signature->version));
	memcpy(signature->signature, "Speeduino Dual MAF Adapter", sizeof(signature->signature));
	memcpy(signature->version_info, "Speeduino Dual MAF Adapter v0.0.1", sizeof(signature->version_info));

	config->hfmIN_voltage[0] = 0;
	config->hfmIN_voltage[1] = 1239;
	config->hfmIN_voltage[2] = 1364;
	config->hfmIN_voltage[3] = 1524;
	config->hfmIN_voltage[4] = 1874;
	config->hfmIN_voltage[5] = 2371;
	config->hfmIN_voltage[6] = 2999;
	config->hfmIN_voltage[7] = 3749;
	config->hfmIN_voltage[8] = 4169;
	config->hfmIN_voltage[9] = 4457;

	config->hfmIN_airmass[0] = 0;
	config->hfmIN_airmass[1] = 80;
	config->hfmIN_airmass[2] = 100;
	config->hfmIN_airmass[3] = 150;
	config->hfmIN_airmass[4] = 300;
	config->hfmIN_airmass[5] = 600;
	config->hfmIN_airmass[6] = 1200;
	config->hfmIN_airmass[7] = 2500;
	config->hfmIN_airmass[8] = 3700;
	config->hfmIN_airmass[9] = 4800;

	config->hfmOUT_voltage[0] = 1400;
	config->hfmOUT_voltage[1] = 2367;
	config->hfmOUT_voltage[2] = 2662;
	config->hfmOUT_voltage[3] = 3009;
	config->hfmOUT_voltage[4] = 3442;
	config->hfmOUT_voltage[5] = 4224;
	config->hfmOUT_voltage[6] = 4773;
	config->hfmOUT_voltage[7] = 0;
	config->hfmOUT_voltage[8] = 0;
	config->hfmOUT_voltage[9] = 0;

	config->hfmOUT_airmass[0] = 0;
	config->hfmOUT_airmass[1] = 150;
	config->hfmOUT_airmass[2] = 300;
	config->hfmOUT_airmass[3] = 600;
	config->hfmOUT_airmass[4] = 1200;
	config->hfmOUT_airmass[5] = 2800;
	config->hfmOUT_airmass[6] = 4800;
	config->hfmOUT_airmass[7] = 0;
	config->hfmOUT_airmass[8] = 0;
	config->hfmOUT_airmass[9] = 0;
}

void set_page_value (uint16_t offset, uint16_t value, TS_Config* config)
{
	uint8_t* tmp_config = (uint8_t*)config;

	tmp_config[offset] = (value & 0xff);
	tmp_config[offset+1] = (value >> 8);

	config = (TS_Config*)tmp_config;
}
