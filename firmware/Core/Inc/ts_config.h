/*
 * ts_config.h
 *
 *  Created on: Nov 9, 2023
 *      Author: heinrich
 */

#ifndef INC_TS_CONFIG_H_
#define INC_TS_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TS_SIGNATURE_PAGE		0
#define TS_CONFIG_PAGE			5

#ifndef word
  #define word(h, l) ((h << 8) | l) //word() function not defined for this platform in the main library
#endif

typedef struct __TS_Config
{
	uint16_t hfmIN_voltage[10];
	uint16_t hfmIN_airmass[10];
	uint16_t hfmOUT_voltage[10];
	uint16_t hfmOUT_airmass[10];
	uint8_t isChannel1Enabled :1;
	uint8_t isChannel2Enabled :1;
	uint8_t showRealADCVoltages :1;
	uint8_t unused1 :5;
} TS_Config;

typedef struct __TS_Signature
{
	uint8_t signature[27];
	uint8_t version[3];
	uint8_t version_info[34];
} TS_Signature;

/**
  * @brief  creates a initial config for TunerStudio
  * @param  config Pointer to a TS_Config structure that contains
  *                		  the configuration information
  * @retval void
  */
void create_initial_config (TS_Config *config, TS_Signature *signature);
void set_page_value (uint16_t offset, uint16_t value, TS_Config* config);

#ifdef __cplusplus
}
#endif

#endif /* INC_TS_CONFIG_H_ */
