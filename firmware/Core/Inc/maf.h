/*
 * maf.h
 *
 *  Created on: Nov 20, 2023
 *      Author: Heinrich
 */

#ifndef INC_MAF_H_
#define INC_MAF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct __MAF
{
	uint16_t maf_in_voltage;
	uint16_t maf_in_airmass;
	uint16_t maf_out_voltage;
} MAF[2];

uint16_t interpolate_voltage2airMass (uint16_t inputValue);
uint16_t interpolate_airMass2voltage (uint16_t inputValue);
uint16_t calculate_dac_value_for_voltage (uint16_t voltage);

#ifdef __cplusplus
}
#endif

#endif /* INC_MAF_H_ */
