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

typedef struct __TS_Config
{
	uint16_t configuration_length;
} TS_Config;


#ifdef __cplusplus
}
#endif

#endif /* INC_TS_CONFIG_H_ */
