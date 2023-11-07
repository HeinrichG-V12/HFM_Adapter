/*
 * at24cxx_hal.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Heinrich
 */

#ifndef INC_AT24CXX_HAL_H_
#define INC_AT24CXX_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx_hal.h"

typedef struct __AT24CXX_HandleTypeDef
{
	I2C_HandleTypeDef i2c_device;
	uint16_t at24cxx_address;		// chip address
	uint16_t at24cxx_size;			// chip size
	uint16_t at24cxx_page_size;		// page size in bytes
	uint16_t at24cxx_page_number;	// page number
} AT24CXX_HandleTypeDef;


/**
  * @brief  Checks if eeprom is connected and ready for communication.
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @retval bool status
  */
bool at24cxx_connected(AT24CXX_HandleTypeDef eeprom_device);

/**
  * @brief  erase the specified page in eeprom
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @param	page page number to be erased
  * @retval bool status
  */
bool at24cxx_erase_page(AT24CXX_HandleTypeDef eeprom_device, uint16_t page);

/**
  * @brief  Erase the whole eeprom chip
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @retval bool status
  */
bool at24cxx_erase_chip(AT24CXX_HandleTypeDef eeprom_device);


#ifdef __cplusplus
}
#endif

#endif /* INC_AT24CXX_HAL_H_ */
