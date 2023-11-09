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
	uint16_t at24cxx_page_size;		// page size in bytes
	uint16_t at24cxx_page_number;	// page number
} AT24CXX_HandleTypeDef;

/**
  * @brief  calculates remains bytes in the page
  * @param  page_size page size in eeprom chip
  *	@param	size bytes count to write
  *	@param	offset offset in the page
  * @retval HAL_StatusTypeDef
  */
uint16_t bytestowrite (uint16_t page_size, uint16_t size, uint16_t offset);

/**
  * @brief  Checks if eeprom is connected and ready for communication.
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef at24cxx_connected(AT24CXX_HandleTypeDef eeprom_device);

/**
  * @brief  erase the specified page in eeprom
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @param	page page number to be erased, from 0 to PAGE_NUM-1
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef at24cxx_erase_page(AT24CXX_HandleTypeDef eeprom_device, uint16_t page);

/**
  * @brief  Erase the whole eeprom chip
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef at24cxx_erase_chip(AT24CXX_HandleTypeDef eeprom_device);

/**
  * @brief  erase the specified page in eeprom
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @param	page is the number of the start page. from 0 to PAGE_NUM-1
  * @param	offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
  * @param	data is the pointer to the data to write in bytes
  * @param	size is size of data
  * @retval bool status
  */
HAL_StatusTypeDef at24cxx_write(AT24CXX_HandleTypeDef eeprom_device, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

/**
  * @brief  read specified number of bytes from specified page with specified offset
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @param	page is the number of the start page. from 0 to PAGE_NUM-1
  * @param	offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
  * @param	data is the pointer to the data to write in bytes
  * @param	size is size of data
  * @retval bool status
  */
HAL_StatusTypeDef at24cxx_read(AT24CXX_HandleTypeDef eeprom_device, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* INC_AT24CXX_HAL_H_ */
