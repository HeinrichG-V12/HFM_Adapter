/*
 * at24cxx_hal.c
 *
 *  Created on: Nov 7, 2023
 *      Author: Heinrich
 */


#include <string.h>
#include <math.h>
#include "at24cxx_hal.h"

/**
  * @brief  Checks if eeprom is connected and ready for communication.
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef at24cxx_connected(AT24CXX_HandleTypeDef eeprom_device)
{
	return HAL_I2C_IsDeviceReady(&eeprom_device.i2c_device, eeprom_device.at24cxx_address, 2, 100);
}

/**
  * @brief  erase the specified page in eeprom
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @param	page page number to be erased
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef at24cxx_erase_page(AT24CXX_HandleTypeDef eeprom_device, uint16_t page)
{
	if (page > eeprom_device.at24cxx_page_number)
	{
		return false;
	}

	// calculate the memory address based on the page number
	int paddrposition = log(eeprom_device.at24cxx_page_size)/log(2);
	uint16_t MemAddress = page << paddrposition;

	uint8_t page_data[eeprom_device.at24cxx_page_size];
	memset(page_data,0xff,eeprom_device.at24cxx_page_size);

	return HAL_I2C_Mem_Write(&eeprom_device.i2c_device, eeprom_device.at24cxx_address, MemAddress, 2, page_data, eeprom_device.at24cxx_page_size, 1000);
}

/**
  * @brief  Erase the whole eeprom chip
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef at24cxx_erase_chip(AT24CXX_HandleTypeDef eeprom_device)
{
	HAL_StatusTypeDef retVal;

	retVal = HAL_OK;

	return retVal;
}
