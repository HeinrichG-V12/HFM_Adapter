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
  * @brief  calculates remains bytes in the page
  * @param  page_size page size in eeprom chip
  *	@param	size bytes count to write
  *	@param	offset offset in the page
  * @retval uint16_t remain bytes
  */
uint16_t bytestowrite (uint16_t page_size, uint16_t size, uint16_t offset)
{
	if ((size+offset) < page_size) return size;
	else return (page_size - offset);
}

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
	HAL_StatusTypeDef retVal;

	if (page > eeprom_device.at24cxx_page_number)
	{
		return false;
	}

	// calculate the memory address based on the page number
	int paddrposition = log(eeprom_device.at24cxx_page_size)/log(2);
	uint16_t MemAddress = page << paddrposition;

	uint8_t page_data[eeprom_device.at24cxx_page_size];
	memset(page_data,0xff,eeprom_device.at24cxx_page_size);

	retVal = HAL_I2C_Mem_Write(&eeprom_device.i2c_device, eeprom_device.at24cxx_address, MemAddress, 2, page_data, eeprom_device.at24cxx_page_size, 1000);

	if (retVal == HAL_OK)
	{
		HAL_Delay(5);
	}

	return retVal;
}

/**
  * @brief  Erase the whole eeprom chip
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef at24cxx_erase_chip(AT24CXX_HandleTypeDef eeprom_device)
{
	HAL_StatusTypeDef retVal = HAL_OK;

	uint16_t current_page = 0;

	while (current_page < eeprom_device.at24cxx_page_number && retVal == HAL_OK)
	{
		retVal = at24cxx_erase_page(eeprom_device, current_page);
		current_page++;
	}

	return retVal;
}

/**
  * @brief  write specified number of bytes to specified page with specified offset
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @param	page is the number of the start page. from 0 to PAGE_NUM-1
  * @param	offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
  * @param	data is the pointer to the data to write in bytes
  * @param	size is size of data
  * @retval bool status
  */
HAL_StatusTypeDef at24cxx_write(AT24CXX_HandleTypeDef eeprom_device, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	uint16_t pos = 0, current_page = 0;
	HAL_StatusTypeDef retVal = HAL_OK;

	// Find out the number of bit, where the page addressing starts
	int paddrposition = log(eeprom_device.at24cxx_page_size)/log(2);

	// calculate the start page and the end page
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/eeprom_device.at24cxx_page_size);

	// number of pages to be written
	uint16_t numofpages = (endPage-startPage) + 1;


	while (current_page < numofpages && retVal == HAL_OK)
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(eeprom_device.at24cxx_page_size, size, offset);  // calculate the remaining bytes to be written

		retVal = HAL_I2C_Mem_Write(&eeprom_device.i2c_device, eeprom_device.at24cxx_address, MemAddress, 2, &data[pos], bytesremaining, 1000);  // write the data to the EEPROM
		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   // since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		current_page++;

		if (retVal == HAL_OK)
		{
			HAL_Delay(5);
		}
	}
	return retVal;
}

/**
  * @brief  write a uint16_t value to specified page with specified offset
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @param	page is the number of the start page. from 0 to PAGE_NUM-1
  * @param	offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
  * @param	data is the pointer to the data to write in bytes
  * @param	size is size of data
  * @retval bool status
  */
HAL_StatusTypeDef at24cxx_write_16(AT24CXX_HandleTypeDef eeprom_device, uint16_t page, uint16_t offset, uint16_t data)
{
	uint8_t array[2]={ data >> 8 , data & 0xff};

	return at24cxx_write(eeprom_device, page, offset, (uint8_t *) array, sizeof(array));
}

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
HAL_StatusTypeDef at24cxx_read(AT24CXX_HandleTypeDef eeprom_device, uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
	uint16_t pos = 0, current_page = 0;
	HAL_StatusTypeDef retVal = HAL_OK;
	uint16_t startPage = 0;

	// Find out the number of bit, where the page addressing starts
	int paddrposition = log(eeprom_device.at24cxx_page_size)/log(2);

	// calculate the start page and the end page

	if (offset >= eeprom_device.at24cxx_page_size)
	{
		startPage = page + 1;
		offset = offset - eeprom_device.at24cxx_page_size;
	}
	else
	{
		startPage = page;
	}

	uint16_t endPage = startPage + ((size+offset)/eeprom_device.at24cxx_page_size);

	// number of pages to be written
	uint16_t numofpages = (endPage-startPage) + 1;


	while (current_page < numofpages && retVal == HAL_OK)
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(eeprom_device.at24cxx_page_size, size, offset);  // calculate the remaining bytes to be written

		retVal = HAL_I2C_Mem_Read(&eeprom_device.i2c_device, eeprom_device.at24cxx_address, MemAddress, 2, &data[pos], bytesremaining, 1000);  // write the data to the EEPROM
		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   // since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		current_page++;
	}
	return retVal;
}

/**
  * @brief  read uint16_t value from eeprom chip
  * @param  eeprom_device Pointer to a AT24CXX_HandleTypeDef structure that contains
  *                		  the configuration information for the specified eeprom device.
  * @param	page is the number of the start page. from 0 to PAGE_NUM-1
  * @param	offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
  * @param	data is the pointer to the data to write in bytes
  * @param	size is size of data
  * @retval uint16_t value
  */
uint16_t at24cxx_read_16(AT24CXX_HandleTypeDef eeprom_device, uint16_t page, uint16_t offset)
{
	uint8_t bytes_to_read = 2;
	uint8_t bytes[bytes_to_read];

	if (at24cxx_read(eeprom_device, page, offset, bytes, bytes_to_read) == HAL_OK)
	{
		return ((bytes[0] << 8) | bytes[1]);
	}

	return 0xffff;
}
