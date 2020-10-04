/*
 * redirect_SWV.c
 *
 *  Created on: Sep 27, 2020
 *      Author: PC User
 */

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include "common.h"

/*******************************************************************************
 * @fn      _write
 * @brief   Retaget printf to SWV ITM data console
 * @param	file
 * 			ptr
 * 			len
 * @return	None
 ******************************************************************************/
uint32_t _write(uint32_t file, char *ptr, uint32_t len)
{
	uint32_t i = 0;
	for(i = 0; i < len; i++)
	{
		ITM_SendChar((*ptr++));
	}
	return len;
}

