/**
 * @file 	Flash.c
 * @brief 	Saving configurations in flash
 *
 * @author 	Tym Zhu
 */
#include "Flash.h"

/**
 * @brief Calculate CRC
 *
 * @param p		pointer
 * @param size	size
 * @return
 */
uint32_t CRC_Calc(uint32_t *p, uint32_t size)
{
	return HAL_CRC_Calculate(&hcrc, p, size);
}

/**
 * @brief Save motor config in flash
 *
 * @param pMotor_Config
 * @return
 */
uint8_t Write_Motor_Config(Motor_Config_TypeDef *pMotor_Config)
{
	uint32_t Config_CRC = CRC_Calc((uint32_t*) pMotor_Config, MOTOR_CONFIG_SIZE);
	uint32_t addr;

	HAL_FLASH_Unlock();
	/*Erase sector first*/
	FLASH_Erase_Sector(CONFIG_SECTOR, FLASH_VOLTAGE_RANGE_3);
	/*Program CRC*/
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CONFIG_CRC_ADD, Config_CRC);
	/*Program Config*/
	for (addr = 0; addr < 4 * MOTOR_CONFIG_SIZE; addr += 4)
	{
		//printf("ADDR:%x DATA:%x\n",CONFIG_ADD+addr, *((uint32_t*)((uint32_t)pMotor_Config+addr)));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CONFIG_ADD + addr,
				*((uint32_t*) ((uint32_t) pMotor_Config + addr)));
	}

	HAL_FLASH_Lock();
	return ((*(uint32_t*) CONFIG_CRC_ADD != Config_CRC)
			|| (CRC_Calc((uint32_t*) CONFIG_ADD, MOTOR_CONFIG_SIZE)
					!= Config_CRC));
}

