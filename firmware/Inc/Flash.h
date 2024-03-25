/**
 * @file 	Flash.h
 * @brief 	Header for Flash.c
 *
 * @author 	Tym Zhu
 */
#ifndef FLASH_H
#define FLASH_H

#include "main.h"
#include "Config.h"

/** STM32F446 Flash layout
 * Main memory
 * Sector 0 0x0800 0000 - 0x0800 3FFF    16 Kbytes
 * Sector 1 0x0800 4000 - 0x0800 7FFF    16 Kbytes
 * Sector 2 0x0800 8000 - 0x0800 BFFF    16 Kbytes
 * Sector 3 0x0800 C000 - 0x0800 FFFF    16 Kbytes
 * Sector 4 0x0801 0000 - 0x0801 FFFF    64 Kbytes
 * Sector 5 0x0802 0000 - 0x0803 FFFF   128 Kbytes
 * Sector 6 0x0804 0000 - 0x0805 FFFF   128 Kbytes
 * Sector 7 0x0806 0000 - 0x0807 FFFF   128 Kbytes
 */

/** By default without changing option bytes, STM32 starts at
 *  0x0800 0000. Sector 0 can be used for bootloader.
 *  Actual program starts at Sector 5. Sector 1~4 can be used
 *  for data storage.
 */
/**	stm32f446xx_flash_Sector5.icf
 * Vector Table: 	0x0800 0000 (Sector 0)
 *
 * Config Data: 	0x0800 4000 (Sector 1)
 * Cal Data: 		0x0800 8000 (Sector 2)
 *
 * ROM Code:		0x0802 0000 (Sector 5~)
 */


/**
 * @name Flash address defines
 * @{
 */
#define CONFIG_SECTOR	FLASH_SECTOR_1
#define CONFIG_CRC_ADD	0x08004000U
#define CONFIG_ADD		0x08004004U

#define CAL_SECTOR		FLASH_SECTOR_2
#define CAL_CRC_ADDR	0x08008000U
#define CAL_ADDR		0x08008004U
/**
 * @}
 */

extern uint32_t CRC_Calc(uint32_t *p, uint32_t size);

extern uint8_t Write_Motor_Config(Motor_Config_TypeDef *pMotor_Config);

#endif
