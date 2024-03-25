/**
 * @file 	TMP112.h
 * @brief 	Header for TMP112.c
 *			* 2.85MHz max SCL
 *			* TMP102 Accuracy Without Calibration:
 *				– 2.0°C (max) from –25°C to 85°C
 *				– 3.0°C (max) from –40°C to 125°C
 *			* TMP112A Accuracy Without Calibration :
 *				– 0.5°C (max) from 0°C to +65°C (3.3 V)
 *				– 1.0°C (max) from –40°C to +125°C
 *			* TMP112B Accuracy Without Calibration :
 *				– 0.5°C (max) from 0°C to +65°C (1.8 V)
 *				– 1.0°C (max) from –40°C to +125°C
 *			* TMP112N Accuracy Without Calibration:
 *				– 1.0°C (max) from –40°C to +125°C
 * @author 	Tym Zhu
 */
#ifndef TMP112_H
#define TMP112_H

#include "main.h"
//#include "Config.h"

/**
 * @name TMP112 defines
 * @{
 */
#define TMP112_ADDR0	0x90
#define TMP112_ADDR1	0x92
#define TMP112_ADDR2	0x94
#define TMP112_ADDR3	0x96

#define TMP112_025HZ	0x00
#define TMP112_1HZ		0x01
#define TMP112_4HZ		0x02
#define TMP112_8HZ		0x03

#define TMP112_NORMAL_MODE		0x00
#define TMP112_EXTENDED_MODE	0x01
/**
 * @}
 */


extern I2C_HandleTypeDef hi2c1;

extern HAL_StatusTypeDef TMP112_ReadTemp(uint8_t Addr, float32_t* pData, uint8_t EM);
extern HAL_StatusTypeDef TMP112_WriteConfig(uint8_t Addr,uint8_t CR ,uint8_t EM);













#endif
