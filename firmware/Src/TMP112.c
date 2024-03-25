/**
 * @file 	TMP112.c
 * @brief 	TMP112 temperature sensor
 *
 * @author 	Tym Zhu
 */
#include "TMP112.h"

/**
 * @brief read temperature from sensor
 *
 * @param Addr  device address
 * @param pData data
 * @param EM	EM mode?
 * @return		HAL_OK?
 */
HAL_StatusTypeDef TMP112_ReadTemp(uint8_t Addr, float32_t *pData, uint8_t EM)
{
	HAL_StatusTypeDef Status;
	uint16_t TempU16;
	//float32_t	TempF32;

	//HAL_Lib reading 2 bytes temprature register
	Status = HAL_I2C_Mem_Read(&HI2C_TMP112, Addr, 0, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &TempU16, 2, 2);

	if (Status != HAL_OK)
	{
		return Status;
	}
	else
	{
		TempU16 = __REV16(TempU16);	//reverse bytes
		if (EM == TMP112_EXTENDED_MODE)
			*pData = ((float32_t) (*((int16_t*) (&TempU16)))) * 0.0078125f;
		else
			*pData = ((float32_t) (*((int16_t*) (&TempU16)))) * 0.00390625f;
	}
	return HAL_OK;
}
/**
 * @brief write config
 *
 * @param Addr device address
 * @param CR   CR
 * @param EM   EM
 * @return	   HAL_OK?
 */
HAL_StatusTypeDef TMP112_WriteConfig(uint8_t Addr, uint8_t CR, uint8_t EM)
{
	uint8_t TMP_Config[] =
	{ 0x60, 0x20 };

	TMP_Config[1] |= (CR << 6);		//Conversion rate
	if (EM)
		TMP_Config[1] |= 0x10;	//Extended mode

	return HAL_I2C_Mem_Write(&HI2C_TMP112, Addr, 1, I2C_MEMADD_SIZE_8BIT,
			TMP_Config, 2, 2);

}

