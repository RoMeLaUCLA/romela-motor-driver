/**
 * @file 	MAX31855.c
 * @brief 	MAX31855 thermocouple
 *
 * @author 	Tym Zhu
 */

#include "MAX31855.h"

MAX31855_TypeDef TC_Reading;

/**
 * @brief read thermocouple and internal temperature
 *
 * @param Result ponter to MAX31855
 * @return fault
 */
uint8_t MAX31855_Read(MAX31855_TypeDef *Result)
{
	uint16_t MAX31855_RX[2];
	uint16_t Uint_ThC, Uint_In;

	SPI_MAX31855_EN();			//CS
	//__HAL_SPI_ENABLE(&HSPI_MAX31855);

	HSPI_MAX31855.Instance->DR = 0;
	while (!(__HAL_SPI_GET_FLAG(&HSPI_MAX31855, SPI_FLAG_RXNE)))
		;	//Check RXNE flag
	MAX31855_RX[0] = HSPI_MAX31855.Instance->DR;
	HSPI_MAX31855.Instance->DR = 0;
	while (!(__HAL_SPI_GET_FLAG(&HSPI_MAX31855, SPI_FLAG_RXNE)))
		;	//Check RXNE flag
	MAX31855_RX[1] = HSPI_MAX31855.Instance->DR;

	//__HAL_SPI_DISABLE(&HSPI_MAX31855);
	SPI_MAX31855_DIS();//CS

	Uint_ThC = (MAX31855_RX[0] >> 2) + (MAX31855_RX[0] >> 15) * 0xC000;
	Result->tc = (float32_t) (*(int16_t*) (&Uint_ThC)) * 0.25f;
	Uint_In = (MAX31855_RX[1] >> 4) + (MAX31855_RX[1] >> 15) * 0xF000;
	Result->in = (float32_t) (*(int16_t*) (&Uint_In)) * 0.0625f;

	Result->Fault = MAX31855_RX[1] & 0x7;	//Fault Code 1:OC 2:GND 4:Vcc

	return (Result->Fault);
}

