/**
 * @file 	DRV8320.c
 * @brief 	Functions for DRV8320
 *
 * @author 	Tym Zhu
 */
#include "DRV8320.h"

#include "util.h"

static __INLINE void Delay_400ns();

/**
 * @brief Write data to device and confirm by reading
 *
 * @param addr	address
 * @param dat	data
 * @return		HAL_OK?
 */
HAL_StatusTypeDef DRV8320_Write_Confirm(uint8_t addr, uint16_t dat)
{

	DRV8320_Write(addr, dat);
	
	if (DRV8320_Read(addr) == dat)
		return HAL_OK;
	else
		return HAL_ERROR;

}

/**
 * @brief Write data
 *
 * @param addr	address
 * @param dat	data
 */
void DRV8320_Write(uint8_t addr, uint16_t dat)
{
	uint16_t DRV8320_TX = (uint16_t)addr*0x800 + (dat&0x7FF);
	/* Delay for 400ns nCS */
	Delay_400ns();
	SPI_DRV8320_EN();	//CS
	//__HAL_SPI_ENABLE(&HSPI_DRV8320);

	HSPI_DRV8320.Instance->DR = DRV8320_TX;
	while (!(__HAL_SPI_GET_FLAG(&HSPI_DRV8320, SPI_FLAG_RXNE)));	//Check RXNE flag
	DRV8320_TX = HSPI_DRV8320.Instance->DR;	//dummy read
	
	//__HAL_SPI_DISABLE(&HSPI_DRV8320);
	SPI_DRV8320_DIS();	//CS

}

/**
 * @brief Read data
 *
 * @param addr  address
 * @return		data
 */
uint16_t DRV8320_Read(uint8_t addr)
{
	uint16_t DRV8320_TX = 0x8000 + (uint16_t)addr*0x800;
	uint16_t DRV8320_RX = 0;
	/* Delay for 400ns nCS */
	Delay_400ns();
	/* Change to low baudrate */
	MODIFY_REG( HSPI_DRV8320.Instance->CR1, SPI_BAUDRATEPRESCALER_256, 
			   								DRV_SPI_READ_PRESCALER);
	SPI_DRV8320_EN();	//CS
	//__HAL_SPI_ENABLE(&HSPI_DRV8320);
	
	HSPI_DRV8320.Instance->DR = DRV8320_TX;
	while (!(__HAL_SPI_GET_FLAG(&HSPI_DRV8320, SPI_FLAG_RXNE)));	//Check RXNE flag
	DRV8320_RX = HSPI_DRV8320.Instance->DR;
	
	//__HAL_SPI_DISABLE(&HSPI_DRV8320);
	SPI_DRV8320_DIS();	//CS
	
	/* Change back to high baudrate */
	MODIFY_REG( HSPI_DRV8320.Instance->CR1, SPI_BAUDRATEPRESCALER_256, 
			   								DRV_SPI_WRITE_PRESCALER);
	
	return (DRV8320_RX&0x7FF);
}

/**
 * @brief Reset device by pulling reset pin
 *
 */
void DRV8320_Reset()
{
	/* An 8 to 40us low pulse can be
	used to reset fault conditions */
	DRV8320_DIS();
	DelayMicros(12);
	DRV8320_EN();
}

/**
 * @brief delay 400ns
 *
 */
static __INLINE void Delay_400ns() //400ns
{
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();
}
