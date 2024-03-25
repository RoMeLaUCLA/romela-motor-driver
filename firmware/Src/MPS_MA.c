/**
 * @file 	MPS_MA.c
 * @brief 	MPS MA external encoder
 *
 * @author 	Tym Zhu
 */
#include "MPS_MA.h"

#include "util.h"

static inline uint16_t MA_TransmitReceive_U16(uint16_t dat);

/**
 * @brief Simple data read
 *
 * @return data
 */
uint16_t MA_Read_Posi_HAL()
{
	uint16_t SSIRCV=0;
	uint16_t SSITX=0;
	
	SSI_EXT_ENC_EN();	//CS Pin
	//HAL_USART_Receive(&HUSART_EXT_ENC, (uint8_t *)(&SSIRCV), 2, 5);
	HAL_USART_TransmitReceive(&HUSART_EXT_ENC, (uint8_t*)&SSITX, (uint8_t*)&SSIRCV, 2, 5);
	SSI_EXT_ENC_DIS();	//CS Pin
	
	SSIRCV=__RBIT(SSIRCV)>>16;
	
	return SSIRCV;
}

/**
 * @brief read position
 *
 * @return position
 */
uint16_t MA_Read_Posi()
{
	return MA_TransmitReceive_U16(0x0000);
}

/**
 * @brief read register
 *
 * @param addr	address
 * @return		data
 */
uint8_t MA_Read_Reg(uint8_t addr)
{
	uint16_t RCV;
	MA_TransmitReceive_U16( MA_CMD_READ | ((uint16_t)addr<<8) );
	/* delay for reg read */
	DelayMicros(1);
	
	RCV = MA_TransmitReceive_U16(0x0000);
	return (RCV>>8);
}

/**
 * @brief write and confirm register
 *
 * @param addr	address
 * @param dat	data
 * @return		HAL_OK?
 */
HAL_StatusTypeDef MA_Write_Confirm_Reg(uint8_t addr, uint8_t dat)
{
	uint16_t RCV;
	MA_TransmitReceive_U16( (MA_CMD_WRITE | ((uint16_t)addr<<8)) + dat );
	/* delay for reg write */
	DelayMicros(20000);
	
	RCV = MA_TransmitReceive_U16(0x0000);
	if ((RCV>>8)==dat)
		return HAL_OK;
	else
		return HAL_ERROR;
}

/**
 * @brief Tranceive uint16 data
 *
 * @param dat	tx data
 * @return		rx data
 */
static inline uint16_t MA_TransmitReceive_U16(uint16_t dat)
{
	uint16_t RCV = 0;
	
	SSI_EXT_ENC_EN();	//CS Pin
	
	USART_EXT_ENC->DR = (uint8_t)(__RBIT(dat)>>16);
	while((__HAL_USART_GET_FLAG(&HUSART_EXT_ENC, USART_FLAG_RXNE) ? SET : RESET) == RESET);
	*((uint8_t *)(&RCV))=(uint8_t)(USART_EXT_ENC->DR);
	
	USART_EXT_ENC->DR = (uint8_t)(__RBIT(dat)>>24);
	while((__HAL_USART_GET_FLAG(&HUSART_EXT_ENC, USART_FLAG_RXNE) ? SET : RESET) == RESET);
	*(((uint8_t *)(&RCV))+1)=(uint8_t)(USART_EXT_ENC->DR);
	
	SSI_EXT_ENC_DIS();	//CS Pin
	
	RCV = __RBIT(RCV)>>16;
	
	return RCV;
}

