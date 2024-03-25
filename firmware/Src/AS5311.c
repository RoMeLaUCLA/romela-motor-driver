/**
 * @file 	AS5311.c
 * @brief
 *
 * @author 	Tym Zhu
 */
#include "AS5311.h"


/*	Simple address read for AS5311  */
uint16_t AS5311_Simple_Read()
{
	uint16_t SSIRCV=0;

	
	SSI_EXT_ENC_EN();	//CS Pin
	HAL_USART_Receive(&HUSART_EXT_ENC, (uint8_t *)(&SSIRCV), 2, 5);
	//HAL_USART_Receive_DMA(&HUSART_EXT_ENC, (uint8_t *)(&SSIRCV), 1);
	//while (HUSART_EXT_ENC.State!=HAL_USART_STATE_READY);
	SSI_EXT_ENC_DIS();	//CS Pin
	
	SSIRCV=(__RBIT(SSIRCV))>>16;
	

	return (SSIRCV>>3)&0x0FFF;
}
uint16_t AS5311_Simple_Read_Fast()
{
	uint16_t SSIRCV=0;
	
	SSI_EXT_ENC_EN();	//CS Pin
  	SET_BIT(USART_EXT_ENC->CR2, USART_CR2_CPHA);
	/* Send Dummy Byte in order to generate clock */
	USART_EXT_ENC->DR = 0xFF;
	while((__HAL_USART_GET_FLAG(&HUSART_EXT_ENC, USART_FLAG_RXNE) ? SET : RESET) == RESET);
	*((uint8_t *)(&SSIRCV))=(uint8_t)(USART_EXT_ENC->DR);
	
	CLEAR_BIT(USART_EXT_ENC->CR2, USART_CR2_CPHA);
	USART_EXT_ENC->DR = 0xFF;
	while((__HAL_USART_GET_FLAG(&HUSART_EXT_ENC, USART_FLAG_RXNE) ? SET : RESET) == RESET);
	*(((uint8_t *)(&SSIRCV))+1)=(uint8_t)(USART_EXT_ENC->DR);
	SSI_EXT_ENC_DIS();	//CS Pin
	
	SSIRCV=(__RBIT(SSIRCV))>>16;
	SSIRCV=(SSIRCV>>3)&0x0FFF;
	return SSIRCV;
}

uint16_t AS5047_Simple_Read_Fast()
{
	uint16_t SSIRCV=0;
	
	SSI_EXT_ENC_EN();	//CS Pin
  	SET_BIT(USART_EXT_ENC->CR2, USART_CR2_CPHA);
	/* Send Dummy Byte in order to generate clock */
	USART_EXT_ENC->DR = 0xFF;
	while((__HAL_USART_GET_FLAG(&HUSART_EXT_ENC, USART_FLAG_RXNE) ? SET : RESET) == RESET);
	*((uint8_t *)(&SSIRCV))=(uint8_t)(USART_EXT_ENC->DR);
	
	CLEAR_BIT(USART_EXT_ENC->CR2, USART_CR2_CPHA);
	USART_EXT_ENC->DR = 0xFF;
	while((__HAL_USART_GET_FLAG(&HUSART_EXT_ENC, USART_FLAG_RXNE) ? SET : RESET) == RESET);
	*(((uint8_t *)(&SSIRCV))+1)=(uint8_t)(USART_EXT_ENC->DR);
	SSI_EXT_ENC_DIS();	//CS Pin
	
	SSIRCV=(__RBIT(SSIRCV))>>16;
	SSIRCV=(SSIRCV&0x3FFF);
	return SSIRCV;
}

uint16_t RGB_Enc_Read()
{
	uint16_t SSIRCV=0;
	
	SET_BIT(USART_EXT_ENC->CR2, USART_CR2_CPHA);	
	/* Send Dummy Byte in order to generate clock */
	USART_EXT_ENC->DR = 0xFF;
	while((__HAL_USART_GET_FLAG(&HUSART_EXT_ENC, USART_FLAG_RXNE) ? SET : RESET) == RESET);
	*((uint8_t *)(&SSIRCV))=(uint8_t)(USART_EXT_ENC->DR);
	
	CLEAR_BIT(USART_EXT_ENC->CR2, USART_CR2_CPHA);
	USART_EXT_ENC->DR = 0xFF;
	while((__HAL_USART_GET_FLAG(&HUSART_EXT_ENC, USART_FLAG_RXNE) ? SET : RESET) == RESET);
	*(((uint8_t *)(&SSIRCV))+1)=(uint8_t)(USART_EXT_ENC->DR);
	
	SSIRCV=(__RBIT(SSIRCV))>>16;
	
	return SSIRCV;
}

//	SSI_EXT_ENC_EN();	//CS Pin
//	/* Send Dummy Byte in order to generate clock */
//	USART_EXT_ENC->DR = 0xFF;
//	while((__HAL_USART_GET_FLAG(&HUSART_EXT_ENC, USART_FLAG_RXNE) ? SET : RESET) == RESET);
//	*((uint8_t *)(&Motor1.SSIRCV+1))=(uint8_t)USART_EXT_ENC->DR;
//	USART_EXT_ENC->DR = 0xFF;
//	while((__HAL_USART_GET_FLAG(&HUSART_EXT_ENC, USART_FLAG_RXNE) ? SET : RESET) == RESET);
//	*((uint8_t *)(&Motor1.SSIRCV))=(uint8_t)USART_EXT_ENC->DR;
//	SSI_EXT_ENC_DIS();	//CS Pin











