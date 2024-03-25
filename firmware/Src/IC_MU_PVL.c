/**
 * @file 	IC_MU_PVL.c
 * @brief	ic-haus ic-MU ic-PVL
 *
 * @author 	Tym Zhu
 */
#include "IC_MU_PVL.h"


/**
 * @brief DMA register define, copied from HAL library.
 *
 */
typedef struct
{
	__IO uint32_t ISR; /*!< DMA interrupt status register */
	__IO uint32_t Reserved0;
	__IO uint32_t IFCR; /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

IC_MU_TypeDef MU;

static inline void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress,
		uint32_t DstAddress, uint32_t DataLength);
static inline uint8_t MU_Read_Status();
//static inline void MU_Tranceive();
static inline void MU_Start_Tranceive();
static inline void MU_Wait_Tranceive();
static inline void Delay_500ns();

/**
 * @brief Read position
 *
 * @param MT multi-turn value
 * @param ST single-turn value
 */
void MU_Read_Posi(int32_t *MT, int32_t *ST)
{
	MU_Start_Read_Posi();
	MU_Wait_Read_Posi(MT, ST);
	return;
}
/**
 * @brief start position read
 *
 */
void MU_Start_Read_Posi()
{
	MU.TX_Buff[0] = MU_OP_SDAD_TRANSMIT;
	memset(&(MU.TX_Buff[1]), 0x00, MU_SDAD_SIZE);

	MU_Start_Tranceive(MU_SDAD_SIZE + 1);
}
/**
 * @brief Wait for position read to complete
 *
 * @param MT multi-turn value
 * @param ST single-turn value
 */
void MU_Wait_Read_Posi(int32_t *MT, int32_t *ST)
{
	int32_t MU_OUT;

	MU_Wait_Tranceive();

#if (MU_SDAD_SIZE == 4)
	MU_OUT = __REV(*(int32_t*) (&MU.RX_Buff[1]));

	*MT = (int32_t) (MU_OUT & (~(((1U) << MU_ST_BIT_SIZE) - 1)))
			/ (int32_t) ((1U) << MU_ST_BIT_SIZE);

//	*MT = MU_OUT / (int32_t)((1U) << MU_ST_BIT_SIZE);
	*ST = MU_OUT & (((1U) << MU_ST_BIT_SIZE) - 1);
#else
	static_assert(0, "MHM_MT_SIZE invalid!");
#endif

	return;
}
/**
 * @brief read register
 *
 * @param adr address
 * @return	  data
 */
uint8_t MU_Read_Reg(uint8_t adr)
{
	MU.TX_Buff[0] = MU_OP_READ_REG_SGL;
	MU.TX_Buff[1] = adr;
	do
	{
		MU_Start_Tranceive(2);
		MU_Wait_Tranceive();
		/* wait while busy */
		while (MU_Read_Status() & MU_SPI_BUSY)
			;

	} while (MU.RX_Buff[1] & MU_SPI_FAIL);

	if ((MU.RX_Buff[1] & MU_SPI_DISMISS) || (MU.RX_Buff[1] & MU_SPI_ERROR))
	{
		/* error handling */
		return MU.RX_Buff[2];
	}
	else
		return MU.RX_Buff[2];
}
/**
 * @brief write data to register
 *
 * @param adr address
 * @param dat data
 */
void MU_Write_Reg(uint8_t adr, uint8_t dat)
{
	MU.TX_Buff[0] = MU_OP_WRITE_REG_SGL;
	MU.TX_Buff[1] = adr;
	MU.TX_Buff[2] = dat;
	do
	{
		MU_Start_Tranceive(3);
		MU_Wait_Tranceive();
		/* wait while busy */
		while (MU_Read_Status() & MU_SPI_BUSY)
			;

	} while (MU.RX_Buff[1] & MU_SPI_FAIL);

	if ((MU.RX_Buff[1] & MU_SPI_DISMISS) || (MU.RX_Buff[1] & MU_SPI_ERROR))
	{
		/* error handling */
		return;
	}
	else
		return;

}
/**
 * @brief read status register
 *
 * @return status register
 */
static inline uint8_t MU_Read_Status()
{
	MU.TX_Buff[0] = MU_OP_READ_REG_STAT;
	MU.TX_Buff[1] = 0;
	MU.TX_Buff[2] = 0;

	Delay_500ns();

	MU_Start_Tranceive(3);
	MU_Wait_Tranceive();

	return MU.RX_Buff[1];

}

//static inline void MU_Tranceive(uint32_t DataLength)
//{
//	uint32_t i;
//	
////	
////	Delay_500ns();
//	
//	SPI_MHM_EN();	//CS Pin
//	
//	for (i = 0; i<DataLength; i++)
//	{
//		USART_EXT_ENC->DR = (uint8_t)(__RBIT(MU.TX_Buff[i])>>24);
//		while((__HAL_USART_GET_FLAG(&HUSART_EXT_ENC, USART_FLAG_RXNE) ? SET : RESET) == RESET);
//		MU.RX_Buff[i] = (uint8_t)(__RBIT(USART_EXT_ENC->DR)>>24);
//	}
//	
//	
//	SPI_MHM_DIS();	//CS Pin
//	
//	
//	return;
//
//}
/**
 * @brief start SPI DMA tranceive
 *
 * @param DataLength size
 */
static inline void MU_Start_Tranceive(uint32_t DataLength)
{

	SPI_MU_EN();	//CS	

	/*** Enable Rx DMA Request ***/
	SET_BIT(HSPI_MU.Instance->CR2, SPI_CR2_RXDMAEN);

	/*** Enable RX DMA ***/
	/* Configure the source, destination address and the data length */
	DMA_SetConfig(&HDMA_MU_RX, (uint32_t) &(HSPI_MU.Instance->DR),
			(uint32_t) MU.RX_Buff, DataLength);
	/* Enable the Peripheral */
	__HAL_DMA_ENABLE(&HDMA_MU_RX);
	/*** Enable TX DMA ***/
	/* Configure the source, destination address and the data length */
	DMA_SetConfig(&HDMA_MU_TX, (uint32_t) MU.TX_Buff,
			(uint32_t) &(HSPI_MU.Instance->DR), DataLength);
	/* Enable the Peripheral */
	__HAL_DMA_ENABLE(&HDMA_MU_TX);

	/*** Enable Tx DMA Request ***/
	SET_BIT(HSPI_MU.Instance->CR2, SPI_CR2_TXDMAEN);
	/*** Enable SPI ***/
	__HAL_SPI_ENABLE(&HSPI_MU);

}
/**
 * @brief wait SPI DMA transceive
 *
 */
static inline void MU_Wait_Tranceive()
{
	DMA_Base_Registers *regs_TX =
			(DMA_Base_Registers*) HDMA_MU_TX.StreamBaseAddress;
	DMA_Base_Registers *regs_RX =
			(DMA_Base_Registers*) HDMA_MU_RX.StreamBaseAddress;
	/* Wait then Clear the transfer complete flag */
	while ((regs_TX->ISR & (DMA_FLAG_TCIF0_4 << HDMA_MU_TX.StreamIndex))
			== RESET)
		;
	regs_TX->IFCR = DMA_FLAG_TCIF0_4 << HDMA_MU_TX.StreamIndex;
	while ((regs_RX->ISR & (DMA_FLAG_TCIF0_4 << HDMA_MU_RX.StreamIndex))
			== RESET)
		;
	regs_RX->IFCR = DMA_FLAG_TCIF0_4 << HDMA_MU_RX.StreamIndex;

	/* Disable the Peripheral */
	__HAL_DMA_DISABLE(&HDMA_MU_TX);
	__HAL_DMA_DISABLE(&HDMA_MU_RX);

	/* Disable SPI */
	__HAL_SPI_DISABLE(&HSPI_MU);

	/* Disable Rx/Tx DMA Request (done by default to handle the case master rx direction 2 lines) */
	CLEAR_BIT(HSPI_MU.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

	SPI_MU_DIS();	//CS
}

/**
 * @brief  Sets the DMA Transfer parameter.
 * @param  hdma       pointer to a DMA_HandleTypeDef structure that contains
 *                     the configuration information for the specified DMA Stream.
 * @param  SrcAddress The source memory Buffer address
 * @param  DstAddress The destination memory Buffer address
 * @param  DataLength The length of data to be transferred from source to destination
 * @retval HAL status
 */
static inline void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress,
		uint32_t DstAddress, uint32_t DataLength)
{
	/* Clear DBM bit */
	hdma->Instance->CR &= (uint32_t) (~DMA_SxCR_DBM);

	/* Configure DMA Stream data length */
	hdma->Instance->NDTR = DataLength;

	/* Memory to Peripheral */
	if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
	{
		/* Configure DMA Stream destination address */
		hdma->Instance->PAR = DstAddress;

		/* Configure DMA Stream source address */
		hdma->Instance->M0AR = SrcAddress;
	}
	/* Peripheral to Memory */
	else
	{
		/* Configure DMA Stream source address */
		hdma->Instance->PAR = SrcAddress;

		/* Configure DMA Stream destination address */
		hdma->Instance->M0AR = DstAddress;
	}
}
/**
 * @brief delay 500ns
 *
 */
static inline void Delay_500ns() //500ns
{
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();//7
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
}





