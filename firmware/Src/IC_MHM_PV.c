/**
 * @file 	IC_MHM_PV.c
 * @brief 	ic-haus ic-MHM and ic-PV functions
 *
 * @author 	Tym Zhu
 */
#include "IC_MHM_PV.h"


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

IC_MHM_TypeDef MHM;

static inline void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress,
		uint32_t DstAddress, uint32_t DataLength);
static inline void MHM_Start_Tranceive();
static inline void MHM_Wait_Tranceive();
static inline void Delay_500ns();

/**
 * @brief Read position
 *
 * @param MT multi-turn value
 * @param ST single-turn value
 * @return
 */
uint8_t MHM_Read_Posi(int32_t *MT, int32_t *ST)
{
	MHM_Start_ReadPosi();
	return MHM_Wait_Read_Posi(MT, ST);
}

/**
 * @brief start position read
 *
 */
void MHM_Start_ReadPosi()
{
	MHM.TX_Buff[0] = MHM_OP_READ_POS;
	memset(&(MHM.TX_Buff[1]), 0x00, (MHM_MT_SIZE + MHM_ST_SIZE + 1));

	MHM_Start_Tranceive((MHM_MT_SIZE + MHM_ST_SIZE + 2));//1 OP byte + 1 ERR byte = 2
}

/**
 * @brief Wait for position read to complete
 *
 * @param MT multi-turn value
 * @param ST single-turn value
 * @return   error bits
 */
uint8_t MHM_Wait_Read_Posi(int32_t *MT, int32_t *ST)
{
	MHM_Wait_Tranceive();

#if    (MHM_MT_SIZE == 0)
	/* do nothing to *MT */
#elif  (MHM_MT_SIZE == 4)
	*MT = __REV(*(int32_t*) (&MHM.RX_Buff[1]));
#else
	static_assert(0, "MHM_MT_SIZE invalid!");
#endif

	*ST = __REV16(*(uint16_t*) (&MHM.RX_Buff[MHM_MT_SIZE + 1]));	//&0xFFF0;

	/* return nERR nWARN 000000 */
	return MHM.RX_Buff[MHM_MT_SIZE + MHM_ST_SIZE + 1];
}

/**
 * @brief read register
 *
 * @param adr address
 * @return	  data
 */
uint8_t MHM_Read_Reg(uint8_t adr)
{
	MHM.TX_Buff[0] = MHM_OP_READ_REG_CONT;
	MHM.TX_Buff[1] = adr;
	MHM.TX_Buff[2] = 0x00;

	MHM_Start_Tranceive(3);
	MHM_Wait_Tranceive();

	return MHM.RX_Buff[2];
}

/**
 * @brief read multiple registers
 *
 * @param adr   address
 * @param size  number of bytes
 * @param pdata pointer to data
 *
 */
void MHM_Read_Regs(uint8_t adr, uint8_t size, uint8_t* pdata)
{
	MHM.TX_Buff[0] = MHM_OP_READ_REG_CONT;
	MHM.TX_Buff[1] = adr;
	memset(&(MHM.TX_Buff[2]), 0x00, size);

	MHM_Start_Tranceive(size + 2);
	MHM_Wait_Tranceive();

	memcpy(pdata, MHM.RX_Buff + 2, size);
}

/**
 * @brief start register write
 *
 * @param adr address
 * @param dat data
 */
void MHM_Start_Write_Reg(uint8_t adr, uint8_t dat)
{
	MHM.TX_Buff[0] = MHM_OP_WRITE_REG_SGL;
	MHM.TX_Buff[1] = adr;
	MHM.TX_Buff[2] = dat;

	MHM_Start_Tranceive(3);
}
/**
 * @brief wait register write to complete
 *
 */
void MHM_Wait_Write_Reg()
{
	MHM_Wait_Tranceive();
}
/**
 * @brief write data to register
 *
 * @param adr address
 * @param dat data
 */
void MHM_Write_Reg(uint8_t adr, uint8_t dat)
{
	MHM_Start_Write_Reg(adr, dat);
	MHM_Wait_Write_Reg();
}
/**
 * @brief write multiple registers
 *
 * @param adr  address
 * @param pdat pointer to data
 * @param size size
 */
void MHM_Write_Regs(uint8_t adr, uint8_t *pdat, uint8_t size)
{
	MHM.TX_Buff[0] = MHM_OP_WRITE_REG_CONT;
	MHM.TX_Buff[1] = adr;
	memcpy(&MHM.TX_Buff[2], pdat, size);

	MHM_Start_Tranceive(size + 2);

	MHM_Wait_Tranceive();
}
/**
 * @brief reset multi-turn
 *
 */
void MHM_Rest_MT()
{
	/* Pulse ic-pv Preset pin */
	MHM_Write_Reg(MHM_ADDR_FIO, 0x08);
	Delay_500ns();
	MHM_Write_Reg(MHM_ADDR_FIO, 0x00);
	/* ic-pv needs 20ms wait for eeprom */
	HAL_Delay(20);
	/* Reset MHM */
	MHM_Write_Reg(MHM_ADDR_RESET, MHM_INSTR_RESET);
	HAL_Delay(2);

}
/**
 * @brief start SPI DMA tranceive
 *
 * @param DataLength size
 */
static inline void MHM_Start_Tranceive(uint32_t DataLength)
{
	SPI_MHM_EN();	//CS	

	/*** Enable Rx DMA Request ***/
	SET_BIT(HSPI_MHM.Instance->CR2, SPI_CR2_RXDMAEN);

	/*** Enable RX DMA ***/
	/* Configure the source, destination address and the data length */
	DMA_SetConfig(&HDMA_MHM_RX, (uint32_t) &(HSPI_MHM.Instance->DR),
			(uint32_t) MHM.RX_Buff, DataLength);
	/* Enable the Peripheral */
	__HAL_DMA_ENABLE(&HDMA_MHM_RX);
	/*** Enable TX DMA ***/
	/* Configure the source, destination address and the data length */
	DMA_SetConfig(&HDMA_MHM_TX, (uint32_t) MHM.TX_Buff,
			(uint32_t) &(HSPI_MHM.Instance->DR), DataLength);
	/* Enable the Peripheral */
	__HAL_DMA_ENABLE(&HDMA_MHM_TX);

	/*** Enable Tx DMA Request ***/
	SET_BIT(HSPI_MHM.Instance->CR2, SPI_CR2_TXDMAEN);
	/*** Enable SPI ***/
	__HAL_SPI_ENABLE(&HSPI_MHM);

}
/**
 * @brief wait SPI DMA transceive
 *
 */
static inline void MHM_Wait_Tranceive()
{
	DMA_Base_Registers *regs_TX =
			(DMA_Base_Registers*) HDMA_MHM_TX.StreamBaseAddress;
	DMA_Base_Registers *regs_RX =
			(DMA_Base_Registers*) HDMA_MHM_RX.StreamBaseAddress;
	/* Wait then Clear the transfer complete flag */
	while ((regs_TX->ISR & (DMA_FLAG_TCIF0_4 << HDMA_MHM_TX.StreamIndex))
			== RESET)
		;
	regs_TX->IFCR = DMA_FLAG_TCIF0_4 << HDMA_MHM_TX.StreamIndex;
	while ((regs_RX->ISR & (DMA_FLAG_TCIF0_4 << HDMA_MHM_RX.StreamIndex))
			== RESET)
		;
	regs_RX->IFCR = DMA_FLAG_TCIF0_4 << HDMA_MHM_RX.StreamIndex;

	/* Disable the Peripheral */
	__HAL_DMA_DISABLE(&HDMA_MHM_TX);
	__HAL_DMA_DISABLE(&HDMA_MHM_RX);

	/* Disable SPI */
	__HAL_SPI_DISABLE(&HSPI_MHM);

	/* Disable Rx/Tx DMA Request (done by default to handle the case master rx direction 2 lines) */
	CLEAR_BIT(HSPI_MHM.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

	SPI_MHM_DIS();	//CS
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








