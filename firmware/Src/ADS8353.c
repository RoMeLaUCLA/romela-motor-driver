/**
 * @file 	ADS8353.c
 * @brief 	Functions for ADS8353 ADC.
 *
 * @author 	Tym Zhu
 */
#include "ADS8353.h"

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


//uint32_t i_8353=0;
ADS8353_TypeDef ADS8353;	/**< ADS8353 device */

static inline void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress,
		uint32_t DstAddress, uint32_t DataLength);
static inline void ADS8353_Start_Tranceive();
static inline void ADS8353_Wait_Tranceive();

/**
 * @brief Write and read confirm register
 *
 * @param wReg	Register to write
 * @param rReg	Register to read
 * @param dat	data to be written
 * @return
 */
HAL_StatusTypeDef ADS8353_Write_Confirm_Reg(uint16_t wReg,
		uint16_t rReg, uint16_t dat)
{
	ADS8353_Write_Reg(wReg, dat);
	if (ADS8353_Read_Reg(rReg) == dat)
		return HAL_OK;
	else
		return HAL_ERROR;
}

/**
 * @brief Write config register
 *
 * @param reg	Register to write
 * @param dat	data to be written
 */
void ADS8353_Write_Reg(uint16_t reg, uint16_t dat)
{
	/* First frame */
	ADS8353.TX_Buff[0] = reg + dat;
	ADS8353_Start_Tranceive();
	ADS8353_Wait_Tranceive();

	return;
}

/**
 * @brief Read register data
 *
 * @param reg Register to write
 * @return    data
 */
uint16_t ADS8353_Read_Reg(uint16_t reg)
{
	/* First frame */
	ADS8353.TX_Buff[0] = reg;
	ADS8353_Start_Tranceive();
	ADS8353_Wait_Tranceive();

	/* Second frame */
	ADS8353.TX_Buff[0] = 0x00;
	ADS8353_Start_Tranceive();
	ADS8353_Wait_Tranceive();

	return ADS8353.RX_Buff[0];
}

/**
 * @brief Start tranceive for data
 *
 */
void ADS8353_Start_GetData()
{
	ADS8353.TX_Buff[0] = 0x0000;
	ADS8353_Start_Tranceive();
	return;
}

/**
 * @brief Blocking wait for data
 *
 * @param ADC_A	pointer to data for channel A
 * @param ADC_B pointer to data for channel B
 */
void ADS8353_Wait_GetData(uint16_t *ADC_A, uint16_t *ADC_B)
{
	ADS8353_Wait_Tranceive();
	*ADC_A = ADS8353.RX_Buff[1];
	*ADC_B = ADS8353.RX_Buff[2];
	return;
}

/**
 * @brief Start SPI DMA tranceive
 *
 */
static inline void ADS8353_Start_Tranceive()
{
	SPI_ADS8353_EN();	//CS
	/*** Enable Rx DMA Request ***/
	SET_BIT(HSPI_ADS8353.Instance->CR2, SPI_CR2_RXDMAEN);
	/* Enable the Peripheral */
	__HAL_DMA_ENABLE(&HDMA_ADS8353_RX);
	/* Enable the Peripheral */
	__HAL_DMA_ENABLE(&HDMA_ADS8353_TX);
	/*** Enable Tx DMA Request ***/
	SET_BIT(HSPI_ADS8353.Instance->CR2, SPI_CR2_TXDMAEN);

	/*** Enable SPI ***/
	__HAL_SPI_ENABLE(&HSPI_ADS8353);
}

/**
 * @brief Wait for SPI DMA tranceive
 *
 */
static inline void ADS8353_Wait_Tranceive()
{
	DMA_Base_Registers *regs_TX =
			(DMA_Base_Registers*) HDMA_ADS8353_TX.StreamBaseAddress;
	DMA_Base_Registers *regs_RX =
			(DMA_Base_Registers*) HDMA_ADS8353_RX.StreamBaseAddress;
	/* Wait then Clear the transfer complete flag */
	while ((regs_TX->ISR & (DMA_FLAG_TCIF0_4 << HDMA_ADS8353_TX.StreamIndex))
			== RESET)
		;
	regs_TX->IFCR = DMA_FLAG_TCIF0_4 << HDMA_ADS8353_TX.StreamIndex;
	while ((regs_RX->ISR & (DMA_FLAG_TCIF0_4 << HDMA_ADS8353_RX.StreamIndex))
			== RESET)
		;
	regs_RX->IFCR = DMA_FLAG_TCIF0_4 << HDMA_ADS8353_RX.StreamIndex;

	/* Disable the Peripheral */
	__HAL_DMA_DISABLE(&HDMA_ADS8353_TX);
	__HAL_DMA_DISABLE(&HDMA_ADS8353_RX);

	/* Disable SPI */
	__HAL_SPI_DISABLE(&HSPI_ADS8353);

	/* Disable Rx/Tx DMA Request (done by default to handle the case master rx direction 2 lines) */
	CLEAR_BIT(HSPI_ADS8353.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

	SPI_ADS8353_DIS();	//CS
}

/**
 * @brief Initialize DMA before start tranceive can be called
 *
 */
void ADS8353_DMA_Init()
{
	/*** Enable RX DMA ***/
	/* Configure the source, destination address and the data length */
	DMA_SetConfig(&HDMA_ADS8353_RX, (uint32_t) &(HSPI_ADS8353.Instance->DR),
			(uint32_t) ADS8353.RX_Buff, ADS8353_DataLength);
//    /* Enable the Peripheral */
//    __HAL_DMA_ENABLE(&HDMA_ADS8353_RX);
//	/*** Enable Rx DMA Request ***/
//	SET_BIT(HSPI_ADS8353.Instance->CR2, SPI_CR2_RXDMAEN);
	/*** Enable TX DMA ***/
	/* Configure the source, destination address and the data length */
	DMA_SetConfig(&HDMA_ADS8353_TX, (uint32_t) ADS8353.TX_Buff,
			(uint32_t) &(HSPI_ADS8353.Instance->DR), ADS8353_DataLength);
//    /* Enable the Peripheral */
//    __HAL_DMA_ENABLE(&HDMA_ADS8353_TX);
//	/*** Enable Tx DMA Request ***/
//  	SET_BIT(HSPI_ADS8353.Instance->CR2, SPI_CR2_TXDMAEN);
//	/*** Enable SPI ***/
//	__HAL_SPI_ENABLE(&HSPI_ADS8353);
}

/**
 * @brief  Sets the DMA Transfer parameter. Copied from HAL driver.
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

