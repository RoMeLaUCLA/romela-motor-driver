/**
 * @file 	ADS8353.h
 * @brief 	Header file for ADS8353.c
 *
 * @author 	Tym Zhu
 */
#ifndef ADS8353_H
#define ADS8353_H

#include "main.h"
//#include "Config.h"


#define ADS8353_DataLength	0x03	/**< ADS8353 Data Length */

/**
 * @name ADS8353 CFR bits defines
 * @{
 */
#define ADS_RD_CLK_MODE_Pos		(11U)
#define ADS_RD_DATA_LINES_Pos	(10U)
#define ADS_INPUT_RANGE_Pos		(9U)
#define ADS_INM_SEL_Pos			(7U)
#define ADS_REF_SEL_Pos			(6U)
#define ADS_STANDBY_Pos			(5U)
#define ADS_RD_DATA_FORMAT_Pos	(4U)
/**
 * @}
 */

/**
 * @name ADS8353 operation defines
 * @{
 */
#define ADS_READ_REFDAC_A	0x1000	/**< REFDAC_A read */
#define ADS_READ_REFDAC_B	0x2000	/**< REFDAC_B read */
#define ADS_READ_CFR		0x3000	/**< CFR read */
#define ADS_WRITE_REFDAC_A	0x9000	/**< REFDAC_A write */
#define ADS_WRITE_REFDAC_B	0xA000	/**< REFDAC_B write */
#define ADS_WRITE_CFR		0x8000	/**< CFR write */
/**
 * @}
 */

/**
 * @brief ADS8353 device Type Define
 *
 */
typedef struct
{
	volatile uint16_t TX_Buff[4];
	volatile uint16_t RX_Buff[4];
} ADS8353_TypeDef;


extern ADS8353_TypeDef ADS8353;

extern HAL_StatusTypeDef ADS8353_Write_Confirm_Reg(uint16_t wReg,
		uint16_t rReg, uint16_t dat);
extern void ADS8353_Write_Reg(uint16_t reg, uint16_t dat);
extern uint16_t ADS8353_Read_Reg(uint16_t reg);

extern void ADS8353_DMA_Init();

extern void ADS8353_Start_GetData();
extern void ADS8353_Wait_GetData(uint16_t *ADC_A, uint16_t *ADC_B);

#endif
