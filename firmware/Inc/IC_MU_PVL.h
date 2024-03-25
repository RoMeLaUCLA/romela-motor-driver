/**
 * @file 	IC_MU_PVL.h
 * @brief	Header for IC_MU_PVL.c
 *
 * @author 	Tym Zhu
 */
#ifndef IC_MU_PVL_H
#define IC_MU_PVL_H

#include "main.h"
//#include "Config.h"

/**
 * @name MHM defines
 * @{
 */
#define MU_SDAD_SIZE			(4U)
#define MU_ST_BIT_SIZE			(19U)
#define MU_TXRX_BUFFER_SIZE		(8U)

#define MU_OP_ACTIVATE			0xB0
#define MU_OP_SDAD_TRANSMIT		0xA6
#define MU_OP_SDAD_STATUS		0xF5
#define MU_OP_READ_REG_SGL		0x97
#define MU_OP_WRITE_REG_SGL		0xD2
#define MU_OP_READ_REG_STAT		0xAD


#define MU_ADDR_ACGAIN_M		0x2B
#define MU_ADDR_ACGAIN_N		0x2F
#define MU_ADDR_STATUS0			0x76
#define MU_ADDR_STATUS1			0x77


//#define MHM_INSTR_RESET		0x01
//#define MHM_INSTR_PRESET	0x02

#define MU_ERR_AM_MIN		0x01	// poor level (master track)
#define MU_ERR_AM_MAX		0x02	// clipping (master track)
#define MU_ERR_AN_MIN		0x04	// poor level (nonius track)
#define MU_ERR_AN_MAX		0x08	// clipping (nonius track)
#define MU_ERR_STUP			0x10	// Startup iC-MU

#define MU_ERR_CMD_EXE		0x01	// Command execution in progress
#define MU_ERR_FRQ_CNV		0x02	// Excessive signal frequency for internal 12 Bit converter
#define MU_ERR_FRQ_ABZ		0x04	// Excessive signal frequency for ABZ-converter
#define MU_ERR_NON_CTR		0x08	// Period counter consistency error
#define MU_ERR_MT_CTR		0x10	// Multiturn data consistency error
#define MU_ERR_MT_ERR		0x20	// Multiturn communication error
#define MU_ERR_EPR_ERR		0x40	// I2C communication error
#define MU_ERR_CRC_ERR		0x80	// Invalid check sum internal RAM

#define MU_SPI_VALID		0x01	// DATA is valid
#define MU_SPI_BUSY			0x02	// Slave is busy with a request
#define MU_SPI_FAIL			0x04	// Data request has failed
#define MU_SPI_DISMISS		0x08	// Address rejected
#define MU_SPI_ERROR		0x80	// Opcode not implemented

/**
 * @}
 */

/** IC_MU_TypeDef */
typedef struct {
	uint32_t		 State;
    uint8_t			 TX_Buff[MU_TXRX_BUFFER_SIZE];
    volatile uint8_t RX_Buff[MU_TXRX_BUFFER_SIZE];
} IC_MU_TypeDef;

extern IC_MU_TypeDef MU;


extern void		MU_Start_Read_Posi();
extern void		MU_Wait_Read_Posi(int32_t *MT, int32_t *ST);
extern void		MU_Read_Posi(int32_t *MT, int32_t *ST);

extern uint8_t MU_Read_Reg(uint8_t adr);
extern void    MU_Write_Reg(uint8_t adr, uint8_t dat);

//extern void MU_Rest_MT();



#endif
