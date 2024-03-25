/**
 * @file 	IC_MHM_PV.h
 * @brief 	Header for IC_MHM_PV.c
 *
 * @author 	Tym Zhu
 */
#ifndef IC_MHM_PV_H
#define IC_MHM_PV_H

#include "main.h"
//#include "Config.h"

/**
 * @name MHM defines
 * @{
 */
#define MHM_MT_SIZE			(4U)
#define MHM_ST_SIZE			(2U)

#define MHM_OP_ACTIVATE			0xB0
#define MHM_OP_READ_POS			0xA6
#define MHM_OP_READ_REG_CONT	0x8A
#define MHM_OP_WRITE_REG_CONT	0xCF
#define MHM_OP_READ_STAT		0x9C
#define MHM_OP_WRITE_INSTR		0xD9
#define MHM_OP_READ_REG_SGL		0x97
#define MHM_OP_WRITE_REG_SGL	0xD2
#define MHM_OP_READ_REG_STAT	0xAD

#define MHM_ADDR_ERROR		0x70
#define MHM_ADDR_PORTS		0x71
#define MHM_ADDR_GAINS		0x72
#define MHM_ADDR_CHIPREL	0x73
#define MHM_ADDR_RESET		0x74
#define MHM_ADDR_FIO		0x75
#define MHM_ADDR_SETGAIN	0x76

#define MHM_POS_NERR		0x80
#define MHM_POS_NWARN		0x40

#define MHM_INSTR_RESET		0x01
#define MHM_INSTR_PRESET	0x02

#define MHM_ERR_CFG			0x01	/**< Configuration Data CRC Error    */
#define MHM_ERR_OFFS		0x02	/**< Position Offset CRC Error       */
#define MHM_ERR_POS			0x04	/**< Absolute Position Not Available */
#define MHM_ERR_EXT			0x08	/**< External Error                  */
#define MHM_ERR_AMIN		0x10	/**< Minimum Amplitude Error         */
#define MHM_ERR_AMAX		0x20	/**< Maximum Amplitude Error         */
#define MHM_ERR_MTI			0x40	/**< Multiturn Interface Error       */
#define MHM_ERR_MT			0x80	/**< Multiturn Position Error        */

#define MHM_BOOTUP_TIME		(45) 						/*< 45ms or 45 times */
#define MHM_PV_DELAY		((uint32_t)(HIGH_LOOP_HZ/50))	/*< 20ms */
#define MHM_RESET_DELAY		((uint32_t)(HIGH_LOOP_HZ/40))	/*< 25ms */

#define MHM_RESET()			{MHM.State = 1;}			/*< reset MHM */

/**
 * @}
 */

/** IC_MHM_TypeDef */
typedef struct
{
	volatile uint32_t State;
	volatile uint8_t TX_Buff[24];
	volatile uint8_t RX_Buff[24];
} IC_MHM_TypeDef;

extern IC_MHM_TypeDef MHM;

extern void MHM_Start_ReadPosi();
extern uint8_t MHM_Wait_Read_Posi(int32_t *MT, int32_t *ST);
extern uint8_t MHM_Read_Posi(int32_t *MT, int32_t *ST);

extern uint8_t MHM_Read_Reg(uint8_t adr);
extern void MHM_Read_Regs(uint8_t adr, uint8_t size, uint8_t* pdata);

extern void MHM_Start_Write_Reg(uint8_t adr, uint8_t dat);
extern void MHM_Wait_Write_Reg();
extern void MHM_Write_Reg(uint8_t adr, uint8_t dat);
extern void MHM_Write_Regs(uint8_t adr, uint8_t *pdat, uint8_t size);

extern void MHM_Rest_MT();

//					  {	0x07, 0x07, 0x06, 0x28, 0x80, 0x00, 0x00, 0x00, 0x0B, 0x80, 
//						0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8C};

#endif
