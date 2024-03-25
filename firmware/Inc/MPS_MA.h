/**
 * @file 	MPS_MA.h
 * @brief 	Header for MPS_MA.c
 *
 * @author 	Tym Zhu
 */
#ifndef MPS_MA_H
#define MPS_MA_H

#include "main.h"
//#include "Config.h"


/**
 * @name Flash address defines
 * @{
 */
#define MA_CMD_READ		0x4000
#define MA_CMD_WRITE	0x8000

#define MA_ADDR_BCT		0x02
#define MA_ADDR_ET		0x03
#define MA_ADDR_RD		0x09
/**
 * @}
 */

extern uint16_t MA_Read_Posi_HAL();
extern uint16_t MA_Read_Posi();

extern uint8_t MA_Read_Reg(uint8_t addr);

extern HAL_StatusTypeDef MA_Write_Confirm_Reg(uint8_t addr, uint8_t dat);



#endif
