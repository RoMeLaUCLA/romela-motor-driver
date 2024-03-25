/**
 * @file 	DRV8320.h
 * @brief 	Header for DRV8320.c
 *
 * @author 	Tym Zhu
 */
#ifndef DRV8320_H
#define DRV8320_H

#include "main.h"
//#include "Config.h"

/**
 * @name DRV8320 defines
 * @{
 */

#define DRV_SPI_WRITE_PRESCALER SPI_BAUDRATEPRESCALER_4
#define DRV_SPI_READ_PRESCALER  SPI_BAUDRATEPRESCALER_256

#define DRV_ADDR_FAULT1	0x00
#define DRV_ADDR_FAULT2	0x01
#define DRV_ADDR_DCTRL	0x02
#define DRV_ADDR_GDHS	0x03
#define DRV_ADDR_GDLS	0x04
#define DRV_ADDR_OCP	0x05

/* 0x00 Fault status 1*/
#define DRV_FAULT			(1U<<10U)
#define DRV_FAULT_VDS_OCP	(1U<<9U)
#define DRV_FAULT_GDF		(1U<<8U)
#define DRV_FAULT_UVLO		(1U<<7U)
#define DRV_FAULT_OTSD		(1U<<6U)
#define DRV_FAULT_VDS_HA	(1U<<5U)
#define DRV_FAULT_VDS_LA	(1U<<4U)
#define DRV_FAULT_VDS_HB	(1U<<3U)
#define DRV_FAULT_VDS_LB	(1U<<2U)
#define DRV_FAULT_VDS_HC	(1U<<1U)
#define DRV_FAULT_VDS_LC	(1U)
/* 0x01 Fault status 2*/
#define DRV_FAULT_SA_OC		(1U<<10U)
#define DRV_FAULT_SB_OC		(1U<<9U)
#define DRV_FAULT_SC_OC		(1U<<8U)
#define DRV_FAULT_OTW		(1U<<7U)
#define DRV_FAULT_CPUV		(1U<<6U)
#define DRV_FAULT_VGS_HA	(1U<<5U)
#define DRV_FAULT_VGS_LA	(1U<<4U)
#define DRV_FAULT_VGS_HB	(1U<<3U)
#define DRV_FAULT_VGS_LB	(1U<<2U)
#define DRV_FAULT_VGS_HC	(1U<<1U)
#define DRV_FAULT_VGS_LC	(1U)
/* 0x02 Drive Control */
#define DRV_DC_DIS_CPUV_Pos		(9U)
#define DRV_DC_DIS_GDF_Pos		(8U)
#define DRV_DC_OTW_REP_Pos		(7U)
#define DRV_DC_PWM_MODE_Pos		(5U)
#define DRV_DC_1PWM_COM_Pos		(4U)
#define DRV_DC_1PWM_DIR_Pos		(3U)
#define DRV_DC_COAST_Pos		(2U)
#define DRV_DC_BRAKE_Pos		(1U)
#define DRV_DC_CLR_FLT_Pos		(0U)
/* 0x03 Gate Drive HS */
#define DRV_GDHS_LOCK_Pos		(8U)
#define DRV_GDHS_IDRIVEP_HS_Pos	(4U)
#define DRV_GDHS_IDRIVEN_HS_Pos	(0U)
/* 0x04 Gate Drive LS */
#define DRV_GDLS_CBC_Pos		(10U)
#define DRV_GDLS_TDRIVE_Pos		(8U)
#define DRV_GDLS_IDRIVEP_LS_Pos	(4U)
#define DRV_GDLS_IDRIVEN_LS_Pos	(0U)
/* 0x05 OCP Control */
#define DRV_OCP_TRETRY_Pos		(10U)
#define DRV_OCP_DEADTIME_Pos	(8U)
#define DRV_OCP_MODE_Pos		(6U)
#define DRV_OCP_DEG_Pos			(4U)
#define DRV_OCP_VDS_LVL_Pos		(0U)

/**
 * @}
 */
extern HAL_StatusTypeDef DRV8320_Write_Confirm(uint8_t addr, uint16_t dat);
extern void		DRV8320_Write(uint8_t addr, uint16_t dat);
extern uint16_t	DRV8320_Read(uint8_t addr);

extern void DRV8320_Reset();







#endif
