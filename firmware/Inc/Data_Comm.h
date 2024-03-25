/**
 * @file 	Data_Comm.h
 * @brief 	Header for Data_Comm.c
 *
 * @author 	Tym Zhu
 */
#ifndef DATA_COMM_H
#define DATA_COMM_H

//#include "main.h"
#include "Config.h"


extern void UART_Debug_Routine(void);

extern void UART_RS485_Direct_Routine(void);
extern void UART_RS485_Bulk_Routine(void);
extern void UART_RS485_Bulk_Response(void);

void Update_Motor_Stat(void);
extern void Update_Motor_Config(void);

/** minimum for configurations */
static const Motor_Config_TypeDef Config_Min  = 
{
	0x00,		//	uint32_t	ID;
	0x00,		//	uint32_t	Mode;
	2748,		//	uint32_t	Baudrate;
	THETA_MIN_F,//	float32_t	Homing_Offset;

	0.0f,		//	float32_t	P_Id;
	0.0f,		//	float32_t	I_Id;
	0.0f,		//	float32_t	D_Id;
	
	0.0f,		//	float32_t	P_Iq;
	0.0f,		//	float32_t	I_Iq;
	0.0f,		//	float32_t	D_Iq;
	
	0.0f,		//	float32_t	P_Velo;
	0.0f,		//	float32_t	I_Velo;
	0.0f,		//	float32_t	D_Velo;
	
	0.0f,		//	float32_t	P_Posi;
	0.0f,		//	float32_t	I_Posi;
	0.0f,		//	float32_t	D_Posi;
	
	0.0f,		//	float32_t	P_DirectF;
	0.0f,		//	float32_t	I_DirectF;
	0.0f,		//	float32_t	D_DirectF;
	//	
	0.0f,		//	float32_t	Acc_Max;
	0.0f,		//	float32_t	I_Max;
	0.0f,		//	float32_t	Velo_Max;
	THETA_MIN_F,//	float32_t	Posi_Min;
	THETA_MIN_F,//	float32_t	Posi_Max;
	MIN_VBUS,	//	float32_t	Vbus_Min;
	MIN_VBUS,	//	float32_t	Vbus_Max;
	0,			//	uint32_t	Watchdog_Timeout;
	0.0f,		//	float32_t	Temp_Min;
	0.0f,		//	float32_t	Temp_Max;
	0			//	int32_t		Rtr_MT_Offset;
};
/** maximum for configurations */
static const Motor_Config_TypeDef Config_Max = 
{
	0xFD,		//	uint32_t	ID;
	0x03,		//	uint32_t	Mode;
	11250000,	//	uint32_t	Baudrate;
	THETA_MAX_F,//	float32_t	Homing_Offset;
	//	
	10.0f,		//	float32_t	P_Id;
	10.0f,		//	float32_t	I_Id;
	10.0f,		//	float32_t	D_Id;
	
	10.0f,		//	float32_t	P_Iq;
	10.0f,		//	float32_t	I_Iq;
	10.0f,		//	float32_t	D_Iq;
	
	1000.0f,	//	float32_t	P_Velo;
	1000.0f,	//	float32_t	I_Velo;
	1000.0f,	//	float32_t	D_Velo;
	
	10000.0f,	//	float32_t	P_Posi;
	10000.0f,	//	float32_t	I_Posi;
	10000.0f,	//	float32_t	D_Posi;
	
	10000.0f,	//	float32_t	P_DirectF;
	10000.0f,	//	float32_t	I_DirectF;
	10000.0f,	//	float32_t	D_DirectF;
	//	
	10000.0f,	//	float32_t	Acc_Max;
	CONFIG_IQ_MAX,	//	float32_t	I_Max;
	10000.0f,	//	float32_t	Velo_Max;
	THETA_MAX_F,//	float32_t	Posi_Min;
	THETA_MAX_F,//	float32_t	Posi_Max;
	MAX_VBUS,	//	float32_t	Vbus_Min;
	MAX_VBUS,	//	float32_t	Vbus_Max;
	10000000,	//	uint32_t	Watchdog_Timeout;
	CONFIG_TEMPERATURE_MAX,		//	float32_t	Temp_Min;
	CONFIG_TEMPERATURE_MAX,		//	float32_t	Temp_Max;
	0			//	int32_t		Rtr_MT_Offset;
};
/** defaults for configurations */
static const Motor_Config_TypeDef Config_Default = 
{
	0x01,		//	uint32_t	ID;
	0x00,		//	uint32_t	Mode;
	8000000,	//	uint32_t	Baudrate;
	0.0f,		//	float32_t	Homing_Offset;
	//	
	DEFAULT_KP,	//	float32_t	P_Id;
	DEFAULT_KI,	//	float32_t	I_Id;
	0.0f,		//	float32_t	D_Id;
	
	DEFAULT_KP,	//	float32_t	P_Iq;
	DEFAULT_KI,	//	float32_t	I_Iq;
	0.0f,		//	float32_t	D_Iq;
	
	1.0f,		//	float32_t	P_Velo;
	0.01f,		//	float32_t	I_Velo;
	0.0f,		//	float32_t	D_Velo;
	
	10.0f,		//	float32_t	P_Posi;
	0.0f,		//	float32_t	I_Posi;
	0.0f,		//	float32_t	D_Posi;
	
	10.0f,		//	float32_t	P_DirectF;
	0.0f,		//	float32_t	I_DirectF;
	1.0f,		//	float32_t	D_DirectF;
	//	
	5.0f,		//	float32_t	Acc_Max;
	5.0f,		//	float32_t	I_Max;
	1000.0f,	//	float32_t	Velo_Max;
	THETA_MIN_F,//	float32_t	Posi_Min;
	THETA_MAX_F,//	float32_t	Posi_Max;
	MIN_VBUS,	//	float32_t	Vbus_Min;
	MAX_VBUS,	//	float32_t	Vbus_Max;
	0,			//	uint32_t	Watchdog_Timeout;
	80.0f,		//	float32_t	Temp_Min;
	100.0f,		//	float32_t	Temp_Max;
	0			//	int32_t		Rtr_MT_Offset;
};


/** defaults for status */
static const Motor_Stat_TypeDef Stat_Default = 
{
	0x00,		//	uint32_t	Enable;
	0x00,		//	uint32_t	Homing_Done;
	//	
	0.0f,		//	float32_t	Goal_Id;
	0.0f,		//	float32_t	Goal_Iq;
	0.0f,		//	float32_t	Goal_Velo;
	0.0f,		//	float32_t	Goal_Posi;
	0.0f,		//	float32_t	Present_Id;
	0.0f,		//	float32_t	Present_Iq;
	0.0f,		//	float32_t	Present_Velo;
	0.0f,		//	float32_t	Present_Posi;
	//	
	0.0f,		//	float32_t	Vbus;
	0.0f,		//	float32_t	Temp_Winding;
	0.0f,		//	float32_t	Temp_MOSFET;
	0.0f,		//	float32_t	Temp_IC;
	//	
	0x00,		//	uint32_t 	Error;
	0x00		//	uint32_t 	Iq_Temps;
};


extern Packet_TypeDef Debug_Packet;
extern Packet_TypeDef RS485_Packet;

//extern uint8_t    RS485_TX_Buffer[255];
//extern uint16_t  *RS485_TX_Header;
//extern uint8_t   *RS485_TX_ID;
//extern uint8_t   *RS485_TX_Length;
//extern uint8_t   *RS485_TX_Error;
//extern uint8_t   *RS485_TX_Data_u8;
//extern float32_t *RS485_TX_Data_f32;
//extern uint32_t  *RS485_TX_Data_u32;

#endif
