/**
 * @file 	Data_Comm.c
 * @brief 	Data communication
 *
 * @author 	Tym Zhu
 */
#include "Data_Comm.h"

#include "main.h"

#include "IC_MHM_PV.h"
//#include "IC_MU_PVL.h"
//#include "DRV8320.h"
//#include "TMP112.h"
//#include "MAX31855.h"
//#include "ADS8353.h"
//#include "AS5311.h"
//#include "MPS_MA.h"
//#include "Flash.h"
//
#include "Control_LL.h"
//#include "Control.h"
//#include "Data_Comm.h"
#include "Error.h"
#include "Multiturn_Enc.h"


Packet_TypeDef Debug_Packet;
Packet_TypeDef RS485_Packet;

uint8_t    RS485_TX_Buffer[255];
uint16_t  *RS485_TX_Header = (uint16_t*) RS485_TX_Buffer;
uint8_t   *RS485_TX_ID = (uint8_t*) (RS485_TX_Buffer + 2);
uint8_t   *RS485_TX_Length = (uint8_t*) (RS485_TX_Buffer + 3);
uint8_t   *RS485_TX_Error = (uint8_t*) (RS485_TX_Buffer + 4);
uint8_t   *RS485_TX_Data_u8 = (uint8_t*) (RS485_TX_Buffer + 5);
float32_t *RS485_TX_Data_f32 = (float32_t*) (RS485_TX_Buffer + 5);
uint32_t  *RS485_TX_Data_u32 = (uint32_t*) (RS485_TX_Buffer + 5);

/**
 * @brief Simple USART Put Char polling
 *
 * @param USARTx pointer to USART
 * @param ch	 character
 */
void USART_PutChar(USART_TypeDef *USARTx, uint8_t ch)
{
	while (!(USARTx->SR & USART_SR_TXE)){};
	USARTx->DR = ch;
}
/**
 * @brief Simple USART Put String
 *
 * @param USARTx pointer to USART
 * @param str	 string
 */
void USART_PutString(USART_TypeDef *USARTx, char *str)
{
	while (*str != 0)
	{
		USART_PutChar(USARTx, *str);
		str++;
	}
}

/**
 * @brief Checksum calculation
 *
 * @param p			pointer to data
 * @param Length	size
 * @return			checksum
 */
static inline uint8_t UART_CheckSum(uint8_t *p, uint8_t Length)
{
	uint8_t i;
	uint8_t CheckSum = 0;
	for (i = 0; i < Length; i++)
	{
		CheckSum += p[i];
	}
	return ~CheckSum;
}

/**
 * @brief Send data on RS485 bus using DMA
 *
 * @param size size of data
 */
static inline void RS485_DMA_TX(uint8_t size)
{
	/*DMA transmit*/
	RS485_TXEN();
	HAL_UART_Transmit_DMA(&HUART_RS485, RS485_TX_Buffer, size);
	Error_Reset(WARNING_COMM);
}

/**
 * @brief RS485 routine called in interrupt
 *
 */
void UART_RS485_Direct_Routine()
{
	uint8_t i;
	//uint8_t CheckSum=0;

	/*switch instruction*/
	switch (RS485_Packet.Buff.Buffer[0])
	{
	default:
		/*No matched case!*/
		Error_Set(WARNING_COMM);
		break;
	case 0x01: //PING
		*RS485_TX_Header = 0xffff;
		*RS485_TX_ID = Config_Active.ID;
		*RS485_TX_Length = 6;				//length
		*RS485_TX_Error = (Mode.Error | 0x80);		//error

		RS485_TX_Data_u32[0] = (HW_VERSION << 24) + (DRV_VERSION << 16)
				+ (FW_VERSION & 0xFFFF);

		RS485_TX_Data_u8[4] = UART_CheckSum(RS485_TX_ID, 7);	//CheckSum

		/*DMA transmit*/
		RS485_DMA_TX(10);
		break;

	case 0x02: //Read Stat
		*RS485_TX_Header = 0xffff;
		*RS485_TX_ID = Config_Active.ID;
		*RS485_TX_Length = 2 + 4 * (RS485_Packet.Packet_Size - 2);	//length
		*RS485_TX_Error = (Mode.Error | 0x80);		//error
		/*Fetch all the data*/
		for (i = 0; i < (RS485_Packet.Packet_Size - 2); i++)
		{
			RS485_TX_Data_u32[i] =
					((uint32_t*) (&Stat_Active))[RS485_Packet.Buff.Buffer[i + 1]];
		}
		RS485_TX_Data_u8[(*RS485_TX_Length - 2)] = UART_CheckSum(RS485_TX_ID,
				(*RS485_TX_Length + 1));	//CheckSum

		/*DMA Transmit*/
		RS485_DMA_TX(4 + *RS485_TX_Length);
		break;
	case 0x03: //Write Stat

		for (i = 0; i < (RS485_Packet.Packet_Size / 5); i++)
		{
			if (RS485_Packet.Buff.Buffer[1 + 5 * i] < MOTOR_STAT_SIZE)
			{
				/*Store in Buffer first*/
				((uint32_t*) (&Stat_Buff))[RS485_Packet.Buff.Buffer[1 + 5 * i]] =
						*((uint32_t*) (&(RS485_Packet.Buff.Buffer[2 + 5 * i])));
			}
			else
			{
				/*TODO: Error management*/
				Error_Set(WARNING_COMM);
			}
		}
		/*No return*/
		break;

	case 0x04: //Read Config
		*RS485_TX_Header = 0xffff;
		*RS485_TX_ID = Config_Active.ID;
		*RS485_TX_Length = 2 + 4 * (RS485_Packet.Packet_Size - 2);	//length
		*RS485_TX_Error = (Mode.Error | 0x80);		//error
		/*Fetch all the data*/
		for (i = 0; i < (RS485_Packet.Packet_Size - 2); i++)
		{
			RS485_TX_Data_u32[i] =
					((uint32_t*) (&Config_Active))[RS485_Packet.Buff.Buffer[i
							+ 1]];
		}

		RS485_TX_Data_u8[(*RS485_TX_Length - 2)] = UART_CheckSum(RS485_TX_ID,
				(*RS485_TX_Length + 1));	//CheckSum

		/*DMA Transmit*/
		RS485_DMA_TX(4 + *RS485_TX_Length);
		break;

	case 0x05: //Write Config
		for (i = 0; i < (RS485_Packet.Packet_Size / 5); i++)
		{
			if (RS485_Packet.Buff.Buffer[1 + 5 * i] < MOTOR_CONFIG_SIZE)
			{
				/*Store in Buffer first*/
				((uint32_t*) (&Config_Buff))[RS485_Packet.Buff.Buffer[1 + 5 * i]] =
						*((uint32_t*) (&(RS485_Packet.Buff.Buffer[2 + 5 * i])));
			}
			else
			{
				/*TODO: Error management*/
				Error_Set(WARNING_COMM);
			}
		}

		break;

	case 0x06: //Save to flash
		Save_Config();
		Trig_Slow_Loop();

		break;

	case 0x08: /* Set current absolute position */

		if ((*(float32_t*) (&RS485_Packet.Buff.Buffer[1]) >= THETA_MIN_F) && // requested position
				(*(float32_t*) (&RS485_Packet.Buff.Buffer[1]) < THETA_MAX_F))
		{
			MLT_MT.Exp_Theta = *(float32_t*) (&RS485_Packet.Buff.Buffer[1]);
			MLT_MT.Tolerance = *(float32_t*) (&RS485_Packet.Buff.Buffer[5]);
			if (Mode.Error & ERROR_ABS_POSI)
			{
#ifdef MHM_PV_MULTITURN
				/* Reset then calculate */
				MLT_MT.State = 1;	//primed
				MHM_RESET();
#else
				/* Calculate now */
				MLT_MT.State = 2;	//calculate
				Trig_Slow_Loop();
#endif

			}
			else
			{
				/* Calculate now */
				MLT_MT.State = 2;	//calculate
				Trig_Slow_Loop();
			}

		}

		break;

	}

	/*Clear Bulk Status*/
	RS485_Packet.Bulk_Wait = 0;
}

/**
 * @brief RS485 routine for bulk read write
 *
 */
void UART_RS485_Bulk_Routine()
{
	uint8_t i, j;
	uint8_t Num_Motor, Num_Read, Num_Write;
	/*switch instruction*/
	switch (RS485_Packet.Buff.Buffer[0])
	{
	default:
		/*No matched case!*/
		Error_Set(WARNING_COMM);
		break;

	case 0x12:
		/*Bulk Status Read/Write*/
		Num_Motor = RS485_Packet.Buff.Buffer[1];
		Num_Read = (RS485_Packet.Buff.Buffer[2]) >> 4;
		Num_Write = (RS485_Packet.Buff.Buffer[2]) & 0x0F;
		/*Check packet length*/
		if (RS485_Packet.Packet_Size
				== (4 + Num_Read + Num_Write + Num_Motor * (1 + 4 * Num_Write)))
		{
			/*Length Check Success!*/
			/*finding ID match*/
			for (i = 0; i < Num_Motor; i++)
			{
				if (RS485_Packet.Buff.Buffer[3 + Num_Read + Num_Write
						+ i * (1 + 4 * Num_Write)] == Config_Active.ID)
				{
					/*ID match!*/
					/*update the status registers*/
					for (j = 0; j < Num_Write; j++)
					{
						if (RS485_Packet.Buff.Buffer[3 + Num_Read + j]
								< MOTOR_STAT_SIZE)
						{
							/*Store in Buffer first*/
							((uint32_t*) (&Stat_Buff))[RS485_Packet.Buff.Buffer[3
									+ Num_Read + j]] =
									*((uint32_t*) (&(RS485_Packet.Buff.Buffer[4
											+ Num_Read + Num_Write
											+ i * (1 + 4 * Num_Write) + 4 * j])));
						}
						else
						{
							/*TODO: Error management*/
							Error_Set(WARNING_COMM);
						}
					}

					if (Num_Read != 0)
					{
						/*Prime the return packet*/
						*RS485_TX_Header = 0xffff;
						*RS485_TX_ID = Config_Active.ID;
						*RS485_TX_Length = 2 + 4 * Num_Read;			//length
						*RS485_TX_Error = (Mode.Error | 0x80);	//error
						/*Fetch all the data*/
						for (j = 0; j < Num_Read; j++)
						{
							RS485_TX_Data_u32[j] =
									((uint32_t*) (&Stat_Active))[RS485_Packet.Buff.Buffer[3
											+ j]];
						}
						RS485_TX_Data_u8[(*RS485_TX_Length - 2)] =
								UART_CheckSum(RS485_TX_ID,
										(*RS485_TX_Length + 1));	//CheckSum

						if (i == 0)
						{
							/*1st on the ID list*/
							/*DMA Transmit*/
							RS485_DMA_TX(4 + *RS485_TX_Length);
							/*Clear Bulk Status*/
							RS485_Packet.Bulk_Wait = 0;
						}
						else
						{
							/*wait on other motor IDs*/
							RS485_Packet.Bulk_Wait = 1;
							RS485_Packet.Bulk_Follow_ID =
									RS485_Packet.Buff.Buffer[3 + Num_Read
											+ Num_Write
											+ (i - 1) * (1 + 4 * Num_Write)];
						}
					}
					else
					{
						/*Write only*/
						/*Clear Bulk Status*/
						RS485_Packet.Bulk_Wait = 0;
					}

				}
			}

		}
		else
		{
			/*TODO: Wrong Packet Length!*/
			Error_Set(WARNING_COMM);
		}

		break;
	}
}

/**
 * @brief Send Bulk read response
 *
 */
void UART_RS485_Bulk_Response()
{
	/*DMA Transmit*/
	RS485_DMA_TX(4 + *RS485_TX_Length);
	/*Clear Bulk Status*/
	RS485_Packet.Bulk_Wait = 0;
}

/* Quick and risky way of updating stats */
#define STAT_CHANGE(DATA) (Stat_Active.DATA!=Stat_Buff.DATA)
//#define STAT_UPDATE_DEST(DATA,DEST,MIN,MAX) if (Stat_Active.DATA!=Stat_Buff.DATA) {\
//		if ( (Stat_Buff.DATA>=MIN) && (Stat_Buff.DATA<=MAX) ) \
//			DEST=Stat_Active.DATA=Stat_Buff.DATA; \
//		else Stat_Buff.DATA=Stat_Active.DATA;   }
#define STAT_UPDATE_DEST(DATA,DEST,MIN,MAX) if ( (Stat_Buff.DATA>=MIN) && (Stat_Buff.DATA<=MAX) ) \
			DEST=Stat_Buff.DATA; \
		
/**
 * @brief Updating(write only) the stats
 *
 */
void Update_Motor_Stat()
{
	/*Enable*/
	if (STAT_CHANGE(Enable))
	{
//			if (((Motor1.Rtr_Theta<Config_Active.Posi_Min)||(Motor1.Rtr_Theta>Config_Active.Posi_Max))==0)
//				Error_Reset(ERROR_JOINT_LIMIT);

		if (Stat_Buff.Enable == 0x00)
		{
			/*Disable*/
			Motor_Disable();
			Stat_Active.Enable = 0;
		}
		else if (Stat_Buff.Enable == 0x01)
		{
			/*Enable*/

			if ((Mode.Error & (ERROR_ALL)) == 0)
			{
				Motor_Enable();
				Stat_Active.Enable = 1;
			}
		}
		else if (Stat_Buff.Enable == 0x03)
		{
			/* external estop */
			Error_Set(ERROR_WATCHDOG);
			Stat_Active.Enable = 0x03;
		}
		Stat_Buff.Enable = Stat_Active.Enable;	//TODO: Error management
	}

	/*update Goals with limits*/
	STAT_UPDATE_DEST(Goal_Id, Motor1.I_d_cmd, -Config_Active.I_Max,
			Config_Active.I_Max);
	STAT_UPDATE_DEST(Goal_Iq, Motor1.I_q_cmd, -Config_Active.I_Max,
			Config_Active.I_Max);
	STAT_UPDATE_DEST(Goal_Velo, Motor1.ThetaDot_cmd, -Config_Active.Velo_Max,
			Config_Active.Velo_Max);
	STAT_UPDATE_DEST(Goal_Posi, Motor1.Theta_cmd, Config_Active.Posi_Min,
			Config_Active.Posi_Max);
}

/*macro define for updating config*/
/*Readonly registers */
/*TODO: Error Management*/
#define CONFIG_UPDATE_RO(DATA)  if (Config_Active.DATA != Config_Buff.DATA) \
									Config_Buff.DATA = Config_Active.DATA;

#define CONFIG_UPDATE_DEST(DATA, DEST)  if (Config_Active.DATA != Config_Buff.DATA) { \
										if ( (Config_Buff.DATA >= Config_Min.DATA) && (Config_Buff.DATA <= Config_Max.DATA) ) \
											 DEST = Config_Active.DATA = Config_Buff.DATA; \
										else Config_Buff.DATA = Config_Active.DATA; }

#define CONFIG_UPDATE(DATA) 	if (Config_Active.DATA != Config_Buff.DATA) { \
								if ( (Config_Buff.DATA >= Config_Min.DATA) && (Config_Buff.DATA <= Config_Max.DATA) ) \
									 Config_Active.DATA = Config_Buff.DATA; \
								else Config_Buff.DATA = Config_Active.DATA; }

/**
 * @brief Config updating called in 22kHz loop
 *
 */
void Update_Motor_Config()
{

	/* ID */
	CONFIG_UPDATE(ID);
	/* Mode */
	if (Config_Active.Mode != Config_Buff.Mode)
	{
		if ((Config_Buff.Mode >= Config_Min.Mode)
				&& (Config_Buff.Mode <= Config_Max.Mode))
		{
			/*Change control mode -> reset all PIDs*/
			Mode.Ctrl_Mode = Config_Active.Mode = Config_Buff.Mode;
			arm_pid_reset_f32_L(&(Motor1.PID_Iq));
			arm_pid_reset_f32_L(&(Motor1.PID_Id));
			arm_pid_reset_f32_L(&(Motor1.PID_Velo));
			arm_pid_reset_f32_L(&(Motor1.PID_Posi));
			arm_pid_reset_f32_L(&(Motor1.PID_DF));
		}
		else
			Config_Buff.Mode = Config_Active.Mode; //Error management
	}

	/* Baudrate */
	CONFIG_UPDATE(Baudrate);
	if (Config_Active.Baudrate != huart1.Init.BaudRate)
	{
		/*DeInitialize & ReInitialize*/
		HAL_UART_DeInit(&huart1);
		huart1.Init.BaudRate = Config_Active.Baudrate;
		HAL_UART_Init(&huart1);
	}

	/* Homing offset */
	if (Config_Active.Homing_Offset != Config_Buff.Homing_Offset)
	{
		if ((Config_Buff.Homing_Offset >= Config_Min.Homing_Offset)
				&& (Config_Buff.Homing_Offset <= Config_Max.Homing_Offset))
		{

			Config_Active.Homing_Offset = Config_Buff.Homing_Offset;
			Motor1.Rtr_Theta_Offset32 =
					(int32_t) ((pCal_Flash->Theta_Zero_Offset
							+ Config_Active.Homing_Offset
							+ Motor1.Ext_Theta_ZeroOffset) * RAD2RtrTheta);
		}
		else
			Config_Buff.Homing_Offset = Config_Active.Homing_Offset;
	}

	/* Miscellaneous */

	CONFIG_UPDATE_DEST(Acc_Max, Motor1.Max_ThetaDotDot);
	CONFIG_UPDATE_DEST(I_Max,
			Motor1.PID_DF.Limit=Motor1.PID_Velo.Limit=Motor1.Max_Iq);
	CONFIG_UPDATE_DEST(Velo_Max, Motor1.PID_Posi.Limit=Motor1.Max_ThetaDot);

	CONFIG_UPDATE(Posi_Min);
	CONFIG_UPDATE(Posi_Max);
	CONFIG_UPDATE(Vbus_Min);
	CONFIG_UPDATE(Vbus_Max);
	CONFIG_UPDATE(Watchdog_Timeout);

	/*TODO: To make sure min<max*/
	CONFIG_UPDATE(Temp_Min);
	CONFIG_UPDATE(Temp_Max);

	/* PID Loops */
	/*Id loop*/
	if ((Config_Active.P_Id != Config_Buff.P_Id)
			|| (Config_Active.I_Id != Config_Buff.I_Id)
			|| (Config_Active.D_Id != Config_Buff.D_Id))
	{
		Config_Active.P_Id = Config_Buff.P_Id;
		Config_Active.I_Id = Config_Buff.I_Id;
		Config_Active.D_Id = Config_Buff.D_Id;
		PID_Init(&(Motor1.PID_Id), Config_Active.P_Id, Config_Active.I_Id,
				Config_Active.D_Id, MAX_PWM_RATIO);
	}
	/*Iq loop*/
	if ((Config_Active.P_Iq != Config_Buff.P_Iq)
			|| (Config_Active.I_Iq != Config_Buff.I_Iq)
			|| (Config_Active.D_Iq != Config_Buff.D_Iq))
	{
		Config_Active.P_Iq = Config_Buff.P_Iq;
		Config_Active.I_Iq = Config_Buff.I_Iq;
		Config_Active.D_Iq = Config_Buff.D_Iq;
		PID_Init(&(Motor1.PID_Iq), Config_Active.P_Iq, Config_Active.I_Iq,
				Config_Active.D_Iq, MAX_PWM_RATIO);
	}
	/*Velocity loop*/
	if ((Config_Active.P_Velo != Config_Buff.P_Velo)
			|| (Config_Active.I_Velo != Config_Buff.I_Velo)
			|| (Config_Active.D_Velo != Config_Buff.D_Velo))
	{
		Config_Active.P_Velo = Config_Buff.P_Velo;
		Config_Active.I_Velo = Config_Buff.I_Velo;
		Config_Active.D_Velo = Config_Buff.D_Velo;
		PID_Init(&(Motor1.PID_Velo), Config_Active.P_Velo, Config_Active.I_Velo,
				Config_Active.D_Velo, Motor1.Max_Iq);
	}
	/*Position loop*/
	if ((Config_Active.P_Posi != Config_Buff.P_Posi)
			|| (Config_Active.I_Posi != Config_Buff.I_Posi)
			|| (Config_Active.D_Posi != Config_Buff.D_Posi))
	{
		Config_Active.P_Posi = Config_Buff.P_Posi;
		Config_Active.I_Posi = Config_Buff.I_Posi;
		Config_Active.D_Posi = Config_Buff.D_Posi;
		PID_Init(&(Motor1.PID_Posi), Config_Active.P_Posi, Config_Active.I_Posi,
				Config_Active.D_Posi, Motor1.Max_ThetaDot);
	}
	/*Direct Force loop*/
	if ((Config_Active.P_DirectF != Config_Buff.P_DirectF)
			|| (Config_Active.I_DirectF != Config_Buff.I_DirectF)
			|| (Config_Active.D_DirectF != Config_Buff.D_DirectF))
	{
		Config_Active.P_DirectF = Config_Buff.P_DirectF;
		Config_Active.I_DirectF = Config_Buff.I_DirectF;
		Config_Active.D_DirectF = Config_Buff.D_DirectF;
		PID_Init(&(Motor1.PID_DF), Config_Active.P_DirectF,
				Config_Active.I_DirectF, Config_Active.D_DirectF,
				Motor1.Max_Iq);
	}

}

/**
 * @brief Debug port routine
 *
 */
void UART_Debug_Routine(void)
{
	uint32_t i;
	switch (Debug_Packet.Buff.Buffer[0])
	{
	default:
		break;
	case 0x01: //PING

		break;

	case 0x0D: //HAL_DeInit
		HAL_DeInit();
		break;

	case 0x0E: //reset stm32
		NVIC_SystemReset();
		break;

	case 0x0F:
		Motor_Enable();
//		arm_pid_reset_f32_L(&(Motor1.PID_Iq));
//		arm_pid_reset_f32_L(&(Motor1.PID_Id));
//		arm_pid_reset_f32_L(&(Motor1.PID_Velo));
//		arm_pid_reset_f32_L(&(Motor1.PID_Posi));
//		arm_pid_reset_f32_L(&(Motor1.PID_DF));
//		ENGATE_GPIO_Port->BSRR=ENGATE_Pin;

		break;
	case 0x10:
		Motor_Disable();
		break;

	case 0x11:	//Write PID Iq Id
		PID_Init(&(Motor1.PID_Id),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[1]))),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[5]))),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[9]))),
				MAX_PWM_RATIO);
		PID_Init(&(Motor1.PID_Iq),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[1]))),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[5]))),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[9]))),
				MAX_PWM_RATIO);
		break;
	case 0x12:	//Write PID V
		PID_Init(&(Motor1.PID_Velo),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[1]))),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[5]))),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[9]))),
				Motor1.Max_Iq);
		break;
	case 0x13:	//Write PID P
		PID_Init(&(Motor1.PID_Posi),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[1]))),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[5]))),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[9]))),
				Motor1.Max_ThetaDot);
		break;
	case 0x14:	//Write PID F
		PID_Init(&(Motor1.PID_DF),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[1]))),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[5]))),
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[9]))),
				Motor1.Max_Iq);
		break;

	case 0x16:	//Write all Ref

		Stat_Buff.Goal_Id = Motor1.I_d_cmd =
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[1])));
		Stat_Buff.Goal_Iq = Motor1.I_q_cmd =
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[5])));
		Stat_Buff.Goal_Velo = Motor1.ThetaDot_cmd =
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[9])));
		Stat_Buff.Goal_Posi = Motor1.Theta_cmd =
				*((float32_t*) (&(Debug_Packet.Buff.Buffer[13])));

		break;
	case 0x17:  //Write all limits
		Motor1.Max_Iq = *((float32_t*) (&(Debug_Packet.Buff.Buffer[1])));
		Motor1.PID_Velo.Limit = Motor1.Max_Iq;
		Motor1.PID_DF.Limit = Motor1.Max_Iq;
		Motor1.Max_ThetaDot = *((float32_t*) (&(Debug_Packet.Buff.Buffer[5])));
		Motor1.PID_Posi.Limit = Motor1.Max_ThetaDot;
		Motor1.Max_Theta = *((float32_t*) (&(Debug_Packet.Buff.Buffer[9])));
		break;
	case 0x18:  //Mode
		if (*((uint8_t*) (&(Debug_Packet.Buff.Buffer[1]))) != Mode.Ctrl_Mode)
		{
			Mode.Ctrl_Mode = *((uint8_t*) (&(Debug_Packet.Buff.Buffer[1])));
			arm_pid_reset_f32_L(&(Motor1.PID_Iq));
			arm_pid_reset_f32_L(&(Motor1.PID_Id));
			arm_pid_reset_f32_L(&(Motor1.PID_Velo));
			arm_pid_reset_f32_L(&(Motor1.PID_Posi));
			arm_pid_reset_f32_L(&(Motor1.PID_DF));
		}

		break;
	case 0x19:  //Manual_Theta
		if (Debug_Packet.Buff.Buffer[1] == 1)
		{
			Mode.Manual_E_Theta = 1;
			Motor1.Theta_manual =
					*((int32_t*) (&(Debug_Packet.Buff.Buffer[2])));
		}
		else if (Debug_Packet.Buff.Buffer[1] == 0)
			Mode.Manual_E_Theta = 0;

		break;
	case 0x1A:  //Debug_output
		Mode.Debug_Log = Debug_Packet.Buff.Buffer[1];
		break;
	case 0x1B:  //Torque_Comp
		if (Debug_Packet.Buff.Buffer[1] == 1)
			Mode.Torque_Comp = 1;
		else if (Debug_Packet.Buff.Buffer[1] == 0)
			Mode.Torque_Comp = 0;
		break;
	case 0x1C:  //Calibration
		if (Debug_Packet.Buff.Buffer[1] == 1)
			Mode.Calibrating = 1;
		else if (Debug_Packet.Buff.Buffer[1] == 0)
			Mode.Calibrating = 0;
		else if (Debug_Packet.Buff.Buffer[1] == 2)
			Mode.Calibrating = 2;
		break;
	case 0x1D:  //Manual_Vd_Vq
		if (Debug_Packet.Buff.Buffer[1] == 1)
		{
			Mode.Manual_V = 1;
			Motor1.V_d_cmd = *((float32_t*) (&(Debug_Packet.Buff.Buffer[2])));
			Motor1.V_q_cmd = *((float32_t*) (&(Debug_Packet.Buff.Buffer[6])));
		}
		else if (Debug_Packet.Buff.Buffer[1] == 0)
			Mode.Manual_V = 0;

		break;

	case 0x20:
		if (Debug_Packet.Buff.Buffer[1] == 0)
		{
			HAL_FLASH_Lock();
		}
		else if (Debug_Packet.Buff.Buffer[1] == 1)
		{
			HAL_FLASH_Unlock();
		}
		else if (Debug_Packet.Buff.Buffer[1] == 2)
		{
			//erase
			FLASH_Erase_Sector(Debug_Packet.Buff.Buffer[2],
			FLASH_VOLTAGE_RANGE_3);
			FLASH_WaitForLastOperation(5000);	//5sec
		}

		USART_PutChar(USART_DEBUG, 0xFF);
		break;

	case 0x21:	//read flash

		//read flash
		HUART_DEBUG.gState = HAL_UART_STATE_READY;

		HAL_UART_Transmit_DMA(&HUART_DEBUG,
				(uint8_t*) (*((uint32_t*) (&(Debug_Packet.Buff.Buffer[2])))),
				Debug_Packet.Buff.Buffer[1]);

		break;

	case 0x22:	//write flash	

		for (i = 0; i < Debug_Packet.Buff.Buffer[1]; i++)
		{
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
					(*((uint32_t*) (&(Debug_Packet.Buff.Buffer[2]))) + i),
					Debug_Packet.Buff.Buffer[6 + i]);
		}
		USART_PutChar(USART_DEBUG, 0xFF);
		break;

	case 0x23:	// reset iC-PV
		MHM_RESET();
		break;

	case 0x24:	// attempt to find MT
		MLT_MT.Exp_Theta = *(float32_t*) (&Debug_Packet.Buff.Buffer[1]);
		MLT_MT.Tolerance = *(float32_t*) (&Debug_Packet.Buff.Buffer[5]);
		MLT_MT.State = 2;
		Trig_Slow_Loop();
		break;
	}
}

