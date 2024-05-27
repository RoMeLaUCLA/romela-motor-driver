/**
 * @file 	Control.c
 * @brief 	Controllers
 *
 * @author 	Tym Zhu
 */
#include "Control.h"

#include "IC_MHM_PV.h"
#include "IC_MU_PVL.h"
#include "DRV8320.h"
#include "TMP112.h"
#include "MAX31855.h"
#include "ADS8353.h"
#include "AS5311.h"
#include "MPS_MA.h"
#include "Flash.h"

#include "util.h"

#include "Control_LL.h"
#include "Data_Comm.h"
#include "Error.h"
#include "Multiturn_Enc.h"

volatile uint8_t MidCR_Refresh = 0;		/**< reset for mid speed control loop counter */
volatile uint32_t MidCR_Count = 0;		/**< counter for mid speed control loop */
volatile uint8_t ADCR_Refresh = 0;		/**< reset for high speed control loop counter */
volatile uint32_t ADCR_Count = 0;		/**< counter for high speed control loop */

uint16_t DRV8305_REG[6];				/**< DRV8305 register buffer */

/* UART6 debug port */
uint8_t Debug_Buff[65];					/**< buffer for Sending data to debug port */

/** turn debug buffer uint8_t* into float32_t*
  * First byte of debug buffer reserved for 0xff.
  */
volatile float32_t (*Debug_Data_f)[16] = (float32_t (*)[16]) (Debug_Buff + 1);

arm_biquad_casd_df1_inst_f32 IIR_Vbus;		/**< IIR instance for Vbus */
float32_t IIR_Vbus_State[4];				/**< IIR states for Vbus */
arm_biquad_casd_df1_inst_f32 IIR_Posi_ref;	/**< IIR instance for reference position */
float32_t IIR_Posi_ref_State[4];			/**< IIR states for reference position */
//arm_biquad_casd_df1_inst_f32 IIR_TempTC;
//float32_t IIR_TempTC_State[4];

float32_t JointLimitP = 200;		//P gain for joint limit
float32_t JointLimitOffset = 0.1;	//offset for joint limit

/* private function */
void Debug_Terminal();
static inline void ADC_Debug();
static inline void Ctrl_Debug();

/**
 * @brief Initialize controller
 *
 */
void Ctrl_Init()
{
	uint32_t Calculated_CRC;
	uint32_t MHM_CT = 0;

	/*** Check Flash for Calibration data & Motor Config ***/

	/* Calculate CRC for calibration data in flash */
	printf("Motor Cal CRC from flash: 0X%08lX\n", *(uint32_t*) CAL_CRC_ADDR);
	Calculated_CRC = CRC_Calc((uint32_t*) pCal_Flash, CAL_SIZE);

	/* Check calculated CRC against flash */
	if (Calculated_CRC != *(uint32_t*) CAL_CRC_ADDR)
	{
		/* Failed CRC test */
		printf("Motor Cal CRC FAILED!   : 0X%08lX\n", Calculated_CRC);
		printf("Calibration needed! \n");
		/* ERROR: calibration data invalid */
		Error_Set(ERROR_INIT);
	}
	else
	{
		/* Passed the CRC test*/
		printf("Motor Cal CRC SUCCESS! :  0X%08lX\n", Calculated_CRC);
	}

	/* Calculate CRC for motor config in flash */
	Calculated_CRC = CRC_Calc((uint32_t*) pConfig_Flash, MOTOR_CONFIG_SIZE);
	printf("Motor config CRC from flash: 0X%08lX\n",
			*(uint32_t*) CONFIG_CRC_ADD);
	/* Check calculated CRC against flash */
	if (Calculated_CRC != *(uint32_t*) CONFIG_CRC_ADD)
	{
		/* Failed CRC test */
		printf("Motor config CRC FAILED!  : 0X%08lX\n\n", Calculated_CRC);
		printf("Reset motor config to default..\n");
		/*reset flash if failed CRC test*/
		if (Write_Motor_Config((Motor_Config_TypeDef*) &Config_Default) != 0)
			printf("Reset motor config to default failed!\n");
		else
			printf("Reset motor config to default success!\n");
	}
	else
	{
		/* Passed the CRC test*/
		printf("Motor config CRC SUCCESS! :  0X%08lX\n", Calculated_CRC);
	}

	/* Load Config from flash */
	Config_Buff = Config_Active = *pConfig_Flash;

	/*** Setup inital values for motor structure ***/
	Motor1.Rtr_Elec_Offset = pCal_Flash->Elec_Offset;
	Motor1.Rtr_Theta_Offset32 = (int32_t) ((pCal_Flash->Theta_Zero_Offset
			+ Config_Active.Homing_Offset + 0) * RAD2RtrTheta);
	Motor1.Rtr_MT_Offset = Config_Active.Rtr_MT_Offset;
	Motor1.ADC_A_Zero = ADC_MID + pCal_Flash->ADC_A_Offset;
	Motor1.ADC_B_Zero = ADC_MID + pCal_Flash->ADC_B_Offset;

	Motor1.Max_Iq = Config_Active.I_Max;
	Motor1.Max_ThetaDot = Config_Active.Velo_Max;
	Motor1.Max_ThetaDotDot = Config_Active.Acc_Max;

	/*** Encoder finding absolute position ***/
#ifdef MHM_PV_MULTITURN
	/* MHM w/ ic-pv Multiturn enabled */
	/* Try reading MT and ST from MHM */
	while ((MHM_Read_Posi(&Motor1.Rtr_MT, &Motor1.Rtr_ST) & (MHM_POS_NERR)) == 0x00)
	{
		/* try max MHM_BOOTUP_TIME */
		if (MHM_CT == MHM_BOOTUP_TIME) break;
		HAL_Delay(1);
		MHM_CT++;
	}
	if (MHM_CT == MHM_BOOTUP_TIME)
	{
		/* MHM still has error after MHM_BOOTUP_TIME */
		/* Check MHM error register */
		if ( MHM_Read_Reg(MHM_ADDR_ERROR) & 
			(MHM_ERR_CFG| MHM_ERR_OFFS| MHM_ERR_POS|
			 MHM_ERR_AMIN| MHM_ERR_AMAX| MHM_ERR_MTI) )
		{
			Error_Set(ERROR_INIT);
		}
	}
	
#elif defined(EXT_MA_ENCODER) || defined(EXT_AS_ENCODER)

#ifdef INIT_USE_EXT_ENC
	
		MHM_Read_Posi(&Motor1.Rtr_MT, &Motor1.Rtr_ST);	//get rotor postion
		/* get absolute encoder position */
		#ifdef EXT_MA_ENCODER
			Motor1.Ext_Theta_Raw_Prev = Motor1.Ext_Theta_Raw = MA_Read_Posi();
		#else	//EXT_AS_ENCODER
			Motor1.Ext_Theta_Raw_Prev = Motor1.Ext_Theta_Raw = AS5311_Simple_Read_Fast();
		#endif
		/* use ext theta directly */
		Motor1.Ext_Theta = ExtTheta2RAD * Motor1.Ext_Theta_Raw - PI;	
		printf("Ext_Theta:%.4f \n",  Motor1.Ext_Theta);
		/* Set MT offset = 0 */
		Motor1.Rtr_MT_Offset = Config_Active.Rtr_MT_Offset 
							 = Config_Buff.Rtr_MT_Offset = 0;
		/* Rtr Theta with MT offset = 0 */
		Motor1.Rtr_Theta_64 = (int64_t)(Motor1.Rtr_MT + Motor1.Rtr_MT_Offset) * RTR_ENC_RES 
						  	+ Motor1.Rtr_ST;
		Motor1.Rtr_Theta = WrapRtrTheta64(Motor1.Rtr_Theta_64 + Motor1.Rtr_Theta_Offset32) 
							* RtrTheta2RAD;
		printf("Rtr_Theta:%.4f \n",  Motor1.Rtr_Theta);
		/* find difference */
		printf("Rtr_Theta_Offset32:%ld \n",  Motor1.Rtr_Theta_Offset32);
		
		Motor1.Ext_Theta_ZeroOffset = Motor1.Ext_Theta - Motor1.Rtr_Theta;
		printf("Ext_Theta_ZeroOffset:%.4f \n",  Motor1.Ext_Theta_ZeroOffset);
		/* update offset */
		Motor1.Rtr_Theta_Offset32 = (int32_t)((pCal_Flash->Theta_Zero_Offset 
										   + Config_Active.Homing_Offset + Motor1.Ext_Theta_ZeroOffset)*RAD2RtrTheta);
		printf("Rtr_Theta_Offset32:%ld \n",  Motor1.Rtr_Theta_Offset32);
		/* double check rtr theta */
		Motor1.Rtr_Theta = WrapRtrTheta64(Motor1.Rtr_Theta_64 + Motor1.Rtr_Theta_Offset32) 
							* RtrTheta2RAD;
		printf("Rtr_Theta:%.4f \n",  Motor1.Rtr_Theta);
//		printf("Use Ext Theta:%.4f Theta Offset:%d Rtr Theta:%.4f \n",  Motor1.Ext_Theta,
//			   															Motor1.Rtr_Theta_Offset32,
//																		Motor1.Rtr_Theta );
		
	#else
	/* External MA or AS encoder */
	printf("Finding absolute position... ");

	do
	{
		if (MHM_CT == MHM_BOOTUP_TIME)
			break;
		MHM_Read_Posi(&Motor1.Rtr_MT, &Motor1.Rtr_ST);	//get rotor postion
		/* get absolute encoder position */
#ifdef EXT_MA_ENCODER
		Motor1.Ext_Theta_Raw_Prev = Motor1.Ext_Theta_Raw = MA_Read_Posi();
#else	//EXT_AS_ENCODER
				Motor1.Ext_Theta_Raw_Prev = Motor1.Ext_Theta_Raw = AS5311_Simple_Read_Fast();
			#endif

		MHM_CT++;

	} while (MLT_Init_MT(Motor1.Rtr_ST, Motor1.Rtr_MT, Motor1.Ext_Theta_Raw)
			!= 0);

	if (MHM_CT == MHM_BOOTUP_TIME)
	{
		/* Failed to find absolute position */
		printf("FAILED\n");
		Error_Set(ERROR_ABS_POSI);	//can not do position control
	}
	else
	{
		/* Success in finding absolute position */
		printf("DONE\n");
		/* Set new MT offset */
		Motor1.Rtr_MT_Offset = Config_Active.Rtr_MT_Offset =
				Config_Buff.Rtr_MT_Offset = MLT_MT.MT_Offset;
	}
#endif
#elif defined(MU_PVL_MULTITURN)
	

#elif defined(NO_ABS_ENCODER)
	/* No valid absolute encoder. For testing only! */
		printf("No absolute position!\n");
		/* Set new MT offset */
		Motor1.Rtr_MT_Offset = Config_Active.Rtr_MT_Offset 
							 = Config_Buff.Rtr_MT_Offset = 0;	

#else
	#error "Encoder finding absolute position invalid!"
#endif

	Motor1.Rtr_ST_Prev = Motor1.Rtr_ST;
	Motor1.Rtr_MT_Prev = Motor1.Rtr_MT;
	Motor1.Rtr_Theta_64 = (int64_t) (Motor1.Rtr_MT + Motor1.Rtr_MT_Offset)
			* RTR_ENC_RES + Motor1.Rtr_ST;
	Motor1.Rtr_Theta = WrapRtrTheta64(
			Motor1.Rtr_Theta_64 + Motor1.Rtr_Theta_Offset32) * RtrTheta2RAD;

//	Motor1.Ext_Theta = (Motor1.Ext_NumofTurn*EXT_ENC_RES + Motor1.Ext_Theta_Raw)
//						- pCal_Flash->Enc_Ext_A
//							*arm_sin_f32( pCal_Flash->Enc_Ext_FQ
//										  * (Motor1.Ext_NumofTurn*EXT_ENC_RES + Motor1.Ext_Theta_Raw)
//										  + pCal_Flash->Enc_Ext_PH )
//						+ Motor1.Ext_Theta_ZeroOffset;

	Motor1.Theta_Ref = Motor1.Rtr_Theta;

	/*** Initialize all PID loops ***/
	/*Current Loop PID*/
	PID_Init(&(Motor1.PID_Id), Config_Active.P_Id, Config_Active.I_Id,
			Config_Active.D_Id, MAX_PWM_RATIO);
	PID_Init(&(Motor1.PID_Iq), Config_Active.P_Iq, Config_Active.I_Iq,
			Config_Active.D_Iq, MAX_PWM_RATIO);
	/*Velocity Loop*/
	PID_Init(&(Motor1.PID_Velo), Config_Active.P_Velo, Config_Active.I_Velo,
			Config_Active.D_Velo, Motor1.Max_Iq);
	/*Position Loop*/
	PID_Init(&(Motor1.PID_Posi), Config_Active.P_Posi, Config_Active.I_Posi,
			Config_Active.D_Posi, Motor1.Max_ThetaDot);
	/*Position Loop direct force*/
	PID_Init(&(Motor1.PID_DF), Config_Active.P_DirectF, Config_Active.I_DirectF,
			Config_Active.D_DirectF, Motor1.Max_Iq);

	/*** Initialize IIRs ***/

	arm_biquad_cascade_df1_init_f32(&(Motor1.IIR_Id), 1,
			(float32_t*) IIR_I_Coeffs, Motor1.IIR_Id_State);
	arm_biquad_cascade_df1_init_f32(&(Motor1.IIR_Iq), 1,
			(float32_t*) IIR_I_Coeffs, Motor1.IIR_Iq_State);

	arm_biquad_cascade_df1_init_f32(&(Motor1.IIR_Velo), 1,
			(float32_t*) IIR_Velo_Coeffs, Motor1.IIR_Velo_State);
	arm_biquad_cascade_df1_init_f32(&(Motor1.IIR_Velo_aggr), 1,
			(float32_t*) IIR_Velo_aggr_Coeffs, Motor1.IIR_Velo_aggr_State);
	arm_biquad_cascade_df1_init_f32(&(Motor1.IIR_Ext_Velo), 1,
			(float32_t*) IIR_Velo_Ext_Coeffs, Motor1.IIR_Ext_Velo_State);
	/*IIR for Position*/
	arm_biquad_cascade_df1_init_f32(&(Motor1.IIR_Ext_Posi), 1,
			(float32_t*) IIR_Posi_Coeffs, Motor1.IIR_Ext_Posi_State);
	arm_biquad_cascade_df1_init_f32(&(Motor1.IIR_ENC), 1,
			(float32_t*) IIR_ENC_Coeffs, Motor1.IIR_ENC_State);
	/*IIR for Vbus*/
	arm_biquad_cascade_df1_init_f32(&(IIR_Vbus), 1,
			(float32_t*) IIR_Vbus_Coeffs, IIR_Vbus_State);
	/*IIR for position ref*/
	arm_biquad_cascade_df1_init_f32(&(IIR_Posi_ref), 1,
			(float32_t*) IIR_Posi_ref_Coeffs, IIR_Posi_ref_State);
	//IIR for temperature
	//arm_biquad_cascade_df1_init_f32(&(IIR_TempTC), 1, IIR_TempTC_Coeffs, IIR_TempTC_State);

	/*** Initialize Vbus ***/
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 5);
	Mode.Vbus_RAW = (float32_t) (HAL_ADC_GetValue(&hadc1)) * K_Vbus_ADC;
	Mode.Vbus_LowPass = IIR_Vbus_State[0] = IIR_Vbus_State[1] =
			IIR_Vbus_State[2] = IIR_Vbus_State[3] = Mode.Vbus_RAW;

	/*** Initialize UART receive ***/

	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	SET_BIT(USART_DEBUG->CR3, USART_CR3_EIE);
	/* Enable the UART Data Register not empty Interrupt */
	SET_BIT(USART_DEBUG->CR1, USART_CR1_RXNEIE);

	/*** Initialize RS-485 UART receive*/
	/*Enable receive*/
	RS485_RXEN();
	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	SET_BIT(USART_RS485->CR3, USART_CR3_EIE);
	/* Enable the UART Data Register not empty Interrupt */
	SET_BIT(USART_RS485->CR1, USART_CR1_RXNEIE);

	/*** Initialize Mode ***/
	//Mode.Debug_Log=2;
	Mode.Ctrl_Mode = Config_Active.Mode;
	Mode.Torque_Comp = 0;

	/*** AFC ***/
//	AFC_Init(&Motor1.afc_Iq_1, AFC_HARMONIC1, AFC_K, AFC_MAX);
	AFC_Init(&Motor1.afc_Iq_2, AFC_HARMONIC2, AFC_K, AFC_MAX);
	AFC_Init(&Motor1.afc_Id_1, AFC_HARMONIC1, AFC_K, AFC_MAX);


//	JointLimitP = Config_Active.P_DirectF;	// using Direct force P gain for joint limit P gain
}


/**
 * @brief Current Loop 21978Hz HIGH_LOOP_HZ
 *
 */
void ADC_Routine()
{
	int16_t i_i16;

	static uint8_t estop_prev = 0;
	uint8_t estop = 0;
	static uint32_t estop_ct = 0;
	static uint32_t estop_delay = 0;

	//Freq counter
	ADCR_Count++;
	if (ADCR_Refresh)
	{
		ADCR_Count = 1;
		ADCR_Refresh = 0;
	}

	/*** Tasks can be done without Iab & Theta ***/
	/* Update active config from config buffer */
	Update_Motor_Config();

	/* Watchdog */
	if ((Config_Active.Watchdog_Timeout != 0) && (Config_Active.Mode == 0)
			&& (Stat_Active.Enable == 1) && (!(Mode.Error & (ERROR_WATCHDOG))))
	{
		Mode.Watchdog_CT++;
		if ((Mode.Watchdog_CT > (Config_Active.Watchdog_Timeout / HIGH_LOOP_US))
				|| (Mode.Watchdog_CT == 0xFFFFFFFF))
		{
			/*Timeout!*/
			Error_Set(ERROR_WATCHDOG);
			Mode.Watchdog_CT = 0;
		}
	}

	/*** Wait on ADS8353 ***/
	ADS8353_Wait_GetData(&Motor1.ADC_IAB[0], &Motor1.ADC_IAB[1]);

	/*Phase A,B Current*/
	if ((pCal_Flash->Encoder_Config & CAL_ENC_UVW) &&	//Swap A & B
			(Mode.Manual_E_Theta == 0))
	{
		Motor1.I_A_RAW = -((int32_t) (Motor1.ADC_IAB[1]) - Motor1.ADC_B_Zero)
				* ADC_GAININV;
		Motor1.I_B_RAW = ((int32_t) (Motor1.ADC_IAB[0]) - Motor1.ADC_A_Zero)
				* ADC_GAININV;
	}
	else
	{
		Motor1.I_A_RAW = ((int32_t) (Motor1.ADC_IAB[0]) - Motor1.ADC_A_Zero)
				* ADC_GAININV;
		Motor1.I_B_RAW = -((int32_t) (Motor1.ADC_IAB[1]) - Motor1.ADC_B_Zero)
				* ADC_GAININV;
	}

	/*Calculate Ialpha beta*/
	Motor_Calc_Ialpha_Ibeta(&Motor1);

	if (Debug_Packet.Buff.Done)
	{
		UART_Debug_Routine();
		Debug_Packet.Buff.Done = 0;
	}

	Update_Motor_Stat();

#if defined(MHM_NO_EEPROM) || defined(MHM_PV_MULTITURN)
	/*** Wait on MHM Encoder ***/
	/* MHM state machine for reset ic-PV */
	switch (MHM.State)
	{
	case 0: /* Normal Operation */

		if ((MHM_Wait_Read_Posi(&Motor1.Rtr_MT, &Motor1.Rtr_ST) & (MHM_POS_NERR))
				== 0x00)
		{
#ifdef MHM_PV_MULTITURN	// ic-pv error show up as external error
			Error_Set(ERROR_ABS_POSI);
		#endif
		}
		break;
	case 1: /* Set ic-pv Preset pin */
		MHM_Wait_Write_Reg();
		MHM.State++;
		break;
	case 2: /* Reset ic-pv Preset pin */
		MHM_Wait_Write_Reg();
		MHM.State++;
		break;
	default: /* Waiting do nothing */
		MHM.State++;
		break;
	case MHM_PV_DELAY: /* Reset MHM after the delay */
		MHM_Wait_Write_Reg();
		MHM.State++;
		break;
	case MHM_RESET_DELAY:/* Reset delay elapsed */
#ifdef MHM_PV_MULTITURN
		if ((MHM_Wait_Read_Posi(&Motor1.Rtr_MT, &Motor1.Rtr_ST) & (MHM_POS_NERR)) == 0x00)
		{
			Error_Set(ERROR_ABS_POSI);
			if (MLT_MT.State == 1) /* MT Calculation primed */
				MLT_MT.State = 0;
		}
		else	/* success reset */
		{
			if (MLT_MT.State == 1) /* MT Calculation primed */
			{
				MLT_MT.State = 2;
				Trig_Slow_Loop();
			}
		}
		MHM.State = 0;
	#else
		MHM_Wait_Read_Posi(&Motor1.Rtr_MT, &Motor1.Rtr_ST);
		MHM.State = 0;
#endif
		break;

	}

#elif defined(MU_PVL_MULTITURN)
			
#else
	#error "Main Encoder definition invalid!"
#endif

#ifdef MOTOR_U180_HMND	/*** U180_HMND ARTEMIS ***/

	/*** limit torque output when close to joint limit ***/
	// coerce(data,limit) ((data) > 0 ? (min(data,limit)) : (max(data,-limit)))

	if (Motor1.I_q_Ref > 0)
	{
		//limit +tq when close to +joint limit
		Motor1.I_q_Ref = max(0,
						 min(Motor1.I_q_Ref, JointLimitP * (Config_Active.Posi_Max - JointLimitOffset - Motor1.Rtr_Theta)) );
	}
	else
	{
		//limit -tq when close to -joint limit
		Motor1.I_q_Ref = min(0,
						 max(Motor1.I_q_Ref, JointLimitP * (Config_Active.Posi_Min + JointLimitOffset - Motor1.Rtr_Theta)) );
	}

//		Motor1.I_q_Ref = coerce(Motor1.I_q_Ref,
//								JointLimitP * abs(Motor1.Rtr_Theta - Config_Active.Posi_Min) );
//		Motor1.I_q_Ref = coerce(Motor1.I_q_Ref,
//								JointLimitP * abs(Motor1.Rtr_Theta - Config_Active.Posi_Max) );

#endif
	/*** Calculate PWM outputs ***/
	Motor_CalculateOutput(&Motor1);

	/*** Error handling ***/
	/* joint limit */
	if ((Motor1.Rtr_Theta < Config_Active.Posi_Min)
			|| (Motor1.Rtr_Theta > Config_Active.Posi_Max))
		Error_Set(ERROR_JOINT_LIMIT);

	/* estop input handling */
	estop = EXT_ESTOP;	//capture input

	if (estop_prev == 0)
	{
		//previously UNestopped
		if (estop == 1)
		{
			//estop signal detected
			estop_ct++;
			if (estop_ct > ESTOP_TOGGLE_DELAY)
			{
				// delay time elapsed, set error, reset counter
				Error_Set(ERROR_WATCHDOG);
				estop_ct = 0;
				estop_prev = 1;
			}
		}
		else
			estop_ct = 0;		//no estop signal, clear counter

		estop_delay = 0;	//only count estopped time
	}
	else
	{
		//estopped
		if (estop_delay < ESTOP_LATCHING_DELAY)
			estop_delay++;
		else
		{
			//Estop latching time elapsed
			if (estop == 0)
			{
				//UNestop signal detected 
				estop_ct++;
				if (estop_ct > ESTOP_TOGGLE_DELAY)
				{
					// delay time elapsed, unestop, reset counter
					DRV_GATE_DIS(); //RESET
					LED_B_off();
					Stat_Buff.Enable = Stat_Active.Enable = 0;
					Error_Reset(ERROR_WATCHDOG);

					estop_ct = 0;
					estop_prev = 0;
				}
			}
			else
				estop_ct = 0;	//clear counter
		}
	}

	/* Critial errors */
	if ((Mode.Error & ERROR_CRITICAL) && (Mode.Calibrating == 0))
	{
		DRV_GATE_DIS(); //RESET
		LED_B_off();
		Stat_Buff.Enable = Stat_Active.Enable = 2;
		LED_R_dim(); /* Light up red led when error*/
	}
	/* absolute position missing */
	else if ((Mode.Error & ERROR_ABS_POSI)
			&& ((Mode.Ctrl_Mode == 2) || (Mode.Ctrl_Mode == 3)) && /* position modes */
			(Mode.Calibrating == 0))
	{
		DRV_GATE_DIS(); //RESET
		LED_B_off();
		Stat_Buff.Enable = Stat_Active.Enable = 2;
		LED_R_dim(); /* Light up red led when error*/
	}
	/* All other errors */
	else if ((Mode.Error & ERROR_ALL) && (Mode.Calibrating == 0))
	{
		/* Safe Damping mode */
		Stat_Buff.Enable = Stat_Active.Enable = 3;
		TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 0;
		LED_R_dim(); /* Light up red led when error*/
	}
	else
	{
		if ((pCal_Flash->Encoder_Config & CAL_ENC_UVW) &&	//Swap A & C
				(Mode.Manual_E_Theta == 0))
		{
			TIM1->CCR1 = (uint32_t) (Motor1.TC); //V1_C
			TIM1->CCR2 = (uint32_t) (Motor1.TB); //V1_B
			TIM1->CCR3 = (uint32_t) (Motor1.TA); //V1_A
		}
		else
		{
			TIM1->CCR1 = (uint32_t) (Motor1.TA); //V1_A
			TIM1->CCR2 = (uint32_t) (Motor1.TB); //V1_B
			TIM1->CCR3 = (uint32_t) (Motor1.TC); //V1_C
		}
		LED_R_off(); /* red led off*/
	}

	/*** update all stats ***/
	Stat_Active.Present_Id = Motor1.I_d;
	if (Mode.Torque_Comp)
	{
		Stat_Active.Present_Iq = Motor1.I_q - Motor1.I_Torque_Comp;
	}
	else
		Stat_Active.Present_Iq = Motor1.I_q;

	Stat_Active.Present_Velo = Motor1.Rtr_Theta_Dot_LP;
	Stat_Active.Present_Posi = Motor1.Rtr_Theta_LP;

	Stat_Active.Goal_Id = Motor1.I_d_Ref;
	Stat_Active.Goal_Iq = Motor1.I_q_Ref;
	Stat_Active.Goal_Velo = Motor1.ThetaDot_Ref;
	Stat_Active.Goal_Posi = Motor1.Theta_Ref;

	/* Iq, Temp_Winding and Temp_MOSFET */
	i_i16 = (int16_t) (Motor1.I_q * 100.0f);
	Stat_Active.Iq_Temps = (((uint32_t) (*(uint16_t*) &i_i16)) << 16)
			+ (((uint32_t) ((uint8_t) (Stat_Active.Temp_Winding))) << 8)
			+ ((uint32_t) ((uint8_t) (Stat_Active.Temp_MOSFET)));
	/* debug logging */
	ADC_Debug();
}


/**
 * @brief Control routine @ 4000Hz MID_LOOP_HZ
 *
 */
void Mid_Ctrl_Routine()
{
	static uint8_t Loop_CT = 0;	//for 800Hz loop divider
	//Freq counter
	MidCR_Count++;
	if (MidCR_Refresh)
	{
		MidCR_Count = 1;
		MidCR_Refresh = 0;
	}

	/*** Get Vbus Enable the selected ADC software conversion for regular group ***/
	hadc1.Instance->CR2 |= (uint32_t) ADC_CR2_SWSTART;

	/*** external encoder ***/
#if defined(EXT_MA_ENCODER) || defined(EXT_AS_ENCODER)

	Motor1.Ext_Theta_Raw_Prev = Motor1.Ext_Theta_Raw;

#ifdef EXT_MA_ENCODER
	Motor1.Ext_Theta_Raw = MA_Read_Posi();
#else
		Motor1.Ext_Theta_Raw = AS5047_Simple_Read_Fast();
	#endif

	/*Multi turn */
	if (((Motor1.Ext_Theta_Raw_Prev) < (EXT_ENC_RES / 4))
			&& ((Motor1.Ext_Theta_Raw) > (EXT_ENC_RES / 4 * 3)))
	{
		Motor1.Ext_Theta_Dot = ExtThetaDot2RAD
				* (Motor1.Ext_Theta_Raw - Motor1.Ext_Theta_Raw_Prev
						- EXT_ENC_RES);
	}
	else if (((Motor1.Ext_Theta_Raw_Prev) > (EXT_ENC_RES / 4 * 3))
			&& ((Motor1.Ext_Theta_Raw) < (EXT_ENC_RES / 4)))
	{
		Motor1.Ext_Theta_Dot = ExtThetaDot2RAD
				* (EXT_ENC_RES + Motor1.Ext_Theta_Raw
						- Motor1.Ext_Theta_Raw_Prev);
	}
	else
	{
		Motor1.Ext_Theta_Dot = ExtThetaDot2RAD
				* (Motor1.Ext_Theta_Raw - Motor1.Ext_Theta_Raw_Prev);
	}
	/*Ignore multi-turn if in calibration*/
	if (Mode.Calibrating == 1)
		Motor1.Ext_Theta = Motor1.Ext_Theta_Raw;
	else if (Mode.Calibrating == 2)
	{
		Motor1.Ext_Theta = ExtTheta2RAD * Motor1.Ext_Theta_Raw;
	}
	else
	{
		// CHECK!!!
		Motor1.Ext_Theta = ExtTheta2RAD * Motor1.Ext_Theta_Raw
				- Motor1.Ext_Theta_ZeroOffset;
	}
	/* Low pass for speed */
	arm_biquad_cascade_df1_f32_1Stage1Sample(&(Motor1.IIR_Ext_Velo),
			&(Motor1.Ext_Theta_Dot), &(Motor1.Ext_Theta_Dot_LP));
//	/* lowpass filter for theta using external encoder */ 
//	arm_biquad_cascade_df1_f32_1Stage1Sample(&(Motor1.IIR_Ext_Posi), &(Motor1.Ext_Theta), &(Motor1.Ext_Theta_LP));

#endif

	/*** Controllers ***/
	switch (Mode.Ctrl_Mode)
	{
	default:
		break;
	case 0x00: /* 0: Torque Mode */
		Motor1.I_q_Ref = coerce(Motor1.I_q_cmd, Motor1.Max_Iq_Thermal);
		Motor1.I_d_Ref = coerce(Motor1.I_d_cmd, Motor1.Max_Iq_Thermal);

		break;
	case 0x01: /* 1: Velocity Mode */
		Motor1.ThetaDot_Ref = Motor1.ThetaDot_cmd;
		Motor1.I_q_Ref = coerce(
				arm_pid_f32_L(&(Motor1.PID_Velo),
						(Motor1.ThetaDot_Ref - Motor1.Rtr_Theta_Dot_LP)),
				Motor1.Max_Iq_Thermal);
		Motor1.I_d_Ref = 0;
		break;
	case 0x02: /* 2: Position Mode */

		/* smooth Position cmd input */
//		arm_biquad_cascade_df1_f32_1Stage1Sample(&IIR_Posi_ref, &(Motor1.Theta_cmd), &(Motor1.Theta_Ref));
		Motor_Theta_ref_S_Curve(&Motor1.Theta_cmd, &Motor1.Theta_Ref,
				&Motor1.ThetaDot_traj, &Motor1.ThetaDotDot_traj);

		Motor1.ThetaDot_Ref = arm_pid_f32_L_Traj(&(Motor1.PID_Posi),
				deadband((Motor1.Theta_Ref - Motor1.Rtr_Theta_LP),
						THETA_DEADBAND),
				(Motor1.ThetaDot_traj - Motor1.Rtr_Theta_Dot_LP));
		Motor1.I_q_Ref = coerce(
				arm_pid_f32_L(&(Motor1.PID_Velo),
						(Motor1.ThetaDot_Ref - Motor1.Rtr_Theta_Dot_LP)),
				Motor1.Max_Iq_Thermal);

		Motor1.I_d_Ref = 0;
		break;
	case 0x03: /* 3:Direct Force PID */
		Motor1.Theta_Ref = Motor1.Theta_cmd;
		Motor1.ThetaDot_Ref = Motor1.ThetaDot_cmd;

		Motor1.I_q_Ref = coerce(
				arm_pid_f32_L_Traj_FF(&(Motor1.PID_DF),
						(Motor1.Theta_Ref - Motor1.Rtr_Theta_LP),
						(Motor1.ThetaDot_Ref - Motor1.Rtr_Theta_Dot_LP),
						Motor1.I_q_cmd), Motor1.Max_Iq_Thermal);

		Motor1.I_d_Ref = coerce(Motor1.I_d_cmd, Motor1.Max_Iq_Thermal);
		break;

	}

	/*** Packet timeout counter ***/
	if (Debug_Packet.Packet_Stat != IDLE) /* if not idle -> count up */
		Debug_Packet.Packet_NonIdle_CT++;
	else
		Debug_Packet.Packet_NonIdle_CT = 0;
	if (Debug_Packet.Packet_NonIdle_CT > PACKET_TIMEOUT)
		Debug_Packet.Packet_Stat = IDLE;

	/* RS485_Packet timeout counter */
	if (RS485_Packet.Packet_Stat != IDLE) /* if not idle -> count up */
		RS485_Packet.Packet_NonIdle_CT++;
	else
		RS485_Packet.Packet_NonIdle_CT = 0;
	if (RS485_Packet.Packet_NonIdle_CT > PACKET_TIMEOUT) /* timeout */
	{
		RS485_Packet.Packet_Stat = IDLE;
		//TODO:add warning counter
		printf("RS485 packet timeout! \n");
	}
	/* Bulk timeout counter */
	if (RS485_Packet.Bulk_Wait) /* if not idle -> count up */
		RS485_Packet.Bulk_NonIdle_CT++;
	else
		RS485_Packet.Bulk_NonIdle_CT = 0;
	if (RS485_Packet.Packet_NonIdle_CT > PACKET_TIMEOUT) /* timeout */
	{
		RS485_Packet.Bulk_Wait = 0;
		//TODO:add warning counter
		printf("RS485 Bulk packet timeout! \n");
	}

	/*** Check back on Vbus ***/
	while (!(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC))) {};
	Mode.Vbus_RAW = (float32_t) (hadc1.Instance->DR) * K_Vbus_ADC;
	/* Lowpass for Vbus */
	arm_biquad_cascade_df1_f32_1Stage1Sample(&IIR_Vbus, &(Mode.Vbus_RAW),
			&(Mode.Vbus_LowPass));
	Stat_Active.Vbus = Mode.Vbus_LowPass;
	if ((Mode.Vbus_RAW < Config_Active.Vbus_Min)
			|| (Mode.Vbus_RAW > Config_Active.Vbus_Max))
	{
		Error_Set(ERROR_HARDWARE_FAULT);
	}
	else
	{
		//Error_Reset(ERROR_INPUT_VOLTAGE);
	}

	/*** Lower freq tasks ***/
	Loop_CT++;
	if (Loop_CT > 39) /* 100Hz loop (4000Hz/40) */
	{
		Loop_CT = 0;

	}

	/*** debug logging ***/
	Ctrl_Debug();

}


/**
 * @brief Slow loop routine @ 5Hz
 *
 */
void Slow_Loop_Routine()
{
	static uint8_t Loop_CT = 0;	//for 1Hz loop divider
	float32_t Max_Temperature, Max_Iq;
	float32_t Temp_Homing_Offset;

	/*** Reading temperatures ***/
	MAX31855_Read(&TC_Reading);
	//if (TC_Reading.Fault) TC_Reading=MAX31855_Read();
	Stat_Active.Temp_IC = TC_Reading.in;
	Stat_Active.Temp_Winding = TC_Reading.tc;

	/* TMP112 temperature sensor for mosfet */
	TMP112_ReadTemp(TMP112_ADDR0, &Stat_Active.Temp_MOSFET,
			TMP112_EXTENDED_MODE);

	/* Calculate Imax for temp limit */
	Max_Temperature = max(Stat_Active.Temp_Winding, Stat_Active.Temp_MOSFET);
	if (Max_Temperature > Config_Active.Temp_Min)
	{
		/*start to limit the power*/
		Max_Iq = (Config_Active.Temp_Max - Max_Temperature)
				* Config_Active.I_Max
				/ (Config_Active.Temp_Max - Config_Active.Temp_Min);
		if (Max_Iq < 0)
			Max_Iq = 0;
		Motor1.Max_Iq_Thermal = Max_Iq;

		Error_Set(WARNING_OVER_TEMP);
	}
	else
	{
		Motor1.Max_Iq_Thermal = Config_Active.I_Max;	//Config_Max.I_Max;
		Error_Reset(WARNING_OVER_TEMP);
	}

	/*** Finding MT Offset ***/
	if (MLT_MT.State == 2) /* Calculate */
	{
		if (MLT_MT.Tolerance == 0.0f)
		{
			/* Set current position */
			Temp_Homing_Offset = WrapThetaF(
					Config_Active.Homing_Offset + MLT_MT.Exp_Theta
							- Stat_Active.Present_Posi);
			if ((Temp_Homing_Offset >= Config_Min.Homing_Offset)
					&& (Temp_Homing_Offset <= Config_Max.Homing_Offset))
			{
				Config_Active.Homing_Offset = Config_Buff.Homing_Offset =
						Temp_Homing_Offset;
				// CHECK!!!!
				Motor1.Rtr_Theta_Offset32 =
						(int32_t) ((pCal_Flash->Theta_Zero_Offset
								+ Config_Active.Homing_Offset
								+ Motor1.Ext_Theta_ZeroOffset) * RAD2RtrTheta);

				/* Save to flash */
				Save_Config();
			}

		}
		else
		{
			/* Finding MT */
			if (MLT_Find_MT(Motor1.Rtr_ST, Motor1.Rtr_MT) == 0) /* No error */
			{
				/* Set new MT offset */
				Motor1.Rtr_MT_Offset = Config_Active.Rtr_MT_Offset =
						Config_Buff.Rtr_MT_Offset = MLT_MT.MT_Offset;
				/* reset error */
				Error_Reset(ERROR_ABS_POSI);
				/* Save to flash */
				Save_Config();
			}
		}
		MLT_MT.State = 0;
	}

	/*** Saving Configs ***/
	if (Mode.Save_to_Flash)
	{
		/*Flash write requested*/
		if (memcmp(&Config_Active, pConfig_Flash, 4 * MOTOR_CONFIG_SIZE) != 0)
		{
			/*Write Motor Config to Flash*/
			if (Write_Motor_Config(&Config_Active) != 0)
			{
				/*Error when writing flash*/
				/*TODO:*/
			}
		}
		Mode.Save_to_Flash = 0;
	}

	/*** DRV8305 Faults ***/
	DRV8305_REG[0] = DRV8320_Read(DRV_ADDR_FAULT1);
	DRV8305_REG[1] = DRV8320_Read(DRV_ADDR_FAULT2);
	if (DRV8305_REG[0] & DRV_FAULT)
	{
		Error_Set(ERROR_HARDWARE_FAULT);
	}

	Loop_CT++;
	if (Loop_CT > 4)
	{
		//1Hz loop (5Hz/5)
		Loop_CT = 0;

		Debug_Terminal();

	}

}

/**
 * @brief Calculate CPU usage
 *
 * @return CPU percentage
 */
CPUPercent_TypeDef Calc_CPU()
{
	uint32_t tmp = 0;
	CPUPercent_TypeDef CPUt;
	/*Display CPU usage*/

	//Get idle count
#ifdef VECT_TAB_SRAM
	tmp=(10000000 - idle_ct)/10000;     //SRAM
	//tmp=(5000000 - idle_ct)/5000;     //SRAM
#else
	tmp = (18000000 - idle_ct) / 18000;     //flash
	//tmp=(9000000 - idle_ct)/9000;     //flash
#endif

	idle_refresh = 1;	//Refresh idle counter
	if (tmp > 1000)
		tmp = 1000;
	CPUt.CPU_Percent = tmp;

	/*ADC Loop Hz*/
	CPUt.ADC_Freq = ADCR_Count;
	ADCR_Refresh = 1;

	/*Mid Control Loop Hz*/
	CPUt.MidCtrl_Freq = MidCR_Count;
	MidCR_Refresh = 1;

	return CPUt;
}

/**
 * @brief Debug terminal print
 *
 */
void Debug_Terminal()
{

	CPUPercent_TypeDef CPU;

	CPU = Calc_CPU();

	printf("CPU:%03ld.%01ld %ld %ld|", CPU.CPU_Percent / 10, CPU.CPU_Percent % 10,
			CPU.ADC_Freq, CPU.MidCtrl_Freq);

	printf("Temp: ");
	if (TC_Reading.Fault)
		printf("E%d!%.1f %.1f ", TC_Reading.Fault, TC_Reading.tc,
				TC_Reading.in);
	else
		printf("%.1f %.1f ", TC_Reading.tc, TC_Reading.in);

	printf("%.1f|", Stat_Active.Temp_MOSFET);

	printf("V:%.1f|", Stat_Active.Vbus);

	//printf("ADC:%04x %0x|",Motor1.ADC_IAB[0],Motor1.ADC_IAB[1]);

	printf("ENC:%04ld %04lx|", Motor1.Rtr_MT, Motor1.Rtr_ST);

	printf("DRV:%03x %03x\n", DRV8305_REG[0], DRV8305_REG[1]);

}


//#pragma optimize=medium
/**
 * @brief Debug output in ADC loop
 *
 */
static inline void ADC_Debug()
{
	if (Mode.Debug_Log == 1)
	{
		Debug_Buff[0] = 0xff;
		(*Debug_Data_f)[0] = Mode.Vbus_RAW;
		(*Debug_Data_f)[1] = Motor1.V_q;//I_q_Ref_AfterComp;//Rtr_Theta_Dot_LP;//
		(*Debug_Data_f)[2] = Motor1.I_q_RAW;
		(*Debug_Data_f)[3] = Motor1.Rtr_Theta;

		HUART_DEBUG.gState = HAL_UART_STATE_READY;
		HAL_UART_Transmit_DMA(&HUART_DEBUG, Debug_Buff, 17);

	}
}


//#pragma optimize=medium
/**
 * @brief Debug in position loop
 *
 */
static inline void Ctrl_Debug()
{
	if (Mode.Debug_Log == 2)
	{
		Debug_Buff[0] = 0xff;
		(*Debug_Data_f)[0] = Motor1.V_q;
		(*Debug_Data_f)[1] = Motor1.V_d;

		(*Debug_Data_f)[2] = Motor1.I_q_RAW;	//I_q;
		(*Debug_Data_f)[3] = Motor1.I_q_Ref_AfterComp;
		(*Debug_Data_f)[4] = Motor1.I_d_RAW;	//I_d;
		(*Debug_Data_f)[5] = Motor1.I_d_Ref;

		(*Debug_Data_f)[6] = Motor1.Rtr_Theta_Elec;
		(*Debug_Data_f)[7] = Motor1.Rtr_Theta_LP;
		(*Debug_Data_f)[8] = Motor1.Rtr_Theta;
		(*Debug_Data_f)[9] = Motor1.Theta_Ref;

		(*Debug_Data_f)[10] = Motor1.Rtr_Theta_Dot_LP;
		(*Debug_Data_f)[11] = Motor1.ThetaDot_Ref;	//Rtr_Theta_Dot_LP_aggr;//

		(*Debug_Data_f)[12] = Motor1.Ext_Theta;
		(*Debug_Data_f)[13] = Mode.Vbus_RAW;	//LowPass;

		if (Mode.Manual_E_Theta)
		{
			(*Debug_Data_f)[14] = *(float32_t*) (&Motor1.ADC_IAB[0]);
			(*Debug_Data_f)[15] = *(float32_t*) (&Motor1.Theta_manual);
		}
		else
		{
			(*Debug_Data_f)[14] = Motor1.afc_Iq_2.out;
			(*Debug_Data_f)[15] = Motor1.afc_Id_1.out;
		}

		//HUART_DEBUG.gState = HAL_UART_STATE_READY;
		HAL_UART_Transmit_DMA(&HUART_DEBUG, Debug_Buff, 16 * 4 + 1);
	}

}
