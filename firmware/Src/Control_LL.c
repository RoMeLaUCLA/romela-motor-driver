/**
 * @file 	Control_LL.c
 * @brief 	Low level control subroutines.
 *
 * @author 	Tym Zhu
 */
#include "Control_LL.h"

#include "main.h"
#include "util.h"

#include "error.h"


extern arm_biquad_casd_df1_inst_f32 IIR_Posi_ref;
extern float32_t IIR_Posi_ref_State[4];

/**
 * @brief Motor Torque Enable
 *
 */
void Motor_Enable()
{
	TIM1->CCR1 = HALF_PWM_CCR; //V1_A
	TIM1->CCR2 = HALF_PWM_CCR; //V1_B
	TIM1->CCR3 = HALF_PWM_CCR; //V1_C

	Error_Reset(ERROR_JOINT_LIMIT);
	Error_Reset(ERROR_WATCHDOG);
	Error_Reset(ERROR_HARDWARE_FAULT);

	arm_pid_reset_f32_L(&(Motor1.PID_Iq));
	arm_pid_reset_f32_L(&(Motor1.PID_Id));
	arm_pid_reset_f32_L(&(Motor1.PID_Velo));
	arm_pid_reset_f32_L(&(Motor1.PID_Posi));
	arm_pid_reset_f32_L(&(Motor1.PID_DF));
	Stat_Active.Goal_Id = Stat_Buff.Goal_Id = Motor1.I_d_cmd = 0;
	Stat_Active.Goal_Iq = Stat_Buff.Goal_Iq = Motor1.I_q_cmd = 0;

	Stat_Active.Goal_Velo = Stat_Buff.Goal_Velo = Motor1.ThetaDot_cmd =
			Motor1.ThetaDot_traj = 0;
	Stat_Active.Goal_Posi = Stat_Buff.Goal_Posi = Motor1.Theta_cmd =
			Motor1.Theta_Ref = IIR_Posi_ref_State[0] = IIR_Posi_ref_State[1] =
					IIR_Posi_ref_State[2] = IIR_Posi_ref_State[3] =
							Motor1.Rtr_Theta_LP;

	Motor1.afc_Iq_1.sin_i = Motor1.afc_Iq_1.cos_i = 0.0f;
	Motor1.afc_Iq_2.sin_i = Motor1.afc_Iq_2.cos_i = 0.0f;

	DRV_GATE_EN();		//SET
	LED_B_dim();

}

/**
 * @brief Motor Torque Disable
 *
 */
void Motor_Disable()
{
	DRV_GATE_DIS(); //RESET
	LED_B_off();
	Error_Reset(ERROR_JOINT_LIMIT);
	Error_Reset(ERROR_WATCHDOG);
	Error_Reset(ERROR_HARDWARE_FAULT);
}

/**
 * @brief Calculate Ialpha Ibeta
 *
 * @param MT pointer to motor structure
 */
void Motor_Calc_Ialpha_Ibeta(Motor_TypeDef *MT)
{

	/*Clarke Transform to Ialpha Ibeta*/
	arm_clarke_f32(MT->I_A_RAW, MT->I_B_RAW, &(MT->I_alpha), &(MT->I_beta));

}

/**
 * @brief Calculate PWM output
 *
 * @param MT pointer to motor structure
 */
void Motor_CalculateOutput(Motor_TypeDef *MT)
{

	/*** Multi turn and Diff for Velocity ***/
//#if (MHM_MT_SIZE!=0)
	/* Diff raw Theta to get ThetaDot */
	// MHM holds MT value, no need to check zero crossing
	MT->Rtr_Theta_Dot = (((int64_t) (MT->Rtr_MT) * RTR_ENC_RES + MT->Rtr_ST)
			- ((int64_t) (MT->Rtr_MT_Prev) * RTR_ENC_RES + MT->Rtr_ST_Prev))
			* RtrThetaDot2RAD;

//#else //defined(MHM_NO_MULTITURN)
//	
//	if ( ((MT->Rtr_ST_Prev)<(RTR_ENC_RES/4)) && ((MT->Rtr_ST)>(RTR_ENC_RES/4*3)) )
//	{
//		MT->Rtr_MT--;
//		//if ((MT->Rtr_MT)<-MAX_TURN) MT->Rtr_MT=(MAX_TURN-1);
//		MT->Rtr_Theta_Dot = (MT->Rtr_ST - MT->Rtr_ST_Prev - RTR_ENC_RES) * RtrThetaDot2RAD;
//	}
//	else if ( ((MT->Rtr_ST_Prev)>(RTR_ENC_RES/4*3)) && ((MT->Rtr_ST)<(RTR_ENC_RES/4)) )
//	{
//		MT->Rtr_MT++;
//		//if ((MT->Rtr_MT)>(MAX_TURN-1)) MT->Rtr_MT=-MAX_TURN;
//		MT->Rtr_Theta_Dot = (RTR_ENC_RES + MT->Rtr_ST - MT->Rtr_ST_Prev) * RtrThetaDot2RAD;
//	}
//	else
//	{
//		MT->Rtr_Theta_Dot = (MT->Rtr_ST - MT->Rtr_ST_Prev) * RtrThetaDot2RAD;
//	}
//	
////#else
////	static_assert(0, "MHM MULTITURN invalid!");
//#endif

	/* Prepare Prev value */
	MT->Rtr_ST_Prev = MT->Rtr_ST;
	MT->Rtr_MT_Prev = MT->Rtr_MT;

	/* Low pass for Velocity */

//	MT->Rtr_Theta_Dot_LP = MT->Rtr_Theta_Dot;
	arm_biquad_cascade_df1_f32_1Stage1Sample(&(MT->IIR_Velo),
			&(MT->Rtr_Theta_Dot), &(MT->Rtr_Theta_Dot_LP));
//	arm_biquad_cascade_df1_f32_1Stage1Sample(&(MT->IIR_Velo_aggr), &(MT->Rtr_Theta_Dot), &(MT->Rtr_Theta_Dot_LP));

	/*** Calculate Rtr_Theta ***/
	if (Mode.Calibrating == 1) /*Use raw data for calibration*/
	{
		MT->Rtr_Theta_64 = MT->Rtr_ST;
		MT->Rtr_Theta = MT->Rtr_Theta_64;
	}
	else if (Mode.Calibrating == 2) /* when calibrating cogging */
	{
		MT->Rtr_Theta_64 = (int64_t) (MT->Rtr_MT + MT->Rtr_MT_Offset)
				* RTR_ENC_RES + MT->Rtr_ST;
		/* without offset */
		MT->Rtr_Theta = WrapRtrTheta64(MT->Rtr_Theta_64) * RtrTheta2RAD;
	}
	else /* Normal mode calculating Rtr_Theta */
	{
		MT->Rtr_Theta_64 = (int64_t) (MT->Rtr_MT + MT->Rtr_MT_Offset)
				* RTR_ENC_RES + MT->Rtr_ST;
		MT->Rtr_Theta = WrapRtrTheta64(
				MT->Rtr_Theta_64 + MT->Rtr_Theta_Offset32) * RtrTheta2RAD;
	}
	/* lowpass for position */
	arm_biquad_cascade_df1_f32_1Stage1Sample(&(MT->IIR_ENC), &(MT->Rtr_Theta),
			&(MT->Rtr_Theta_LP));
	MT->Rtr_ST_LP = WrapRtrThetaU32(
			(int32_t) (MT->Rtr_Theta_LP * RAD2RtrTheta)
					- MT->Rtr_Theta_Offset32);
	MT->Rtr_ST_f = (float32_t) (MT->Rtr_ST);

	/*** Calculate Id Iq ***/
	/* Calculate Electric Angle */
	if (Mode.Manual_E_Theta)
	{
		/*Manual Theta mode to do calibration*/
		MT->Rtr_Theta_Elec = fmodf((MT->Theta_manual) * K_THETA_E, 360.0f);
	}
	else
	{
		MT->Rtr_Theta_Elec = fmodf(
				(MT->Rtr_ST + MT->Rtr_Elec_Offset) * K_THETA_E, 360.0f);
	}
	/*update sine cosine */
	arm_sin_cos_f32(MT->Rtr_Theta_Elec, &(MT->Sin_Theta_Elec),
			&(MT->Cos_Theta_Elec));
	/*Clarke & Park Transform to Id Iq*/
	arm_park_f32(MT->I_alpha, MT->I_beta, &(MT->I_d_RAW), &(MT->I_q_RAW),
			MT->Sin_Theta_Elec, MT->Cos_Theta_Elec);
	/* IIR for Id and Iq */
	arm_biquad_cascade_df1_f32_1Stage1Sample(&(MT->IIR_Id), &(MT->I_d_RAW),
			&(MT->I_d));
	arm_biquad_cascade_df1_f32_1Stage1Sample(&(MT->IIR_Iq), &(MT->I_q_RAW),
			&(MT->I_q));
	//MT->I_d=(MT->I_d_RAW);
	//MT->I_q=(MT->I_q_RAW);

	/*** Current PID Controller ***/
	/* Torque Compensation */
	if (Mode.Torque_Comp)
	{
		MT->I_Torque_Comp = (pCal_Flash->Torque_Amp)
				* pCal_Flash->Torque_Cal[(MT->Rtr_ST)
						>> (RTR_ENC_RES_BIT - CAL_TORQUE_RES_BIT)];

		if (MT->Rtr_Theta_Dot_LP > (pCal_Flash->Friction_Comp_Velocity))
			MT->I_Torque_Comp = MT->I_Torque_Comp
					+ pCal_Flash->Friction_Comp_Slope
							* (MT->Rtr_Theta_Dot_LP
									- pCal_Flash->Friction_Comp_Velocity)
					+ pCal_Flash->Friction_Comp_Amp;

		else if (MT->Rtr_Theta_Dot_LP < -(pCal_Flash->Friction_Comp_Velocity))
			MT->I_Torque_Comp = MT->I_Torque_Comp
					+ pCal_Flash->Friction_Comp_Slope
							* (MT->Rtr_Theta_Dot_LP
									+ pCal_Flash->Friction_Comp_Velocity)
					- pCal_Flash->Friction_Comp_Amp;
		else
			MT->I_Torque_Comp = (pCal_Flash->Friction_Comp_Amp)
					* MT->Rtr_Theta_Dot_LP
					/ (pCal_Flash->Friction_Comp_Velocity) + MT->I_Torque_Comp;

		MT->I_q_Ref_AfterComp = MT->I_q_Ref + MT->I_Torque_Comp;
	}
	else
		MT->I_q_Ref_AfterComp = MT->I_q_Ref;

	/* AFC */
// 	AFC_Calc(&Motor1.afc_Iq_1, MT->Rtr_Theta_Elec, MT->I_q_Ref_AfterComp - MT->I_q_RAW);
	AFC_Calc(&Motor1.afc_Iq_2, MT->Rtr_Theta_Elec,
			MT->I_q_Ref_AfterComp - MT->I_q_RAW);
	AFC_Calc(&Motor1.afc_Id_1, MT->Rtr_Theta_Elec, MT->I_d_Ref - MT->I_d_RAW);
	/*  */
	if (Mode.Manual_V) /*Manual Id Iq */
	{
		MT->V_d = MT->V_d_cmd;
		MT->V_q = MT->V_q_cmd;
	}
	else
	{
		//q_ref for desired torque; d_ref=0;
		MT->PID_Id.Limit = (MAX_Vd * Mode.Vbus_LowPass);
		MT->PID_Iq.Limit = (MAX_Vq * Mode.Vbus_LowPass);
		if (Mode.Torque_Comp)
		{
			MT->V_d = (arm_pid_f32_L(&(MT->PID_Id), (MT->I_d_Ref - MT->I_d))
					+ MT->afc_Id_1.out) / Mode.Vbus_LowPass;
			MT->V_q = (arm_pid_f32_L(&(MT->PID_Iq),
					(MT->I_q_Ref_AfterComp - MT->I_q)) + MT->afc_Iq_2.out)
					/ Mode.Vbus_LowPass;
		}
		else
		{
			MT->V_d = arm_pid_f32_L(&(MT->PID_Id), MT->I_d_Ref - MT->I_d)
					/ Mode.Vbus_LowPass;
			MT->V_q = arm_pid_f32_L(&(MT->PID_Iq),
					MT->I_q_Ref_AfterComp - MT->I_q) / Mode.Vbus_LowPass;
		}

//		/* V= Thetadot *Km */
//		MT->V_q += MOTOR_KM * MT->Rtr_Theta_Dot_LP_aggr / Mode.Vbus_LowPass;

//		if (Mode.Torque_Comp)	
//		{
////			MT->V_q = MT->V_q +(MT->Rtr_Theta_Dot_LP*
////								 (	MOTOR1_KM) )/(Mode.Vbus_LowPass);
////			MT->V_q = MT->V_q + (MT->Rtr_Theta_Dot_LP*
////								 (	((float32_t)Flux_Cal_Iq[(MT->Rtr_Theta_Raw)>>2])
////								  	*K_FLUX_IQ_COMP +MOTOR1_KM ) )/(Mode.Vbus_LowPass);
////			MT->V_d = MT->V_d + (MT->Rtr_Theta_Dot_LP*
////								 (	((float32_t)Flux_Cal_Id[(MT->Rtr_Theta_Raw)>>2])
////								  	*K_FLUX_ID_COMP  ) )/(Mode.Vbus_LowPass);
//			MT->V_q = MT->V_q +(MT->Rtr_Theta_Dot_LP*
//								 (	MOTOR1_KM) )/(Mode.Vbus_LowPass);
//			
//		}
//		else
//		{
////			
////			MT->V_q = MT->V_q + (MT->Rtr_Theta_Dot_LP*
////								 (	((float32_t)Flux_Cal[(MT->Rtr_Theta_Raw)>>2])
////								  	*K_FLUX_COMP+MOTOR1_KM) )/(Mode.Vbus_LowPass);
//			
////			MT->V_q = MT->V_q +(MT->Rtr_Theta_Dot_LP*
////								 (	MOTOR1_KM) )/(Mode.Vbus_LowPass);
//		}
//	//	else	MT->I_q_Ref_AfterComp=(MT->I_q_Ref);

	}
	/* Limit the abs value */
	arm_sqrt_f32(MT->V_d * MT->V_d + MT->V_q * MT->V_q, &(MT->V_r));

	if (MT->V_r > MAX_PWM_RATIO)
	{
		MT->V_d = MT->V_d / MT->V_r * MAX_PWM_RATIO;
		MT->V_q = MT->V_q / MT->V_r * MAX_PWM_RATIO;
		//MT->V_r = MAX_PWM_RATIO;
	}

	/*Inverse Clarke & Park Transform */
	arm_inv_park_f32((MT->V_d), (MT->V_q), &(MT->V_alpha), &(MT->V_beta),
			MT->Sin_Theta_Elec, MT->Cos_Theta_Elec);

	/* svpwm stuff */
	MT->X = (int32_t) ((-MAX_PWM_CCR_F) * MT->V_beta);
	MT->Y = (int32_t) (SQRT3_PWM_CCR_F * MT->V_alpha
			- MAX_PWM_CCR_F * MT->V_beta) / 2;
	MT->Z = (int32_t) ((-MAX_PWM_CCR_F) * MT->V_beta
			- SQRT3_PWM_CCR_F * MT->V_alpha) / 2;

	if (MT->Y < 0)
	{
		if (MT->Z < 0)
			MT->Sector = 5;
		else
		{
			if (MT->X > 0)
				MT->Sector = 3;
			else
				MT->Sector = 4;
		}
	}
	else
	{
		if (MT->Z < 0)
		{
			if (MT->X > 0)
				MT->Sector = 1;
			else
				MT->Sector = 6;
		}
		else
			MT->Sector = 2;
	}
	switch (MT->Sector)
	{
	default:
		break;
	case 1:
	case 4:
		MT->TA = HALF_PWM_CCR + (MT->X - MT->Z) / 2; //V1_A
		MT->TB = MT->TA + MT->Z; //V1_B
		MT->TC = MT->TB - MT->X; //V1_C
		break;
	case 2:
	case 5:
		MT->TA = HALF_PWM_CCR + (MT->Y - MT->Z) / 2; //V1_A
		MT->TB = MT->TA + MT->Z; //V1_B
		MT->TC = MT->TA - MT->Y; //V1_C
		break;
	case 3:
	case 6:
		MT->TA = HALF_PWM_CCR + (MT->Y - MT->X) / 2; //V1_A
		MT->TC = MT->TA - MT->Y; //V1_C
		MT->TB = MT->TC + MT->X; //V1_B
		break;

	}
}

/**
 * @brief Initialize AFC structure
 *
 * @param afc	pointer to AFC structure
 * @param harm
 * @param k
 * @param max
 */
void AFC_Init(AFC_TypeDef *afc, float32_t harm, float32_t k, float32_t max)
{
	afc->harmonic = harm;
	afc->k = k;
	afc->max = max;
	afc->sin_i = afc->cos_i = 0.0f;
}

/**
 * @brief Calulate AFC output
 *
 * @param afc
 * @param theta
 * @param err
 * @return
 */
float32_t AFC_Calc(AFC_TypeDef *afc, float32_t theta, float32_t err)
{
	arm_sin_cos_f32(afc->harmonic * theta, &(afc->sin), &(afc->cos));
	afc->sin_i += afc->k * err * afc->sin;
	afc->cos_i += afc->k * err * afc->cos;
	/* Limit the abs value */
	arm_sqrt_f32(afc->sin_i * afc->sin_i + afc->cos_i * afc->cos_i,
			&(afc->r_i));
	if (afc->r_i > afc->max)
	{
		afc->sin_i = afc->sin_i / afc->r_i * afc->max;
		afc->cos_i = afc->cos_i / afc->r_i * afc->max;
	}

	afc->out = afc->sin_i * afc->sin + afc->cos_i * afc->cos;
	afc->sin_i *= AFC_DAMPING;
	afc->cos_i *= AFC_DAMPING;
	return (afc->out);

}

/**
 * @brief Tick difference for timing
 *
 * @return
 */
uint32_t Get_Tick_Diff()
{
	static uint32_t Prev = 0;
	uint32_t Curr, Diff;
	Curr = HAL_GetTick();
	Diff = Curr - Prev;
	Prev = Curr;
	return Diff;
}

/**
 * @brief Simplified filter, arm math
 *
 * @param S		pointer to filter
 * @param pSrc	pointer to input
 * @param pDst	pointer to output
 */
void arm_biquad_cascade_df1_f32_1Stage1Sample(
		const arm_biquad_casd_df1_inst_f32 *S, float32_t *pSrc, float32_t *pDst)
{
	/* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
	/* Store the result in the accumulator in the destination buffer. */
	*pDst = (S->pCoeffs[0] * (*pSrc)) + (S->pCoeffs[1] * S->pState[0])
			+ (S->pCoeffs[2] * S->pState[1]) + (S->pCoeffs[3] * S->pState[2])
			+ (S->pCoeffs[4] * S->pState[3]);

	/* Every time after the output is computed state should be updated. */
	/* The states should be updated as:    */
	/* Xn2 = Xn1    */
	/* Xn1 = Xn     */
	/* Yn2 = Yn1    */
	/* Yn1 = acc   */
	S->pState[1] = S->pState[0];
	S->pState[0] = (*pSrc);
	S->pState[3] = S->pState[2];
	S->pState[2] = (*pDst);

}

/**
 * @brief S Curve profile generation
 *
 * @param cmd	command
 * @param ref	reference
 * @param vel	velocity
 * @param acc	acceleration
 */
void Motor_Theta_ref_S_Curve(float32_t *cmd, float32_t *ref, float32_t *vel,
		float32_t *acc)
{
//	static float32_t acc=0;
	float32_t velo = 0;

	if (*cmd > *ref)
	{
		*acc = Motor1.Max_ThetaDotDot;
		velo = coerce(*vel + *acc*MID_LOOP_TIME, Motor1.Max_ThetaDot);
		if (((*cmd - *ref)
				< (0.5f * (velo) * (velo) / *acc + velo * MID_LOOP_TIME))
				&& (*vel > 0))
		{
			//Within reach need to decelerate
			*acc = -*acc;
		}

	}
	else
	{
		*acc = -Motor1.Max_ThetaDotDot;
		velo = coerce(*vel + *acc*MID_LOOP_TIME, Motor1.Max_ThetaDot);
		if (((*cmd - *ref)
				> (0.5f * (velo) * (velo) / *acc + velo * MID_LOOP_TIME))
				&& (*vel < 0))
		{
			//Within reach need to decelerate
			*acc = -*acc;
		}
	}

	*vel = coerce(*vel + *acc*MID_LOOP_TIME, Motor1.Max_ThetaDot);

	if ((abs(*vel) <= abs(*acc*MID_LOOP_TIME))
			&& (abs(*cmd-*ref) < abs(1.5f * (*acc) * MID_LOOP_TIME * MID_LOOP_TIME)))
		*ref = *cmd;	//Close enough
	else
		*ref = *ref + *vel * MID_LOOP_TIME;

}


/**
 * @brief trigger saving configuration
 *
 */
inline void Save_Config()
{
	Mode.Save_to_Flash = 1;
}

/**
 * @brief immediately trigger slow loop
 *
 */
inline void Trig_Slow_Loop()
{
	__HAL_TIM_SET_COUNTER(&HTIM_SLOW_LOOP,
			__HAL_TIM_GET_AUTORELOAD(&HTIM_SLOW_LOOP));
}

