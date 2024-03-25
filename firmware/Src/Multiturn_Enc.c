/**
 * @file 	Multiturn_Enc.c
 * @brief 	Multi-turn encoder related functions
 *
 * @author 	Tym Zhu
 */
#include "Multiturn_Enc.h"
//#include "Control_LL.h"
#include "Config.h"
#include "util.h"

#include <stdio.h>

Encoder_Cal_TypeDef Encoder_Cal;
MLT_MT_TypeDef MLT_MT;

/**
 * @brief 	Find MultiTurn value
 *			ST: single turn value
 *			Exp_Theta: expected theta
 *			tol: tolerance of finding match
 * @param ST ST
 * @param MT MT
 * @return   0 == OK
 */
uint8_t MLT_Find_MT(int32_t ST, int32_t MT)
{
	int32_t iMT;
	int32_t Best_iMT=0;
	float32_t Theta_Candidate;
	float32_t Theta_Candidate_delta;

	/* reset best candidate */
	MLT_MT.Best_Theta_Candidate_delta = 3.40282347e38;
	for (iMT = 0; iMT < MT_CANDIDATES; iMT++) /* Try all MT candidates */
	{
		Theta_Candidate = WrapRtrTheta64(
				(iMT + MT_CANDIDATE_MIN) * RTR_ENC_RES + ST
						+ Motor1.Rtr_Theta_Offset32) * RtrTheta2RAD;
		Theta_Candidate_delta = fabsf(Theta_Candidate - MLT_MT.Exp_Theta);
//		printf("%d:%.2f|", (iMT + MT_CANDIDATE_MIN), Theta_Candidate_delta);
		if (Theta_Candidate_delta < MLT_MT.Best_Theta_Candidate_delta)
		{
			/* New best */
			Best_iMT = iMT;
			MLT_MT.Best_Theta_Candidate_delta = Theta_Candidate_delta;
		}
	}

	printf("Best MT %ld: %.4f\n", (Best_iMT + MT_CANDIDATE_MIN),
			MLT_MT.Best_Theta_Candidate_delta);

	if (MLT_MT.Best_Theta_Candidate_delta < MLT_MT.Tolerance) /* Check for tolerance */
	{
		MLT_MT.Error = 0;
		MLT_MT.Best_MT = Best_iMT + MT_CANDIDATE_MIN;
		MLT_MT.MT_Offset = MLT_MT.Best_MT - MT;
		return MLT_MT.Error;
	}
	else
	{
		MLT_MT.Error = 1;
		return MLT_MT.Error;
	}
}

/**
 * @brief Find MT at initialization
 *
 * @param ST ST
 * @param MT MT
 * @param Ext_Theta external encoder theta
 * @return error
 */
uint8_t MLT_Init_MT(int32_t ST, int32_t MT, int32_t Ext_Theta)
{
	int32_t iMT, iX;
	int32_t Best_iMT=0;
	float32_t Rtr_Theta_Candidate, Rtr_Theta_Candidate_w_Offset,
			Ext_Theta_Candidate;
	float32_t Ext_Theta_Candidate_delta;

	/* reset best candidate */
	MLT_MT.Best_Theta_Candidate_delta = 3.40282347e38;
	for (iMT = 0; iMT < MT_CANDIDATES; iMT++) /* Try all MT candidates */
	{
		Rtr_Theta_Candidate_w_Offset = WrapRtrTheta64(
				(iMT + MT_CANDIDATE_MIN) * RTR_ENC_RES + ST
						+ Motor1.Rtr_Theta_Offset32) * RtrTheta2RAD;
		Rtr_Theta_Candidate = ((iMT + MT_CANDIDATE_MIN) * RTR_ENC_RES + ST)
				* RtrTheta2RAD;

		if ((Rtr_Theta_Candidate_w_Offset >= (-PI))
				&& (Rtr_Theta_Candidate_w_Offset < PI))
		{
			Ext_Theta_Candidate = Rtr_Theta_Candidate + pCal_Flash->Enc_B;
			for (iX = 0; iX < CAL_ENC_N; iX++)
			{
				Ext_Theta_Candidate += pCal_Flash->Enc_A[iX]
						* arm_sin_f32(
								pCal_Flash->Enc_FQ[iX] * Rtr_Theta_Candidate
										+ pCal_Flash->Enc_PH[iX]);
			}

			Ext_Theta_Candidate = fmodf(Ext_Theta_Candidate, 2.0f * PI);
			if (Ext_Theta_Candidate < 0.0f)
				Ext_Theta_Candidate += 2.0f * PI;

			Ext_Theta_Candidate_delta = fabsf(
					Ext_Theta_Candidate - ExtTheta2RAD * Ext_Theta);
//			printf("%d:%.2f|", (iMT + MT_CANDIDATE_MIN), Ext_Theta_Candidate_delta);

			if (Ext_Theta_Candidate_delta < MLT_MT.Best_Theta_Candidate_delta)
			{
				/* New best */
				Best_iMT = iMT;
				MLT_MT.Best_Theta_Candidate_delta = Ext_Theta_Candidate_delta;
			}
		}
	}

	printf("Best MT %ld:%.4f ", (Best_iMT + MT_CANDIDATE_MIN),
			MLT_MT.Best_Theta_Candidate_delta);

	if (MLT_MT.Best_Theta_Candidate_delta < INIT_MT_TOLERANCE) /* Check for tolerance */
	{
		MLT_MT.Error = 0;
		MLT_MT.Best_MT = Best_iMT + MT_CANDIDATE_MIN;
		MLT_MT.MT_Offset = MLT_MT.Best_MT - MT;
		return MLT_MT.Error;
	}
	else
	{
		MLT_MT.Error = 1;
		return MLT_MT.Error;
	}

}

