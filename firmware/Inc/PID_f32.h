/**
 * @file 	PID_f32.h
 * @brief 	Header for pid_f32.c
 *
 * @author 	Tym Zhu
 */
#ifndef PID_F32_H
#define PID_F32_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"
//#include "main.h"

/**
 * @brief Instance structure for the floating-point PID Control.
 */
typedef struct
{
	float32_t state[3]; /**< The state array of length 3. */
	float32_t Kp; 		/**< The proportional gain. */
	float32_t Ki; 		/**< The integral gain. */
	float32_t Kd; 		/**< The derivative gain. */
	float32_t Limit; 	/**< output limit */
	float32_t I_Limit; 	/**< integral limit */
} arm_pid_instance_f32_L;


extern void PID_Init(arm_pid_instance_f32_L *S, float32_t Kp_i, float32_t Ki_i,
		float32_t Kd_i, float32_t Lmt);
extern void PID_Init(arm_pid_instance_f32_L *S, float32_t Kp_i, float32_t Ki_i,
		float32_t Kd_i, float32_t Lmt);
extern float32_t arm_pid_f32_L(arm_pid_instance_f32_L *S, float32_t in);
extern float32_t arm_pid_f32_L_Traj(arm_pid_instance_f32_L *S, float32_t in,
		float32_t v_error);
extern float32_t arm_pid_f32_L_Traj_FF(arm_pid_instance_f32_L *S, float32_t in,
		float32_t v_error, float32_t feedforward);
extern void arm_pid_reset_f32_L(arm_pid_instance_f32_L *S);

#endif

