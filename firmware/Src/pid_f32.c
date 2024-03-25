/**
 * @file 	pid_f32.c
 * @brief 	PID controller float32
 *
 * @author 	Tym Zhu
 */
#include "pid_f32.h"


/**
 * @brief Initialize sequence for arm_pid_instance_f32_L
 *
 * @param S		instance of the floating-point PID Control structure
 * @param Kp_i	Kp
 * @param Ki_i	Ki
 * @param Kd_i	Kd
 * @param Lmt	Limit
 */
void PID_Init(arm_pid_instance_f32_L * S,
							  float32_t Kp_i, float32_t Ki_i, float32_t Kd_i, 
							  float32_t Lmt)
{
	S->Kp=Kp_i;
	S->Ki=Ki_i;
	S->Kd=Kd_i;
	S->Limit=Lmt;
	S->I_Limit=1.0f;
	arm_pid_reset_f32_L(S);
}


/**
 * @brief Process function for the floating-point PID Control.
 *
 * @param S		instance of the floating-point PID Control structure
 * @param in	input sample to process
 * @return		processed output sample
 */
float32_t arm_pid_f32_L(arm_pid_instance_f32_L *S, float32_t in)
{
	float32_t out;
	float32_t I_out = S->Ki * (S->state[1] + in);

	//Limit Ki
	if (I_out > (S->I_Limit * S->Limit))
	{
		out = S->I_Limit * S->Limit;
	}
	else if (I_out < -(S->I_Limit * S->Limit))
	{
		out = -(S->I_Limit * S->Limit);
	}
	else
	{
		S->state[1] += in;	//accumulate
		out = S->Ki * S->state[1];
	}

	out += (S->Kp * in + S->Kd * (in - S->state[0]));

	//Limit
	if (out > S->Limit)
		out = S->Limit;
	if (out < -(S->Limit))
		out = -(S->Limit);
	// Update state
	S->state[0] = in;
	//S->state[2] = out;

	// return to application
	return (out);

}

/**
 * @brief PID with velocity reference
 *
 * @param S			instance of the floating-point PID Control structure
 * @param in		input
 * @param v_error	velocity error
 * @return			output
 */
float32_t arm_pid_f32_L_Traj(arm_pid_instance_f32_L *S,
		float32_t in, float32_t v_error)
{
	float32_t out;
	float32_t I_out = S->Ki * (S->state[1] + in);

	//Limit Ki
	if (I_out > (S->I_Limit * S->Limit))
	{
		out = S->I_Limit * S->Limit;
	}
	else if (I_out < -(S->I_Limit * S->Limit))
	{
		out = -(S->I_Limit * S->Limit);
	}
	else
	{
		S->state[1] += in;	//accumulate
		out = S->Ki * S->state[1];
	}

	out += (S->Kp * in + S->Kd * (v_error));

	//Limit
	if (out > S->Limit)
		out = S->Limit;
	if (out < -(S->Limit))
		out = -(S->Limit);
	// Update state
	S->state[0] = in;
	//S->state[2] = out;

	// return to application
	return (out);

}

/**
 * @brief PID with velocity error and feedforward
 *
 * @param S				instance of the floating-point PID Control structure
 * @param in			input
 * @param v_error		velocity error
 * @param feedforward 	feedforward term
 * @return				output
 */
float32_t arm_pid_f32_L_Traj_FF(arm_pid_instance_f32_L *S,
		float32_t in, float32_t v_error, float32_t feedforward)
{
	float32_t out;
	float32_t I_out = S->Ki * (S->state[1] + in);

	//Limit Ki
	if (I_out > (S->I_Limit * S->Limit))
	{
		out = S->I_Limit * S->Limit;
	}
	else if (I_out < -(S->I_Limit * S->Limit))
	{
		out = -(S->I_Limit * S->Limit);
	}
	else
	{
		S->state[1] += in;	//accumulate
		out = S->Ki * S->state[1];
	}

	out += (S->Kp * in + S->Kd * (v_error) + feedforward);

	//Limit
	if (out > S->Limit)
		out = S->Limit;
	if (out < -(S->Limit))
		out = -(S->Limit);
	// Update state
	S->state[0] = in;
	//S->state[2] = out;

	// return to application
	return (out);

}


/**
 * @brief Reset function for the floating-point PID Control.
 *
 * @param S Instance pointer of PID control data structure.
 */
void arm_pid_reset_f32_L(arm_pid_instance_f32_L *S)
{
	/* Clear the state buffer.  The size will be always 3 samples */
	memset(S->state, 0, 3u * sizeof(float32_t));
}
