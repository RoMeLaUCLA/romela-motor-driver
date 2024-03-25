/**
 * @file 	Config.h
 * @brief 	Configuration file for motor
 *
 * @author 	Tym Zhu
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONFIG_H
#define CONFIG_H

/* macro for quick math */
#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif
#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif
//to limit the max absolute amount
#ifndef coerce
#define coerce(data,limit) ((data) > 0 ? (min(data,limit)) : (max(data,-limit)))
#endif
#ifndef abs
#define abs(x) ((x)<0 ? (-(x)) : (x))
#endif
#ifndef deadband
#define deadband(data,limit) ((abs(data)) > limit ? ( (data) > 0 ? (data-limit):(data+limit) ) : (0))
#endif

/* Includes ------------------------------------------------------------------*/

//#include "main.h"


#include "PID_f32.h"
#include "Build.h"
#include "Versions.h"


/* Private define ------------------------------------------------------------*/

/**** Defines for driver PCB configuration ***/


#if (DRV_VERSION == 3U)

	/* I_A_RAW = (ADC_Value-Neutral) * ADC_GAININV */
	#define ADC_GAININV			3.814697266E-3f	/**< 5.0V/(0.001Ohm*20*65536)
	 	 	 	 	 	 	 	 	 	 	 	 	 2.5V/20/0.001Ohm = +-125A */

	#define DOUBLE_PWM_FREQ						/**< double pwm frequency to 44kHz */

	#define IDRIVE_CURRENT		0x01			/**< 0x01 30mA/60mA */
	#define VDS_OVERCURRENT		0x04			/**< 0x04 0.31V
	 	 	 	 	 	 	 	 	 	 	 	 	 2.1mOhm*1.8*100A = 0.38V */

#elif (DRV_VERSION == 4U)

	/* I_A_RAW = (ADC_Value-Neutral) * ADC_GAININV */
	#define ADC_GAININV			0.00762939453125f /**< 5.0V/(0.0005Ohm*20*65536)
	 	 	 	 	 	 	 	 	 	 	 	 	   2.5V/20/0.5mOhm = +-250A */

	#define IDRIVE_CURRENT		0x02			/**< 0x01 60mA/120mA */
	#define VDS_OVERCURRENT		0x04			/**< 0x04 0.31V
	 	 	 	 	 	 	 	 	 	 	 	 	 0.91mOhm*1.8*200A = 0.33V */
#else
	#error "DRV_VERSION define error!"
#endif
/**
 * @name PWM output
 * @{
 */
/** max pwm ratio for 3 phase pwm */
#define MAX_PWM_RATIO   	1.0f//0.94f//0.949f

#ifdef DOUBLE_PWM_FREQ
	/* PWM frequency doubled to 44kHz, controller still runs at 22kHz */
	#define PWM_CCR				2048	/**< 180MHz / (2048 + 1) /2 = 43924Hz */

#else

	#define PWM_CCR				4096	/**< 180MHz / (4096 + 1) /2 = 21973Hz */

#endif
//#define PWM_CCR				2048//4096
#define MAX_PWM_CCR_F		((float32_t)PWM_CCR)			/**< float32 PWM CCR */
#define HALF_PWM_CCR		(PWM_CCR/2)						/**< 1/2 PWM CCR */
#define SQRT3_PWM_CCR_F		(1.73205080757f*MAX_PWM_CCR_F)	/**< sqrt(3) PWM CCR */

/* max Vd Vq allowed (1.0) */
#define MAX_Vd 				0.5f		/**< maximum Vd */
#define MAX_Vq 				1.0f		/**< maximum Vq */
/**
 * @}
 */

/**
 * @name ADC
 * @{
 */

#define ADC_MID				0x8000		/**< ADC mid point */

/**
 * @}
 */

/**
 * @name Encoders
 * @{
 */
/* Encoder resolution */
//Moved to Versions.h
/** Convert Encoder count to degree */
#define K_THETA_E			((float32_t)(360.0f*POLE_PAIR/RTR_ENC_RES))	//POLE_PAIR*360/RTR_ENC_RES

/*** Multi-turn ***/

/** max number of turns at output shaft */
#define MAX_TURN			16
/** full resolution of rotor with max number of turns */
#define RTR_THETA_FULL_RES	((int64_t)(MAX_TURN*REDUCTION_RATIO*RTR_ENC_RES))
/** minimum rotor theta with number of turns */
#define RTR_THETA_MIN		((int64_t)(-MAX_TURN*REDUCTION_RATIO*RTR_ENC_RES/2))
/** float32 full resolution of rotor with max number of turns */
#define THETA_FULL_RES_F	((float32_t)(MAX_TURN*2.0f*PI))
/** float32 minimum rotor theta with number of turns */
#define THETA_MIN_F			((float32_t)(-MAX_TURN*PI))	//Theta >= THETA_MIN_F
/** float32 maximum rotor theta with number of turns */
#define THETA_MAX_F			((float32_t)(MAX_TURN*PI))	//Theta < THETA_MAX_F


/*** External Absolute Encoder ***/
//#define EXT_ABS_ENC

/**
 * @}
 */
/**
 * @name Loops
 * @{
 */
/* Calculate the ratio between two loop */
#define HIGH_LOOP_HZ		21973					/**< high freq loop frequency */
#define HIGH_LOOP_US		(1000000/HIGH_LOOP_HZ)	/**< high freq loop time in micro seconds */
#define MID_LOOP_HZ			4000					/**< mid freq loop frequency */
#define MID_LOOP_TIME		(1.0f/MID_LOOP_HZ)		/**< mid freq loop time in seconds */

/* Velocity constant */

/** convert theta dot to rad/s */
#define RtrThetaDot2RAD	(2.0f*PI/RTR_ENC_RES*HIGH_LOOP_HZ/REDUCTION_RATIO)	//diff/RTR_RES*2pi*HIGH_LOOP_HZ
/** convert theta to rad */
#define RtrTheta2RAD	(2.0f*PI/RTR_ENC_RES/REDUCTION_RATIO)
/** convert rad to theta in encoder res */
#define RAD2RtrTheta	(1.0f/RtrTheta2RAD)

/** convert external theta dot to rad/s */
#define ExtThetaDot2RAD	(2.0f*PI/EXT_ENC_RES*MID_LOOP_HZ)	//
/** convert external theta to rad */
#define ExtTheta2RAD	(2.0f*PI/EXT_ENC_RES)

/**
 * @}
 */

/**
 * @name Vbus sensing
 * @{
 */
#define K_Vbus_ADC		1.629920373E-2f	/**< Vbus conversion 3.3/2^12/(3.9/(3.9+75)) */

/* default min max bus voltage (Volt) (will be over-written by the stored value) */
#define MAX_VBUS		60.0f 			/**< default max Vbus */
#define MIN_VBUS		8.0f			/**< default min Vbus */

/**
 * @}
 */

/**
 * @name Misc.
 * @{
 */
//#define RTR_THETA_DOT_CUTOFF 4.0f
//#define EXT_THETA_DOT_CUTOFF 1.0f

#define THETA_DEADBAND			0.0f			/**< deadband for position control */

#define PACKET_TIMEOUT			80U 			/**< ticks for packect timeout at 4000Hz */

#define ESTOP_TOGGLE_DELAY		11U				/**< 0.5ms at 22kHz (ticks for estop toggle delay) */
#define ESTOP_LATCHING_DELAY	HIGH_LOOP_HZ	/**< 1Sec (ticks for estop latching delay) */
/**
 * @}
 */

/**
 * @name AFC
 * @{
 */
#define AFC_HARMONIC1	6.0f

#define AFC_K			0.01f
#define AFC_MAX			1.0f
#define AFC_DAMPING		0.999f

#define K_THETA_AFC		((float32_t)(360.0f/RTR_ENC_RES))
/**
 * @}
 */

/**
 * @name Firmware version
 * @{
 */

/** Firmware version
 *  00|5bit Year|4bit Month|5bit Day */
//#define FW_VERSION	 0x0449
#ifndef FW_VERSION
#define FW_VERSION	 ( (((BUILD_YEAR-20)&0x1F)<<9) + \
						((BUILD_MONTH & 0x0F)<<5) + \
						 (BUILD_DAY & 0x1F) )			
#endif

/**
 * @}
 */

/**
 * @name All Coefficients for IIRs
 * @{
 */
static const float32_t IIR_I_Coeffs[] =
{ 0.067455273889072f, 0.134910547778144f, 0.067455273889072f, 1.142980502539901f, -0.412801598096189f }; /**< 2200Hz */

static const float32_t IIR_ENC_Coeffs[] =
{ 1.68889758E-2f, 3.37779517E-2f, 1.68889758E-2f, 1.60020175E+0f, -6.67757656E-1f }; /**< 1000Hz/22000 */

static const float32_t IIR_Velo_Coeffs[] =
//{1.68889758E-2f,3.37779517E-2f,1.68889758E-2f,1.60020175E+0f,-6.67757656E-1f};//1000/22000
//{4.67669924E-3f,9.35339849E-3f,4.67669924E-3f,1.75856440E+0f,-7.77271198E-1f};//400/22000
{ 1.72948243E-3f, 3.45896486E-3f, 1.72948243E-3f, 1.87896597E+0f, -8.85883904E-1f }; /**< 300Hz/22000 */
//={3.19821869E-4f,6.39643739E-4f,3.19821869E-4f,1.93773217E+0f,-9.39011459E-1f};//100/22000
static const float32_t IIR_Velo_aggr_Coeffs[] =
{ 3.19821869E-4f, 6.39643739E-4f, 3.19821869E-4f, 1.93773217E+0f, -9.39011459E-1f }; /**< 100Hz/22000 */

static const float32_t IIR_Posi_Coeffs[] =
//{0.00006101f,0.00012201f,0.00006101f,1.97778648f,-0.97803051f};//55/22000
//{0.00024136f,0.00048272f,0.00024136f,1.95557824f,-0.95654368f};//110/22000
//{0.00094469f,0.00188938f,0.00094469f,1.91119707f,-0.91497583f};//220/22000
//{0.00362168f,0.00724336f,0.00362168f,1.82269493f,-0.83718165f};//440/22000
//{0.25227378f,0.50454756f,0.25227378f,0.16743814f,-0.17653325f};//5000/22000
{ 0.06745527f, 0.13491055f, 0.06745527f, 1.14298050f, -0.41280160f }; /**< 400Hz/4000 */

static const float32_t IIR_Velo_Ext_Coeffs[] =
//{1.0f,0.0f,0.0f,0.0f,0.0f};
{ 0.06745527f, 0.13491055f, 0.06745527f, 1.14298050f, -0.41280160f }; /**< 400Hz/4000 */
//{0.005542717210281f,0.011085434420561f,0.005542717210281f,1.778631777824585f,-0.800802646665708f};//100/4000

static const float32_t IIR_Vbus_Coeffs[] =
{ 6.10061788E-5f, 1.22012358E-4f, 6.10061788E-5f, 1.97778648E+0f, -9.78030508E-1f }; /**< 10Hz/4000 */

static const float32_t IIR_Posi_ref_Coeffs[] =
{ 9.81051457E-5f, 1.96210291E-4f, 9.81051457E-5f, 1.96559107E+0f, -9.65983488E-1f }; /**< 10Hz/4000 */

//static float32_t IIR_TempTC_Coeffs[] =
//{0.63894553f,1.27789105f,0.63894553f,-1.14298050f,-0.41280160f};//2Hz/5
/**
 * @}
 */

/**
 * @name All Type defines
 * @{
 */

/** AFC struct define */
typedef struct
{
	float32_t harmonic;
	float32_t k;
	float32_t max;
	float32_t sin, cos;
	float32_t sin_i, cos_i, r_i;
	float32_t out;
} AFC_TypeDef;

/** Motor Type Define */
typedef struct
{

	/* Rotor Physical Theta */

	int32_t   Rtr_ST, 			/**< rotor position, single turn */
			  Rtr_ST_Prev;		/**< rotor position from previous loop */
	float32_t Rtr_ST_f;			/**< float32 rotor position */
	int32_t   Rtr_ST_LP;		/**< low-pass filtered rotor position */

	int32_t Rtr_MT,				/**< signed rotor number of turn for multi-turn */
			Rtr_MT_Prev, 		/**< rotor multi-turn from previous loop */
			Rtr_MT_Offset;		/**< offset to multi-turn value */

	int32_t Rtr_Theta_Offset32;	/**< offset to rotor theta */

	int64_t   Rtr_Theta_64;		/**< int64 Rotor Theta with multi-turn */
	float32_t Rtr_Theta;		/**< float32 Rotor Theta unit: rad (at output shaft) */
	float32_t Rtr_Theta_LP;		/**< low-pass filtered rotor theta */


	float32_t Rtr_Theta_Dot;	/**< Rotor Theta Dot, unit: rad/s */
	float32_t Rtr_Theta_Dot_LP, /**< low-pass filtered Rotor Theta Dot */
		Rtr_Theta_Dot_LP_aggr;	/**< aggressively filtered Rotor Theta Dot */

//	float32_t Theta_LP,Theta_LP_Prev;	//not in use

	/* Electrical Theta */
	float32_t Rtr_Elec_Offset;	/**< offset for aligning rotor with electric angle */
	float32_t Rtr_Theta_Elec;	/**< electric angle for rotor */
	float32_t Sin_Theta_Elec, 	/**< Sine of electric theta */
			  Cos_Theta_Elec;	/**< Cosine of electric theta */

	/* External encoder */
	int32_t Ext_Theta_Raw, 			/**< External encoder raw 12bit */
			Ext_Theta_Raw_Prev;		/**< External encoder raw previous loop */
//	int32_t		Ext_Theta_int32;	//not in use

	float32_t Ext_Theta;			/**< Output shaft theta with multi-turn and ZeroOffset */
//	float32_t	Ext_Theta_LP;		//Ext_Theta with lowpass
//	int32_t		Ext_NumofTurn;		//output shaft encoder turns
	float32_t Ext_Theta_ZeroOffset;	/**< Align Output shaft to zero */

	float32_t Ext_Theta_Dot;		/**< Output shaft theta dot */
	float32_t Ext_Theta_Dot_LP;		/**< low-pass filtered Output Shaft Theta Dot */

	/* RGB Encoder */
	uint16_t RGB_Theta_Raw;			/**< RGB Theta Raw */
	float32_t RGB_Theta;			/**< float32 RGB Theta */

	/* Manual theta for calibration */
	int32_t Theta_manual;			/**< Manual theta for calibration */

	/* ADC_Neutral_Point */
	int32_t ADC_A_Zero, 		/**< ADC neutral point for Ia */
			ADC_B_Zero;			/**< ADC neutral point for Ib */

	/* I_AB_RAW -> I_alphabeta -> I_dq_RAW */
	uint16_t ADC_IAB[2];		/**< Buffer for getting Ia and Ib */
	float32_t I_A_RAW,			/**< Ia raw data */
			  I_B_RAW;			/**< Ib raw data */
	float32_t I_alpha, 			/**< Ialpha raw data */
			  I_beta; 			/**< Ibeta raw data */
	float32_t I_d_RAW, 			/**< Id raw data */
			  I_q_RAW;			/**< Iq raw data */
	/* Id Iq */
	float32_t I_d,				/**< low-pass filtered Id */
			  I_q;				/**< low-pass filtered Iq */

	/* Vd Vq Valpha Vbeta */
	float32_t V_d,				/**< Vd */
			  V_q;				/**< Vq */
	float32_t V_alpha, 			/**< Valpha */
			  V_beta; 			/**< Vbeta */
	float32_t V_r;				/**< Vr, Abs voltage */

	/*svpwm*/
	int32_t X, Y, Z;			/**< for svpwm */
	int32_t TA, TB, TC;
	uint8_t Sector;

	/* Torque compensation */
	float32_t I_Torque_Comp; 	/**< Torque compensation value for given rtr theta */

	/*controller reference points*/
	float32_t I_q_Ref, 			/**< Iq reference */
			  I_q_Ref_AfterComp;/**< Iq reference after compensation */
	float32_t I_d_Ref; 			/**< Iq reference */
	float32_t Theta_Ref;		/**< theta reference */
	float32_t ThetaDot_Ref;		/**< theta dot reference */

	/*commands*/
	float32_t I_d_cmd, 			/**< Id command */
			  I_q_cmd, 			/**< Iq command */
			  ThetaDot_cmd, 	/**< theta dot command */
			  Theta_cmd;		/**< theta command */
	float32_t V_d_cmd, 			/**< Vd command */
			  V_q_cmd;			/**< Vq command */
	float32_t ThetaDotDot_traj, /**< theta dot dot acceleration for trajectory generation */
			  ThetaDot_traj;	/**< theta dot velocity for trajectory generation */

	/*Limits*/
	float32_t Max_Iq; 			/**< Iq limit */
	float32_t Max_Iq_Thermal;	/**< Iq limit with temperature limit */

	float32_t Max_ThetaDot, 	/**< Theta dot velocity limit */
			  Max_ThetaDotDot;	/**< Theta dot dot acceleration limit */

	float32_t Max_Theta, 		/**< Theta maximum limit */
			  Min_Theta;		/**< Theta minimum limit */

	/*PIDs*/
	arm_pid_instance_f32_L PID_Id;	/**< PID instance for Id */
	arm_pid_instance_f32_L PID_Iq;	/**< PID instance for Iq */
	arm_pid_instance_f32_L PID_Velo;/**< PID instance for Velocity */
	arm_pid_instance_f32_L PID_Posi;/**< PID instance for Position */
	arm_pid_instance_f32_L PID_DF;	/**< PID instance for DirectForce */

	/*IIRs*/
	arm_biquad_casd_df1_inst_f32 IIR_Id;		/**< IIR instance for Id */
	arm_biquad_casd_df1_inst_f32 IIR_Iq;		/**< IIR instance for Iq */
	arm_biquad_casd_df1_inst_f32 IIR_ENC;		/**< IIR instance for rotor */
	arm_biquad_casd_df1_inst_f32 IIR_Velo;		/**< IIR instance for velocity */
	arm_biquad_casd_df1_inst_f32 IIR_Velo_aggr;	/**< IIR instance for velocity aggressive */
	arm_biquad_casd_df1_inst_f32 IIR_Ext_Velo;	/**< IIR instance for external encoder velocity */
	arm_biquad_casd_df1_inst_f32 IIR_Ext_Posi;	/**< IIR instance for external encoder position */

	/* Points to the array of state coefficients.  The array is of length 4*numStages. */
	float32_t IIR_Id_State[4];					/**< IIR states for Id */
	float32_t IIR_Iq_State[4];					/**< IIR states for Iq */
	float32_t IIR_ENC_State[4];					/**< IIR states for rotor */
	float32_t IIR_Velo_State[4];				/**< IIR states for velocity */
	float32_t IIR_Velo_aggr_State[4];			/**< IIR states for velocity aggressive */
	float32_t IIR_Ext_Velo_State[4];			/**< IIR states for external encoder velocity */
	float32_t IIR_Ext_Posi_State[4];			/**< IIR states for external encoder position */

	/* AFC */
	AFC_TypeDef afc_Iq_1;		/**< AFC for Iq 1 */
	AFC_TypeDef afc_Iq_2;		/**< AFC for Iq 2 */
	AFC_TypeDef afc_Id_1;		/**< AFC for Id 1 */

} Motor_TypeDef;

/**
 * @brief motor mode structure
 *
 */
typedef struct
{
	uint8_t Ctrl_Mode;		/**< control mode */
							/**< * 0:Torque Mode
							 *	 * 1:Velocity Mode
							 *	 * 2:Position Mode
							 *	 * 3:Direct Force PID */

	uint8_t Calibrating;	/**< calibration mode */
	uint8_t Manual_E_Theta;	/**< Manual Electric theta for calibration mode */
	uint8_t Manual_V;		/**< Manual V for calibration mode */
	uint8_t Debug_Log;		/**< control mode */
							/**< * 0: No output
							 *	 * 1: Every ADC Loop
							 *	 * 2: Every Position Loop */

	uint8_t Torque_Comp;	/**< 1: torque compensation on */
//	uint8_t AFC;			/**< 1: AFC on? */
	uint8_t Error;			/**< Error byte transmitted back with RS485 */

	uint8_t Save_to_Flash;	/**< ready to save to flash */

	float32_t Vbus_RAW;		/**< Vbus raw data */
	float32_t Vbus_LowPass; /**< filtered Vbus */

	uint32_t Watchdog_CT; 	/**< counter for watchdog timer */

} Mode_TypeDef;

/** buffer type define */
typedef struct
{
	uint8_t Pt;
	uint8_t Done;
	uint8_t Buffer[256];
} Buffer_TypeDef;

/** packet progress status enum */
typedef enum
{
	IDLE = 0, HEADER0, HEADER1, ID, LEN, PAYLOAD
} Packet_Stat_TypeDef;

/** communication packet type define */
typedef struct
{
	Packet_Stat_TypeDef Packet_Stat;
	uint16_t Packet_ID;
	uint16_t Packet_Size;
	uint16_t Packet_NonIdle_CT;
	uint8_t CheckSum;
	Buffer_TypeDef Buff;

	uint8_t Bulk_Wait;		//Waiting on bulk
	uint8_t Bulk_Follow_ID;	//Last motor ID to follow
	uint16_t Bulk_NonIdle_CT;

} Packet_TypeDef;


#define MOTOR_CONFIG_SIZE  30	/**< motor config size */
/**
 * @brief motor config registers define
 *
 */
typedef struct
{
	//uint32_t	Hardware_REV;
	//uint32_t	Firmware_Ver;
	uint32_t ID;				/**< motor ID */
	uint32_t Mode;				/**< motor controller mode */
	uint32_t Baudrate;			/**< communication baudrate */
	float32_t Homing_Offset;	/**< homing offset */

	float32_t P_Id;				/**< P gain for Id */
	float32_t I_Id;				/**< I gain for Id */
	float32_t D_Id;				/**< D gain for Id */
	float32_t P_Iq;				/**< P gain for Iq */
	float32_t I_Iq;				/**< I gain for Iq */
	float32_t D_Iq;				/**< D gain for Iq */
	float32_t P_Velo;			/**< P gain for Velocity */
	float32_t I_Velo;			/**< I gain for Velocity */
	float32_t D_Velo;			/**< D gain for Velocity */
	float32_t P_Posi;			/**< P gain for Position */
	float32_t I_Posi;			/**< I gain for Position */
	float32_t D_Posi;			/**< D gain for Position */
	float32_t P_DirectF;		/**< P gain for DirectForce */
	float32_t I_DirectF;		/**< I gain for DirectForce */
	float32_t D_DirectF;		/**< D gain for DirectForce */

	float32_t Acc_Max;			/**< Acceleration limit */ //Id_Max;
	float32_t I_Max;			/**< Current limit */ //Iq_Max;
	float32_t Velo_Max;			/**< Velocity limit */
	float32_t Posi_Min;			/**< Position minimum limit */
	float32_t Posi_Max;			/**< Position maximum limit */
	float32_t Vbus_Min;			/**< input voltage minimum limit */
	float32_t Vbus_Max;			/**< input voltage maximum limit */
	uint32_t Watchdog_Timeout;	/**< watchdog timeout */
	float32_t Temp_Min;			/**< temperature lower threshold for limiting current */
	float32_t Temp_Max;			/**< temperature upper limit where current limits to 0 */

	int32_t Rtr_MT_Offset;		/**< hidden register for saving multi-turn offset */

} Motor_Config_TypeDef;

#define MOTOR_STAT_SIZE  16		/**< motor stat size */
/**
 * @brief motor stats registers define
 *
 */
typedef struct
{
	uint32_t Enable;			/**< Torque enable */
	uint32_t Homing_Done;		/**< not used? can be changed to motor status */

	float32_t Goal_Id;			/**< goal Id */
	float32_t Goal_Iq;			/**< goal Iq */
	float32_t Goal_Velo;		/**< goal Velocity */
	float32_t Goal_Posi;		/**< goal Position */
	float32_t Present_Id;		/**< present Id */
	float32_t Present_Iq;		/**< present Iq */
	float32_t Present_Velo;		/**< present Velocity */
	float32_t Present_Posi;		/**< present Position */

	float32_t Vbus;				/**< input voltage */
	float32_t Temp_Winding;		/**< winding temperature */
	float32_t Temp_MOSFET;		/**< MOSFET temperature */
	float32_t Temp_IC;			/**< IC temperature */

	uint32_t Error;				/**< not used? Plan to use it for detailed error message */
	uint32_t Iq_Temps;			/**< Combined Iq and temps */ // Iq*100(int16)|Twinding(uint8)|Tmosfet(uint8)

} Motor_Stat_TypeDef;



#define CAL_SIZE			2087	/**< Calibration data size */
									/* Sector 2 0x0800 8000 - 0x0800 BFFF
									 * 16Kbytes (4096Word) */

#define CAL_TORQUE_RES_BIT	(13U)	/**< feedforward torque 13bit */
#define CAL_ENC_UVW			((uint32_t)0x01)	/**< terminal lead order UVW? */
#define CAL_ENC_N			8		/**< Num of harmonics */

/**
 * @brief Calibration data
 *
 */
typedef struct
{

	int32_t ADC_A_Offset;				/**< ADC offset for Ia */
	int32_t ADC_B_Offset;				/**< ADC offset for Ib */

	uint32_t Encoder_Config;			/**< encoder direction configuration */
	float32_t Elec_Offset;				/**< electric angle offset */
	float32_t Theta_Zero_Offset;		/**< theta offset */

	float32_t Torque_Amp;				/**< feedforward torque amplitude */
	int8_t Torque_Cal[(1 << CAL_TORQUE_RES_BIT)];	/**< feedforward torque data */

	float32_t Motor_Km;					/**< not used? motor Km */

	float32_t Friction_Comp_Slope;		/**< friction compensation slope */
	float32_t Friction_Comp_Velocity;	/**< friction compensation starting velocity */
	float32_t Friction_Comp_Amp;		/**< friction compensation amplitude */

	float32_t Enc_B;					/**< encoder harmonics B */
	float32_t Enc_A[CAL_ENC_N];			/**< encoder harmonics A */
	float32_t Enc_FQ[CAL_ENC_N];		/**< encoder harmonics FQ */
	float32_t Enc_PH[CAL_ENC_N];		/**< encoder harmonics PH */
	float32_t Enc_D;					/**< encoder harmonics D */

	float32_t Enc_Ext_FQ;				/**< external encoder harmonics frequency */
	float32_t Enc_Ext_A;				/**< external encoder harmonics amplitude */
	float32_t Enc_Ext_PH;				/**< external encoder harmonics phase */

} Cal_TypeDef;

/**
 * @}
 */


extern Motor_TypeDef Motor1;
//extern Debug_TypeDef Debug;	//not used anymore
extern Mode_TypeDef Mode;

extern Motor_Config_TypeDef Config_Active;
extern Motor_Config_TypeDef Config_Buff;
extern Motor_Config_TypeDef *pConfig_Flash;

extern Motor_Stat_TypeDef Stat_Active;
extern Motor_Stat_TypeDef Stat_Buff;

extern Cal_TypeDef *pCal_Flash;


//#include "IC_MHM_PV.h"
//#include "IC_MU_PVL.h"
//#include "DRV8320.h"
//#include "TMP112.h"
//#include "MAX31855.h"
//#include "ADS8353.h"
//#include "AS5311.h"
//#include "MPS_MA.h"
//#include "Flash.h"

//#include "Control_LL.h"
//#include "Control.h"
//#include "Data_Comm.h"
//#include "Error.h"
//#include "Multiturn_Enc.h"

#endif /* CONFIG_H */
