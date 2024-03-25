/**
 * @file 	Versions.h
 * @brief 	Define firmware version for different motors
 *
 * @author 	Tym Zhu
 */
#ifndef VERSIONS_H
#define VERSIONS_H

/**
 * @name Motor defines uncomment the correct one
 * @{
 */
//#define MOTOR_KB01_R18  	/**< 0x01 KB01 (4006 with Ext AS5047) */
//#define MOTOR_KB01_R24  	/**< 0x02 KB01 (4006 with Ext AS5047) */
//#define MOTOR_U54			/**< 0x03 KB02 (U54) */
//#define MOTOR_U5412_R19  	/**< 0x04 KBMB00 (U5412 R19) */
//#define MOTOR_U5412_R20  	/**< 0x05 KBMB00 (U5412 R20) */
//#define MOTOR_U8			/**< 0x09 U8 (rev2) */
//#define MOTOR_U98			/**< 0x0A PB02&PB02P (U98) */
#define MOTOR_U180_HMND 	/**< 0x20 U180 ARTEMIS */
//#define MOTOR_U10P_HMND 	/**< 0x21 U10Plus ARTEMIS */
//#define MOTOR_4006_HMND 	/**< 0x22 4006 ARTEMIS */
//#define MOTOR_U5412_HMND	/**< 0x23 U5412/KBMB00 ARTEMIS */
//#define MOTOR_U5412_R20_HMND	/**< 0x24 U5412/KBMB00 R20 ARTEMIS */
//#define MOTOR_U54_HMND  	/**< 0x03 U54/KB02 ARTEMIS */
/**
 * @}
 */
/**
 * @name Motor defines for different variants
 * @{
 */
#ifdef MOTOR_U8		/*** U8 (rev2) ***/
	#define DRV_VERSION			3U		/**< Driver board version Gen2.0 */
	#define HW_VERSION			0x09	/**< motor hardware model */
	#define POLE_PAIR			21			//21 for U8
	#define REDUCTION_RATIO		10

	#define MOTOR_KM			0.98f//0.898f	// V/(rad/s) 
	#define MOTOR_R				0.173f		// Ohm

	#define DEFAULT_KP			0.456f
	#define DEFAULT_KI			0.027f

	/* Main Encoder */
	#define MHM_PV_MULTITURN	
	#define RTR_ENC_RES			65536
	#define RTR_ENC_RES_BIT		(16U)
	/* Ext Encoder */
	#define EXT_ENC_RES			1//No Ext Encoder

	#define AFC_HARMONIC2		12.0f

#endif

#ifdef MOTOR_U54	/*** U54 test (KB02) ***/
	#define DRV_VERSION			3U		/**< Driver board version Gen2.0 */
	#define HW_VERSION			0x03	/**< motor hardware model */
	#define POLE_PAIR			20
	#define REDUCTION_RATIO		9

	#define DEFAULT_KP			0.277f
	#define DEFAULT_KI			0.061f

	/* Main Encoder */
	#define MHM_NO_EEPROM
	static const uint8_t MHM_Regs[]=
						  {	0x11, 0x07, 0x00, 0x28, 0x80, 0x00, 0x00, 0x00, 0xC0, 0x80, 
							0x00, 0x00, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8C};
	#define RTR_ENC_RES			65536
	#define RTR_ENC_RES_BIT		(16U)
	/* Ext Encoder */
	#define EXT_MA_ENCODER
	#define EXT_MA_BCT			190
	#define EXT_ENC_RES			65536

	#define AFC_HARMONIC2		2.0f

#endif

#if defined (MOTOR_U5412_R19) || defined (MOTOR_U5412_R20) 	/*** U5412 R19&20 (KBMB00) ***/
	#define DRV_VERSION			3U			/**< Driver board version Gen2.0 */
	#ifdef  MOTOR_U5412_R19
		#define HW_VERSION			0x04	/**< motor hardware model */
		#define REDUCTION_RATIO		19.191666666666667f
	#else //MOTOR_U5412_R20
		#define HW_VERSION			0x05	/**< motor hardware model */
		#define REDUCTION_RATIO		20
	#endif

	#define POLE_PAIR			20

	#define DEFAULT_KP			0.358f
	#define DEFAULT_KI			0.077f

	/* Main Encoder */
	#define MHM_NO_EEPROM
	static const uint8_t MHM_Regs[]=
						  {	0x11, 0x07, 0x00, 0x28, 0x80, 0x00, 0x00, 0x00, 0xC0, 0x80, 
							0x00, 0x00, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8C};
	#define RTR_ENC_RES			65536
	#define RTR_ENC_RES_BIT		(16U)
	/* Ext Encoder */
	#define INIT_USE_EXT_ENC	//use external encoder value for init
	
	#define EXT_MA_ENCODER
	#define EXT_MA_BCT			183
	#define EXT_ENC_RES			65536

	#define AFC_HARMONIC2		2.0f

#endif
	
#ifdef MOTOR_U98	/*** U98 test (PB02) ***/
	#define DRV_VERSION			3U		/**< Driver board version Gen2.0 */
	#define HW_VERSION			0x0A	/**< motor hardware model */
	#define POLE_PAIR			40
	#define REDUCTION_RATIO		6

	#define DEFAULT_KP			0.099f
	#define DEFAULT_KI			0.039f

	/* Main Encoder */
//	#define MHM_NO_EEPROM
//	static const uint8_t MHM_Regs[]=
//						  {	0x11, 0x07, 0x00, 0x28, 0x80, 0x00, 0x00, 0x00, 0xC0, 0x80, 
//							0x00, 0x00, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8C};
	#define MHM_PV_MULTITURN	
	#define RTR_ENC_RES			65536
	#define RTR_ENC_RES_BIT		(16U)
	/* Ext Encoder */
	#define NO_ABS_ENCODER
	#define EXT_ENC_RES			1//No Ext Encoder

	#define AFC_HARMONIC2		2.0f

#endif

#if defined (MOTOR_KB01_R18) ||	defined (MOTOR_KB01_R24) /*** 4006 with Ext AS5047 (KB01) ***/
	#define DRV_VERSION			3U			/**< Driver board version Gen2.0 */
	#ifdef  MOTOR_KB01_R18
		#define HW_VERSION			0x01	/**< motor hardware model */
		#define REDUCTION_RATIO		18
	#else //MOTOR_KB01_R24
		#define HW_VERSION			0x02	/**< motor hardware model */
		#define REDUCTION_RATIO		24
	#endif
	
	#define DEFAULT_KP			0.374f
	#define DEFAULT_KI			0.030f

	#define POLE_PAIR			12
	/* Main Encoder */
	#define MHM_NO_EEPROM
	static const uint8_t MHM_Regs[]=
						  {	0x10, 0x47, 0x00, 0x28, 0x80, 0x00, 0x00, 0x00, 0xC0, 0x80, 
						  	0x00, 0x00, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8C};
	#define RTR_ENC_RES			65536
	#define RTR_ENC_RES_BIT		(16U)
	/* Ext Encoder */
	#define EXT_AS_ENCODER
	#define EXT_ENC_RES			16384

	#define AFC_HARMONIC2		6.0f

#endif

#ifdef MOTOR_U180_HMND	/*** U180_HMND ARTEMIS ***/
	#define DRV_VERSION			4U		/**< Gen2 high current for ARTEMIS */
	#define HW_VERSION			0x20	/**< motor hardware model */
	#define POLE_PAIR			40
	#define REDUCTION_RATIO		5.93103448275862f
	
	#define DEFAULT_KP			0.293f
	#define DEFAULT_KI			0.017f

	/* Main Encoder */
	#define MU_PVL_MULTITURN
	#define RTR_ENC_RES			524288
	#define RTR_ENC_RES_BIT		(19U)
	/* Ext Encoder */
	#define EXT_ENC_RES			1//No Ext Encoder

	#define AFC_HARMONIC2		6.0f

#endif

#ifdef MOTOR_U10P_HMND	/*** U10Plus ARTEMIS ***/
	#define DRV_VERSION			3U		/**< Driver board version Gen2.0 */
	#define HW_VERSION			0x21	/**< motor hardware model */
	#define POLE_PAIR			20
	#define REDUCTION_RATIO		14.46153846153846f

	#define DEFAULT_KP			0.353f
	#define DEFAULT_KI			0.016f

	/* Main Encoder */
	#define MHM_PV_MULTITURN	
	#define RTR_ENC_RES			65536
	#define RTR_ENC_RES_BIT		(16U)
	/* Ext Encoder */
	#define EXT_ENC_RES			1//No Ext Encoder

	#define AFC_HARMONIC2		2.0f

#endif

#ifdef MOTOR_4006_HMND	/*** 4006 ARTEMIS ***/
	#define DRV_VERSION			3U		/**< Driver board version Gen2.0 */
	#define HW_VERSION			0x22	/**< motor hardware model */
	#define POLE_PAIR			12
	#define REDUCTION_RATIO		50.28099173553719f

	#define DEFAULT_KP			0.374f
	#define DEFAULT_KI			0.030f

	/* Main Encoder */
	#define MHM_PV_MULTITURN	
	#define RTR_ENC_RES			65536
	#define RTR_ENC_RES_BIT		(16U)
	/* Ext Encoder */
	#define EXT_ENC_RES			1//No Ext Encoder

	#define AFC_HARMONIC2		6.0f

#endif

#ifdef MOTOR_U5412_HMND	/*** U5412/KBMB00 ARTEMIS ***/
	#define DRV_VERSION			3U		/**< Driver board version Gen2.0 */
	#define HW_VERSION			0x23	/**< motor hardware model */
	#define POLE_PAIR			20
	#define REDUCTION_RATIO		19.19166666666667f

	#define DEFAULT_KP			0.358f
	#define DEFAULT_KI			0.077f

	/* Main Encoder */
	#define MHM_PV_MULTITURN	
	#define RTR_ENC_RES			65536
	#define RTR_ENC_RES_BIT		(16U)
	/* Ext Encoder */
	#define EXT_ENC_RES			1//No Ext Encoder

	#define AFC_HARMONIC2		2.0f

#endif
#ifdef MOTOR_U5412_R20_HMND	/*** U5412/KBMB00 R20 ARTEMIS ***/
	#define DRV_VERSION			3U		/**< Driver board version Gen2.0 */
	#define HW_VERSION			0x24	/**< motor hardware model */
	#define POLE_PAIR			20
	#define REDUCTION_RATIO		20.0f

	#define DEFAULT_KP			0.358f
	#define DEFAULT_KI			0.077f

	/* Main Encoder */
	#define MHM_PV_MULTITURN
	#define RTR_ENC_RES			65536
	#define RTR_ENC_RES_BIT		(16U)
	/* Ext Encoder */
	#define EXT_ENC_RES			1//No Ext Encoder

	#define AFC_HARMONIC2		2.0f

#endif
	
#ifdef MOTOR_U54_HMND	/*** U54/KB02 ARTEMIS ***/
	#define DRV_VERSION			3U		/**< Driver board version Gen2.0 */
	#define HW_VERSION			0x03	/**< motor hardware model */
	#define POLE_PAIR			20
	#define REDUCTION_RATIO		9

	#define DEFAULT_KP			0.277f
	#define DEFAULT_KI			0.061f

	/* Main Encoder */
	#define MHM_PV_MULTITURN	
	#define RTR_ENC_RES			65536
	#define RTR_ENC_RES_BIT		(16U)
	/* Ext Encoder */
	#define EXT_ENC_RES			1//No Ext Encoder

	#define AFC_HARMONIC2		2.0f

#endif
/**
 * @}
 */
#endif
