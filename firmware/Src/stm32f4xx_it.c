/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IC_MHM_PV.h"
#include "IC_MU_PVL.h"
//#include "DRV8320.h"
//#include "TMP112.h"
//#include "MAX31855.h"
#include "ADS8353.h"
//#include "AS5311.h"
//#include "MPS_MA.h"
//#include "Flash.h"
//
//#include "Control_LL.h"
#include "Control.h"
#include "Data_Comm.h"
#include "Error.h"
//#include "Multiturn_Enc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#include "Config.h"

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern USART_HandleTypeDef husart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	//YOLO Faster IRQ
	uwTick += uwTickFreq; 
	return;
	
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
	static uint8_t toggle=1;
	
	/*Start ADC: Capture compare 4 event */
	if((__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_CC4) != RESET)&&(__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC4) !=RESET))
	{
		__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);

#ifdef DOUBLE_PWM_FREQ

		toggle=!toggle;	//!!!!!!!!!!!
		if (toggle)
#endif
		{
			
		#if defined(MHM_NO_EEPROM) || defined(MHM_PV_MULTITURN)
			/* MHM state machine for reset ic-PV */
			switch (MHM.State)
			{
			case 0:				/* Normal Operation */
				MHM_Start_ReadPosi();
				break;
			case 1:				/* Set ic-pv Preset pin */
				MHM_Start_Write_Reg(MHM_ADDR_FIO, 0x08); 
				break;
			case 2:				/* Reset ic-pv Preset pin */
				MHM_Start_Write_Reg(MHM_ADDR_FIO, 0x00); 
				break;
			default:			/* Waiting do nothing */
				
				break;
			case MHM_PV_DELAY:	/* Reset MHM after the delay */
				MHM_Start_Write_Reg(MHM_ADDR_RESET, MHM_INSTR_RESET);
				break;
			case MHM_RESET_DELAY:/* Reset delay elapsed */
				MHM_Start_ReadPosi();
				break;
			}
			
		#elif defined(MU_PVL_MULTITURN)
			/* TODO: reset function */
			MU_Read_Posi(&Motor1.Rtr_MT, &Motor1.Rtr_ST);
		#else
			static_assert(0, "Main Encoder definition invalid!");
		#endif
			
			ADS8353_Start_GetData();
			
			
			ADC_Routine();
		}
		return;
	}
  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
    /*RS485*/
    uint8_t temp;
    
    if( ((USART1->SR & USART_SR_RXNE) != RESET) ) //&& ((USART1->CR1 & USART_CR1_RXNEIE) != RESET))
    {
        /*Get the received byte*/
        temp=(USART1->DR);
        
        switch (RS485_Packet.Packet_Stat)//Last packet stat
        {
        default:
        case IDLE:
            if ((temp==0xFF) ) RS485_Packet.Packet_Stat=HEADER0;
//            else
//            {
//                printf("RS485 error byte: 0X%2X\n",temp);
//            }
            break;
        case HEADER0:
            if (temp==0xFF) 
            {
                RS485_Packet.Packet_Stat=HEADER1;
                RS485_Packet.CheckSum=0;
            }
            else 			RS485_Packet.Packet_Stat=IDLE;
            break;
        case HEADER1:
            RS485_Packet.CheckSum+=RS485_Packet.Packet_ID=temp;
            //CheckSum+=temp;
            RS485_Packet.Packet_Stat=ID;
            break;
        case ID:
            RS485_Packet.CheckSum+=RS485_Packet.Packet_Size=temp;
            //CheckSum+=temp;
            RS485_Packet.Packet_Stat=LEN;
            break;	
        case LEN:
            RS485_Packet.Packet_Stat=PAYLOAD;
            RS485_Packet.Buff.Pt=0;
            /* NO break, Continue to PAYLOAD */
        case PAYLOAD:
            RS485_Packet.CheckSum+=RS485_Packet.Buff.Buffer[RS485_Packet.Buff.Pt]=temp;
            RS485_Packet.Buff.Pt++;
            //CheckSum+=temp;
            
	    if (RS485_Packet.Buff.Pt==RS485_Packet.Packet_Size)
		{
		  /* Done */
		  //RS485_Packet.Buff.Done=1; //not in use
		  if (RS485_Packet.CheckSum!=0xff)
		  {
			/*Failed checksum*/
			printf("RS485 Failed Checksum!\n");
			Error_Set(WARNING_COMM);
		  }
		  else
		  {
			/*Passed CheckSum*/
			if(RS485_Packet.Packet_ID==Config_Active.ID)
			{
			  /*Direct Addressing the motor*/
			  UART_RS485_Direct_Routine();
			  Mode.Watchdog_CT=0;
			}
			else if (RS485_Packet.Packet_ID==0xFE)
			{
			  /*Broadcasting address*/
			  UART_RS485_Bulk_Routine();
			  Mode.Watchdog_CT=0;
			}
			else if ((RS485_Packet.Bulk_Wait != 0) && 
					 (RS485_Packet.Packet_ID == RS485_Packet.Bulk_Follow_ID) &&
					   (RS485_Packet.Buff.Buffer[0]&0x80) )
			{
			  /*Send back the bulk response*/
			  UART_RS485_Bulk_Response();
			  RS485_Packet.Bulk_NonIdle_CT=0;
			}
		  }
		  
		  RS485_Packet.Packet_Stat=IDLE;
		  RS485_Packet.Packet_NonIdle_CT=0;
		  
		}
            break;	
        }
        
        return;
    }
    
    if(((USART1->SR & USART_SR_TC) != RESET) 
		&& ((USART1->CR1 & USART_CR1_TCIE) != RESET))		//moved from hal_usart
    {
        RS485_RXEN();
        
        /* Disable the UART Transmit Complete Interrupt */    
        CLEAR_BIT(USART1->CR1, USART_CR1_TCIE);
        /* Tx process is ended, restore huart->gState to Ready */
        huart1.gState = HAL_UART_STATE_READY;
        
        return;
    }
    
    
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
    
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_USART_IRQHandler(&husart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	
	Slow_Loop_Routine();
	
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	/* TIM Update event */
	if((__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET)&&(__HAL_TIM_GET_IT_SOURCE(&htim7, TIM_IT_UPDATE) !=RESET))
	{
		__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
		Mid_Ctrl_Routine();
		return;
	}
	
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
	uint8_t temp;
	
	if( ((USART6->SR & USART_SR_RXNE) != RESET) ) //&& ((USART1->CR1 & USART_CR1_RXNEIE) != RESET))
	{
		temp=(USART6->DR);	//get the byte
		
		switch (Debug_Packet.Packet_Stat)//Last packet stat
		{
		default:
		case IDLE:
			if (temp==0xFF) Debug_Packet.Packet_Stat=HEADER0;
			break;
		case HEADER0:
			if (temp==0xFF) Debug_Packet.Packet_Stat=HEADER1;
			else 			Debug_Packet.Packet_Stat=IDLE;
			break;
		case HEADER1:
			Debug_Packet.Packet_ID=temp;
			Debug_Packet.Packet_Stat=ID;
			break;
		case ID:
			Debug_Packet.Packet_Size=temp;
			Debug_Packet.Packet_Stat=LEN;
			break;	
		case LEN:
			Debug_Packet.Packet_Stat=PAYLOAD;
			Debug_Packet.Buff.Pt=0;
			/* NO break, Continue to PAYLOAD */
		case PAYLOAD:
			Debug_Packet.Buff.Buffer[Debug_Packet.Buff.Pt]=temp;
			Debug_Packet.Buff.Pt++;
			if (Debug_Packet.Buff.Pt==Debug_Packet.Packet_Size)
			{
				/* Done */
				Debug_Packet.Buff.Done=1;
				Debug_Packet.Packet_Stat=IDLE;
				Debug_Packet.Packet_NonIdle_CT=0;
			}
			break;	
		}
		
		return;
	}
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
