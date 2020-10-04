/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "common.h"
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
void my_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
/* USER CODE BEGIN EV */
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];
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
  * @brief This function handles Prefetch fault, memory access fault.
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

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */
//	my_FDCAN_RxFifo0Callback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void my_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  //int loop = 1;
	int FDCANErrCode = 0;

	//uint32_t aTickstart = 0;    /* 10 */
	//uint32_t aTotalTick = 0;
//	int8_t err = 0;

	printf("my_FDCAN_RxFifo0Callback() entered IRQ \n");

	FDCANErrCode = HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL );
		if(FDCANErrCode != HAL_OK)
		{
			printf("HAL_FDCAN_De-ActivateNotification() %d\n", (int)FDCANErrCode);
			Error_Handler();
		}

		printf("HAL_FDCAN_De-ActivateNotification() OK\n");


#if 0
	  /* Get tick */
	  aTickstart = HAL_GetTick();

	  printf("HAL_FDCAN_GetRxFifoFillLevel()\n");
	  /* Check one message is received in Rx FIFO 0 */
	  if (err == 0) {
		  while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0)
		  {
			  aTotalTick = HAL_GetTick() - aTickstart;
			  err = 0;
			  if( aTotalTick > 100)
			  {
				  printf("HAL_FDCAN_GetRxFifoFillLevel() timeout\n");
				  //Error_Handler();
				  err = 1;
				  break;
			  }
			  printf("HAL_FDCAN_GetRxFifoFillLevel() status 0\n");
		  }
	  }
#endif
	  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)  {


  //while (loop++)	{

//	  while (Wait_Rx_Timeout(hfdcan, 100));

		  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
		  {
		  /* Retreive Rx messages from RX FIFO0 */
		  FDCANErrCode = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
		  if (FDCANErrCode != HAL_OK)
		  {
			  /* Reception Error */
			  printf("HAL_FDCAN_GetRxMessage() error code = %d\n", FDCANErrCode);

			  Error_Handler();

		  }
		  printf("HAL_FDCAN_GetRxMessage() ok\n");

		  printf("RxHeader.Identifier = 0x%x RxHeader.IdType = 0x%x RxHeader.DataLength = 0x%d\n", (unsigned int)RxHeader.Identifier, (unsigned int)RxHeader.IdType, (unsigned int)(RxHeader.DataLength >> 16));
		  /* Display the messages */
		  if ((RxHeader.Identifier == 0x321) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_1))
		  {
			  //State=RxData[0];
			  //printf("RxHd ID = 0x321 and Rxdata =  0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", RxData[0], RxData[1], RxData[2], RxData[3],RxData[4], RxData[5],RxData[6], RxData[7]);
			  printf("RdDataLength %d , RxData 0x%d\n", (int)(RxHeader.DataLength >> 16), (int)RxData[0]);
		  }
#if 1
		  FDCANErrCode = HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
		  if (FDCANErrCode != HAL_OK)
		  {
			  /* Notification Error */
			  printf("HAL_FDCAN_ActivateNotification() error code = %d\n", FDCANErrCode);
			  Error_Handler();
		  }
#else
		  FDCAN_Enable_RX_InterrptL0 (hfdcan);
#endif

		  printf("HAL_FDCAN_ActivateNotification() Ok\n");
//		  loop = 0;

	  	  }
	  }

	  else
	  {
		  printf("IRQ - FiFo 0 - EMPTY\n");
	  }
//		  HAL_Delay (100);
//		  //loop_count++;
//		  if (loop > 10 ) loop = 1;
//	  }
// }
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
