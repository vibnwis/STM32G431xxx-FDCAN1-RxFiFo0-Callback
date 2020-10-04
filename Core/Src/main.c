/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "common.h"
#include "stm32g4xx_hal_fdcan.h"

#define 	A_NODE
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
#define TX_FAST_TIMEOUT 	10
#define RX_FAST_TIMEOUT 	10
#define FDCAN_INTR_RESET 	RESET
#define FDCAN_INTR_SET		SET

_Bool isLedOn = true;
uint8_t count = 0;
char string[10];
volatile int32_t ITM_RxBuffer;

 uint32_t exist_or_not=2;

 GPIO_PinState  state, prev_state;

 HAL_StatusTypeDef Hal_Error_Code;

 uint32_t Tickstart;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

void FDCAN_COMM_config_a(void);
void FDCAN_TX_Config_a(void);
void FDCAN_RX_Config_a(void);
void FDCAN_TX_a(FDCAN_HandleTypeDef *hfdcan);
void FDCAN_RX_a(void);
void Wait_Tx_Timeout(FDCAN_HandleTypeDef *hfdcan, uint32_t set_timeout);
int8_t Wait_Rx_Timeout(FDCAN_HandleTypeDef *hfdcan, uint32_t set_timeout);
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint8_t ubKeyNumber = 0x0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
HAL_StatusTypeDef FDCANErrCode;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(LD2_LED_GPIO_Port, LD2_LED_Pin);
	  isLedOn = !isLedOn;
	  count++;
	  if (count > 255)
		  count = 0;
	  printf("Hello world count = %d\n", (int)count);
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  state=HAL_GPIO_ReadPin(BLUE_SW_GPIO_Port, BLUE_SW_Pin); //GPIO_PIN_SET
	  if ( prev_state ==GPIO_PIN_RESET && state == GPIO_PIN_SET)
	  {
	  	printf("Blue SW ON - Activate transmission through FDCAN\n");

	  }
	  prev_state=state;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */
	/* Bit time configuration:
		 ************************
	            Bit time parameter         |   Nominal    |   Data
	            ---------------------------|--------------|--------------
	            fdcan_ker_ck               | 80 MHz       | 80 MHz
	            Time_quantum (tq)          | 16ns      	  | 16 ns
	            Prescaler                  |  10          |  10
	            Synchronization_segment    |  1 tq        |  1 tq
	            Propagation_segment        | 19 tq        |  5 tq
	            Phase_segment_1            | 13 tq        |  13 tq
	            Phase_segment_2            | 2 tq         |  2 tq
	            Synchronization_Jump_width | 1 tq         |  1 tq

	            Bit_rate                   |  500kBit/s    |  500 kBit/s
		 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  Hal_Error_Code = HAL_FDCAN_Init(&hfdcan1);
    if (Hal_Error_Code != HAL_OK)
    {
    	printf("HAL_FDCAN_Init() %d\n", (int)Hal_Error_Code);
    	Error_Handler();
    }

    printf("HAL_FDCAN_Init() OK\n");



    FDCAN_COMM_config_a();

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_LED_GPIO_Port, LD2_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUE_SW_Pin */
  GPIO_InitStruct.Pin = BLUE_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_LED_Pin */
  GPIO_InitStruct.Pin = LD2_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* Interrupts from FIFO-0 and FDCAN-RX
 * Interrupt sources - (FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
 *
 */
void FDCAN_Setup_RX_InterruptL0(FDCAN_HandleTypeDef *hfdcan) {

	HAL_StatusTypeDef status;

	status = HAL_FDCAN_ConfigInterruptLines(hfdcan, FDCAN_IT_LIST_RX_FIFO0, FDCAN_INTERRUPT_LINE0);

	if (status!= HAL_OK) {
		printf("HAL_FDCAN_ConfigInterruptLines() %d\n", (int)status);
				Error_Handler();
	}

	printf("HAL_FDCAN_ConfigInterruptLines() Ok\n");


}

void FDCAN_Enable_RX_InterrptL0 (FDCAN_HandleTypeDef *hfdcan)
{

	FDCANErrCode = HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL, FDCAN_RX_FIFO0 | FDCAN_RX_FIFO1 );
	if(FDCANErrCode != HAL_OK)
	{
		printf("HAL_FDCAN_ActivateNotification() %d\n", (int)Hal_Error_Code);
		Error_Handler();
	}

	printf("HAL_FDCAN_ActivateNotification() OK\n");
}

void FDCAN_COMM_config_a(void)
{

	FDCAN_RX_Config_a();

	//Setup FDCAN RX interrupt
//	FDCAN_Setup_RX_InterruptL0(&hfdcan1);

#if 0
	// Register callback HAL_FDCAN_RegisterCallback
		FDCANErrCode = HAL_FDCAN_RegisterCallback(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, (void*)HAL_FDCAN_RxFifo0Callback);
		if(FDCANErrCode != HAL_OK)
		{
			printf("HAL_CAN_RegisterCallback() %d\n", (int)Hal_Error_Code);
			Error_Handler();
		}

		printf("HAL_CAN_RegisterCallback() OK\n");

#endif
	// Activate the FDCAN peripheral
	FDCANErrCode = HAL_FDCAN_Start(&hfdcan1);
	if(FDCANErrCode != HAL_OK)
	{
		printf("HAL_FDCAN_Start() %d\n", (int)Hal_Error_Code);
		Error_Handler();
	}

	printf("HAL_FDCAN_Start() OK\n");

	FDCAN_Enable_RX_InterrptL0 (&hfdcan1);

}

void FDCAN_RX_Config_a()
{


	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x321;   // Read back its own message
	sFilterConfig.FilterID2 = 0x7FF;   // Mask off the entire MASK 11-bit registry

	FDCANErrCode = HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
	if(FDCANErrCode != HAL_OK)
	{
		printf("HAL_FDCAN_ConfigFilter() %d\n", (int)FDCANErrCode);
		Error_Handler();
	}

	printf("HAL_FDCAN_ConfigFilter() OK\n");

#if 0   // after running this code, it caused the external loopback failed
	FDCANErrCode = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	if(FDCANErrCode != HAL_OK)
	{
		printf("HAL_FDCAN_ConfigGlobalFilter() %d\n", (int)FDCANErrCode);
		Error_Handler();
	}
	printf("HAL_FDCAN_ConfigGlobalFilter() OK\n");
#endif
	// Proceed to call FDCAN_config_a()

}


void FDCAN_TX_Config_a(void)
{

	/* Prepare Tx Header */
	TxHeader.Identifier = 0x321;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

}

void FDCAN_TX_a(FDCAN_HandleTypeDef *hfdcan)
{

	/* Set the data to be transmitted */
	TxData[0] = 0xA5;
	TxData[1] = 0xAD;
	TxData[2] = 0xDE;
	TxData[3] = 0xAD;
	TxData[4] = 0xBE;
	TxData[5] = 0xEF;
	TxData[6] = 0xFA;
	TxData[7] = 0xCE;

	/* Start the Transmission process */
	FDCANErrCode = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, &TxData[0]);
	 if (FDCANErrCode != HAL_OK)
	 {
		 printf("HAL_FDCAN_AddMessageToTxFifoQ() failed at error code = %d\n", FDCANErrCode);
		 /* Transmission request Error */
		 Error_Handler();
	 }

	 printf("HAL_FDCAN_AddMessageToTxFifoQ: %d\n\n", (int)count);

}

void Wait_Tx_Timeout(FDCAN_HandleTypeDef *hfdcan, uint32_t set_timeout) {

	uint32_t aTickstart = 0;    /* 10 */
	uint32_t aTotalTick = 0;

	  /* Get tick */
	  aTickstart = HAL_GetTick();

	  /* Check transmission occurred before timeout */
	  while(HAL_FDCAN_IsTxBufferMessagePending(hfdcan, FDCAN_TX_BUFFER0) != 0)
	  {
		  aTotalTick = HAL_GetTick() - aTickstart;
	    if( aTotalTick > set_timeout)
	    {
	      printf("HAL_FDCAN_IsTxBufferMessagePending() timeout\n");
	      Error_Handler();
	      break;
	    }
	  }
	  printf("HAL_FDCAN_IsTxBufferMessagePending() time take %d\n", (int)aTotalTick);
}

int8_t Wait_Rx_Timeout(FDCAN_HandleTypeDef *hfdcan, uint32_t set_timeout) {
	  uint32_t aTickstart = 0;    /* 10 */
	  uint32_t aTotalTick = 0;
	  int8_t err = 0;

	  /* Get tick */
	  aTickstart = HAL_GetTick();

	  /* Check one message is received in Rx FIFO 0 */
	  while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) == 0)
	  {
		  aTotalTick = HAL_GetTick() - aTickstart;
		  if( aTotalTick > set_timeout)
		  {
	  	      printf("HAL_FDCAN_GetRxFifoFillLevel() timeout\n");
	  	      //Error_Handler();
	  	      err = 1;
	  	      break;
		  }
	  }
	  printf("HAL_FDCAN_GetRxFifoFillLevel() time take %d\n", (int)aTotalTick);
	  return err;

}


// typedef  void (*pFDCAN_RxFifo0CallbackTypeDef)(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);     /*!< pointer to Rx Fifo 0 FDCAN callback function          */
/**
 *
  * HAL_FDCAN_RxFifo0Callback()
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signaled.
  *                     This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */


void  HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{


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
		  printf("HAL_FDCAN_GetRxMessage() -- meeee callback\n");

		  printf("RxHeader.Identifier = 0x%x RxHeader.IdType = 0x%x RxHeader.DataLength = 0x%d\n", (unsigned int)RxHeader.Identifier, (unsigned int)RxHeader.IdType, (unsigned int)(RxHeader.DataLength >> 16));
		  /* Display the messages */
		  if ((RxHeader.Identifier == 0x321) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_1))
		  {
			  //State=RxData[0];
			  //printf("RxHd ID = 0x321 and Rxdata =  0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", RxData[0], RxData[1], RxData[2], RxData[3],RxData[4], RxData[5],RxData[6], RxData[7]);
			  printf("RdDataLength %d , RxData 0x%x\n", (int)(RxHeader.DataLength >> 16), (unsigned int)RxData[0]);
		  }

		//  FDCAN_Enable_RX_InterrptL0 (hfdcan);

	  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
    printf("Error Handle - endless loop. STOP DEBUG Mode\n");
	while (1) {

	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
