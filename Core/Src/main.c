/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DISPLAY_CAN_MESSAGE 0			// Turn this on/off if you want to print/ignore raw CAN Messages
#define DISPLAY_CAN_ERRORS 0			// Turn this on/off if you want to print/ignore CAN Errors

// Pedal Calibration Inversion Constants
#define  InvertAnalogOnePedal 0 // Swap the max and min pedal voltages on analog line one
#define  InvertAnalogTwoPedal 0 // Swap the max and min pedal voltages on analog line two

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* Definitions for PedalControl */
osThreadId_t PedalControlHandle;
const osThreadAttr_t PedalControl_attributes = {
  .name = "PedalControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataDisplay */
osThreadId_t DataDisplayHandle;
const osThreadAttr_t DataDisplay_attributes = {
  .name = "DataDisplay",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for APPSCalibration */
osThreadId_t APPSCalibrationHandle;
const osThreadAttr_t APPSCalibration_attributes = {
  .name = "APPSCalibration",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for CANWatchdog */
osThreadId_t CANWatchdogHandle;
const osThreadAttr_t CANWatchdog_attributes = {
  .name = "CANWatchdog",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

CAN_TxHeaderTypeDef TxHeader; // Header containing the information of the transmitted frame
CAN_RxHeaderTypeDef RxHeader; // Header containing the information of the received frame

uint8_t TxData[8] = {0}; // Buffer of the data to send (8 bytes data)
uint8_t WatchDogTxData[8] = {1}; // Random Buffer data to simulate messages for Watchdog
uint8_t RxData[8]; // Buffer of the received data (8 bytes data)
uint32_t TxMailbox; // The number of the email box that transmitted the Tx message

const uint8_t MC_NodeId = 0x04; // Node ID of the MC

// MC commands
const uint16_t CMD_SetRelativeCurrent = (0x500 + MC_NodeId); // Set relative AC current message ID
const uint16_t CMD_SetRelativeBrakeCurrent = (0x600 + MC_NodeId);  // Set relative AC brake current message ID
const uint16_t CMD_SetMaxAcCurrent = (0x800 + MC_NodeId); // Set max AC current limit message ID
const uint16_t CMD_SetMaxBrakeCurrent = (0x900 + MC_NodeId); // Set max AC brake current limit message ID
const uint16_t CMD_SetMaxDcCurrent = (0xA + MC_NodeId); // Set max DC current limit message ID
const uint16_t CMD_SetMaxDcBrakeCurrent = (0xB + MC_NodeId); // Set max DC brake current limit message ID


// Data from MC
const uint16_t RCV_MC_GeneralData = (0x1F + MC_NodeId);

// AMS Limits
uint16_t DischargeCurrentLimit = 30; // AMS discharge current limit
uint16_t ChargeCurrentLimit = 20; // AMS charge current limit

// Pedal Calibration Constants
float MaxPedalVoltage[2] = {3.3, 3.3}; // Max voltages on both analog lines for the pedal input
float MinPedalVoltage[2] = {0.5, 0.5}; // Min voltages on both analog lines for the pedal input
float CenterPedalVoltage[2] = {2.0, 2.0}; // Center voltages on both analog lines for the pedal input. This voltage corresponds position in which braking/throttle is swapped


float PedalDeadband = 0.15; // Percent deadband of pedal


uint32_t lastThrottleOrBrake = 0; // Gets the time at which a throttle or break CAN message is sent

float inputPedalVoltage = 0; // voltage that the APPS is outputting to the STM
char msg[20];
char CANBuffer[100]; // Buffer used to display CAN messages on terminal
uint16_t CalculatedValue; // The value to be sent over to the MC over CAN


///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////CAN Watchdog /////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

// Initializes variables to be used to check when the last respective message was sent
// uint32_t lastHeartBeatMessage = 0;
 uint32_t lastTemMessage = 0;
 uint32_t lastAmsMessage = 0;
 uint32_t lastMcMessage = 0;
 uint32_t lastChargerMessage = 0;

 // Initializes flags to indicate when a certain ECU hasn't sent a message in a second, timing out
// uint8_t heartBeatError = 0;
 uint8_t temError = 0;
 uint8_t amsError = 0;
 uint8_t mcError = 0;
 uint8_t chargerError = 0;

 // Creates int to determine if we are charging or not
 uint8_t isCharging = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
void ControlPedal(void *argument);
void DisplayData(void *argument);
void StartAPPSCalibration(void *argument);
void StartCANWatchdog(void *argument);

/* USER CODE BEGIN PFP */

// Prototype for CAN Transmission
//HAL_StatusTypeDef CAN_Send(uint32_t id, uint8_t *data, uint8_t length);

// Prototype to see if a certain message hasn't been seen in over a second
uint8_t lastMessageSent(uint32_t lastMessage);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

//  HAL_Delay(35000);				// Add a delay for startup to have no power to not fault anything

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  TxHeader.StdId = 0x123; // ID the STM is transmitting with

  // Describes how CAN frame should be transmitted

  TxHeader.RTR = CAN_RTR_DATA; // Remote Transmission Request (RTR) tells CAN controller we are sending data
  TxHeader.IDE = CAN_ID_STD; // Identifies if we are using extended(29-bit) or standard (11-bit) CAN; It is currently set to standard
  TxHeader.DLC = 8; // Data Length Code (DLC) -- The number of bytes in the data frame
  TxHeader.TransmitGlobalTime = DISABLE; // Disables internal timestamp when sending data
  TxData[0] = 0;
  TxData[7] = 0xFF;


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of PedalControl */
  PedalControlHandle = osThreadNew(ControlPedal, NULL, &PedalControl_attributes);

  /* creation of DataDisplay */
  DataDisplayHandle = osThreadNew(DisplayData, NULL, &DataDisplay_attributes);

  /* creation of APPSCalibration */
  APPSCalibrationHandle = osThreadNew(StartAPPSCalibration, NULL, &APPSCalibration_attributes);

  /* creation of CANWatchdog */
  CANWatchdogHandle = osThreadNew(StartCANWatchdog, NULL, &CANWatchdog_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

	CAN_FilterTypeDef filter; // Declares filter

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_LOOPBACK;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  // Sets up filter for CAN messages; Allows all messages to pass

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0; // The data will be received in FIO0
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) // checks to see if filter is configured correctly
    {
  	  // Filter configuration error
  	  Error_Handler();
    }
    // Starts CAN peripheral if filter config is good
    if (HAL_CAN_Start(&hcan) != HAL_OK) // Tries to start CAN and checks for error
    {
  	  Error_Handler();
    }


    // Activate CAN RX notifications on FIFO0
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
  	  Error_Handler();
    }

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PB4 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Callback function for data received on FIFO0
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		// Reception Error; error thrown if the receive fails
		Error_Handler();
	}

//    CAN_RxHeaderTypeDef RxHeader;
//    uint8_t RxData[8];


    // Determine standard vs extended CAN ID - NEW
	uint32_t rx_id; // Message ID of received data
	if (RxHeader.IDE == CAN_ID_STD) {
		rx_id = RxHeader.StdId; // Stores CAN ID of standard CAN message
	} else {
		rx_id = RxHeader.ExtId; // Stores CAN ID of extended CAN message
	}


//	printf("CAN Message: %08X\r\n	", rx_id);

	uint8_t CANBufferLengthUsed = 0;

	//snprintf returns the # of characters written
//		printf("CAN Message: %08X	", Rx_Id);

	CANBufferLengthUsed += snprintf(CANBuffer, sizeof(CANBuffer), "CAN Message: %08X	", rx_id); // Appends CAN ID to CANBuffer

	for (int i = 0; i < 8; i++)
	{
		// Prints received CAN data as 2 digit hex on new lines
		// Appends each new data on next empty CANBuffer spot
		CANBufferLengthUsed += snprintf(&CANBuffer[CANBufferLengthUsed], sizeof((CANBuffer) - CANBufferLengthUsed), " %02X", RxData[i]);
	}
	CANBufferLengthUsed += snprintf(&CANBuffer[CANBufferLengthUsed], sizeof((CANBuffer) - CANBufferLengthUsed), "\r\n");

//	    sprintf(msg, "Voltage: %.3f V\r\n", inputPedalVoltage);
//	    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY); // Need to be mindefule of using max delay


	// Sends CANBuffer over UART
	HAL_UART_Transmit(&huart2, (uint8_t *)CANBuffer, CANBufferLengthUsed, HAL_MAX_DELAY); // CANBufferLengthUsed used instead of sizeof(CANBuffer) bc length of the CANBuffer is already known


    // Display the CAN message to the serial monitor
//    if(DISPLAY_CAN_MESSAGE){
//
//    	// Change Print syntax based on standard vs. extended
//    	if (RxHeader.IDE == CAN_ID_STD){
//    		printf("RX STD ID: 0x%03lX | Data:", rx_id);
//    	}
//		else{
//			printf("RX EXT ID: 0x%08lX | Data:", rx_id);
//		}
//
//		for (int i = 0; i < RxHeader.DLC; i++)
//			printf(" %02X", RxData[i]);
//
//		printf("\r\n");
//    }

    // Based on the CAN ID, determines what ECU sent the message, and update the time that
    // the ECU sent it's message, as well as resetting any error
    switch(rx_id)
    {
    	// CAN Heartbeat message
//        case 0x023:
//            lastHeartBeatMessage = HAL_GetTick();
//            heartBeatError = 0;
//            break;

        // TEM Message
        case 0x080:
        	lastTemMessage = HAL_GetTick();
			temError = 0;
			break;

		// AMS Message
        case 0x7E3:
        	lastAmsMessage = HAL_GetTick();
			amsError = 0;

			// Check to see if charging
			if(RxData[0] == 0xFF){
				isCharging = 1;
//				if(DISPLAY_CAN_ERRORS){
//					if(RxData[1] != 0x00){
//						printf("AMS CAN Charging Message is Corrupt.\r\n");
//						fflush(stdout);
//					}
//				}
			}
			else if(RxData[0] == 0x00){
				isCharging = 0;
//				if(DISPLAY_CAN_ERRORS){
//					if(RxData[1] != 0xFF){
//						printf("AMS CAN Charging Message is Corrupt.\r\n");
//						fflush(stdout);
//					}
//				}
			}
			break;

		// MC Message
        case 0x41A:
        	lastMcMessage = HAL_GetTick();
			mcError = 0;
			break;

		// Charger Message - UPDATE
		case 0x18FF50E5:
			lastChargerMessage = HAL_GetTick();
			chargerError = 0;
			break;

    }
}


// CAN Transmit Function
HAL_StatusTypeDef CAN_Send(uint32_t id, uint8_t *data, uint8_t length)
{
//    CAN_TxHeaderTypeDef TxHeader;
//    uint32_t TxMailbox;

    TxHeader.StdId = id;
    TxHeader.ExtId = 0;
    TxHeader.IDE   = CAN_ID_STD;
    TxHeader.RTR   = CAN_RTR_DATA;
    TxHeader.DLC   = length;
    TxHeader.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox);
}

// Function to check if a certain message was received over the last half second
uint8_t lastMessageSent(uint32_t lastMessage){
	if ((HAL_GetTick() - lastMessage) >= 3000) {
		return 1;
	}
	return 0;
}


// Function to override print
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ControlPedal */
/**
  * @brief  Function implementing the PedalControl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ControlPedal */
void ControlPedal(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	UBaseType_t highWaterMark;

	HAL_ADC_Start(&hadc1); // Starts ADC1 on STM32
//	HAL_ADC_PollForConversion(&hadc1, 20); // ADC data collected via polling with timeout of 20 units
	  if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK)
	  {
		  inputPedalVoltage = (HAL_ADC_GetValue(&hadc1)) * (3.3 / 4095);

//		  sprintf(msg, "Voltage: %hu\r\n", inputPedalVoltage);
	  }
//	  else
//	  {
//		  	inputPedalVoltage = 1.5;
//	  }
//	  else
//	  {
//		  sprintf(msg, "ADC Timeout\r\n");
//	  }
	//inputPedalVoltage = HAL_ADC_GetValue(&hadc1); // Gets and stores ADC value from ADC1 into variable
//	  sprintf(msg, "Voltage: %hu \r\n", inputPedalVoltage);
//	  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY); // Need to be mindefule of using max delay

//	inputPedalVoltage = 2.5;

	// Will need to think about what to do when pedal voltage is exactly the center voltage. However, this is where deadband may come in
	if (inputPedalVoltage > CenterPedalVoltage[0] && inputPedalVoltage <= MaxPedalVoltage[0]) // Checks if the car is trying to accelerate and is not faulted
	{
		//TxData[0]++; // Increment the first byte

		TxHeader.StdId = CMD_SetRelativeCurrent; // ID the STM is transmitting with

		CalculatedValue = ((inputPedalVoltage - CenterPedalVoltage[0]) / (MaxPedalVoltage[0] - CenterPedalVoltage[0])) * 1000;


		// Splits 16 bit calculated value into two separate bytes. This is because the MC expects a 2 byte long data frame
		// Shifting order is because of Big Endian format on MC
		// 0xFF used to explicitly select the correct bits to be sent.

		TxData[0] = (CalculatedValue >> 8) & 0xFF;   // 0x00
		TxData[1] = CalculatedValue & 0xFF;          // 0xc8

	}
	else if (inputPedalVoltage < CenterPedalVoltage[0] && inputPedalVoltage >= MinPedalVoltage[0]) // Checks if the car is braking and is not faulted
	{
		TxHeader.StdId = CMD_SetRelativeBrakeCurrent; // ID the STM is transmitting with

		CalculatedValue = ((inputPedalVoltage - MinPedalVoltage[0]) / (CenterPedalVoltage[0] - MinPedalVoltage[0])) * 1000;

		TxData[0] = (CalculatedValue >> 8) & 0xFF;   // 0x00
		TxData[1] = CalculatedValue & 0xFF;          // 0xc8
	}
	else
	{
		TxHeader.StdId = 0x111;

		TxData[0] = 0xFF;   // 0x00
		TxData[1] = 0xFF;
	}


//	printf("CMD = 0x%X\r\n", CMD_SetRelativeCurrent);
//	printf("StdId = 0x%X\r\n", TxHeader.StdId);

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0); /* Wait till a Tx mailbox is free. Using while loop instead of HAL_Delay() */

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	 /* Transmission request Error */
	  Error_Handler();
	}

//	highWaterMark = uxTaskGetStackHighWaterMark(NULL);
//
//	printf("Unused stack: %lu words\r\n", (uint32_t)highWaterMark); // Prints how much space a task is not using

    osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DisplayData */
/**
* @brief Function implementing the DataDisplay thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DisplayData */
void DisplayData(void *argument)
{
  /* USER CODE BEGIN DisplayData */
  /* Infinite loop */
  for(;;)
  {

	if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) //  Checks if there are messages waiting in the receive FIFO
	{
		if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
			{
				Error_Handler();
			}

		uint16_t Rx_Id; // Message ID of received data

		if (RxHeader.IDE == CAN_ID_STD) { // Checks if standard or extended CAN ID is being used
			Rx_Id = RxHeader.StdId; // Stores CAN ID of standard CAN message
		} else {
			Rx_Id = RxHeader.ExtId; // Stores CAN ID of extended CAN message
		}

		uint8_t CANBufferLengthUsed = 0;

		//snprintf returns the # of characters written
//		printf("CAN Message: %08X	", Rx_Id);

		CANBufferLengthUsed += snprintf(CANBuffer, sizeof(CANBuffer), "CAN Message: %08X	", Rx_Id); // Appends CAN ID to CANBuffer

		for (int i = 0; i < 8; i++)
		{
			// Prints received CAN data as 2 digit hex on new lines
			// Appends each new data on next empty CANBuffer spot
			CANBufferLengthUsed += snprintf(&CANBuffer[CANBufferLengthUsed], sizeof((CANBuffer) - CANBufferLengthUsed), " %02X", RxData[i]);
		}
		CANBufferLengthUsed += snprintf(&CANBuffer[CANBufferLengthUsed], sizeof((CANBuffer) - CANBufferLengthUsed), "\r\n");

//	    sprintf(msg, "Voltage: %.3f V\r\n", inputPedalVoltage);
//	    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY); // Need to be mindefule of using max delay


		// Sends CANBuffer over UART
		HAL_UART_Transmit(&huart2, (uint8_t *)CANBuffer, CANBufferLengthUsed, HAL_MAX_DELAY); // CANBufferLengthUsed used instead of sizeof(CANBuffer) bc length of the CANBuffer is already known

	}

    osDelay(600);
  }
  /* USER CODE END DisplayData */
}

/* USER CODE BEGIN Header_StartAPPSCalibration */
/**
* @brief Function implementing the APPSCalibration thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAPPSCalibration */
void StartAPPSCalibration(void *argument)
{
  /* USER CODE BEGIN StartAPPSCalibration */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartAPPSCalibration */
}

/* USER CODE BEGIN Header_StartCANWatchdog */
/**
* @brief Function implementing the CANWatchdog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANWatchdog */
void StartCANWatchdog(void *argument)
{
  /* USER CODE BEGIN StartCANWatchdog */
  /* Infinite loop */
  for(;;)
  {

	  // Send CAN Heartbeat message
	 CAN_Send(0x080, WatchDogTxData, 8); // TEM message
	 CAN_Send(0x7E3, WatchDogTxData, 8); // AMS message
	 CAN_Send(0x41A, WatchDogTxData, 8); // MC message


	 // Throw a lil delay to let the CAN message go thru
	 osDelay(50);

	 // Check if each message has been received over the past half second
//	 heartBeatError = lastMessageSent(lastHeartBeatMessage);
	 temError = lastMessageSent(lastTemMessage);
	 amsError = lastMessageSent(lastAmsMessage);
	 mcError = lastMessageSent(lastMcMessage);

	 // If charging enabled, also check for charger
	 if(isCharging)
	 {
		 chargerError = lastMessageSent(lastChargerMessage);
	 }
	 else{
		 chargerError = 0;
	 }


	 // If any of the error flags are high, Shut the relay down
	 if(temError + amsError + mcError + chargerError)
	 {
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // This turns the LED on
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // This turns the LED off
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

		 // Displays to the console any errors
//		 if(DISPLAY_CAN_ERRORS)
//		 {
////			 if(heartBeatError)
////			 {
////				 printf("Heartbeat message not received within a second.\r\n");
////				 fflush(stdout);
////			 }
//			 if(temError)
//			 {
//				 printf("TEM message not received within a second.\r\n");
//				 fflush(stdout);
//			 }
//			 if(amsError)
//			 {
//				 printf("AMS message not received within a second.\r\n");
//				 fflush(stdout);
//			 }
//			 if(mcError)
//			 {
//				 printf("MC message not received within a second.\r\n");
//				 fflush(stdout);
//			 }
//			 if(chargerError)
//			 {
//				 printf("Charger message not received within a second.\r\n");
//				 fflush(stdout);
//		 	 }
//	 	 }
   	 }

    osDelay(700);
  }
  /* USER CODE END StartCANWatchdog */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
