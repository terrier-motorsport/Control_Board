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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

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

#define DISPLAY_CAN_MESSAGE 1			// Turn this on/off if you want to print/ignore raw CAN Messages
#define DISPLAY_CAN_ERRORS 1			// Turn this on/off if you want to print/ignore CAN Errors

// Pedal Calibration Inversion Constants
#define  InvertAnalogOnePedal 0 // Swap the max and min pedal voltages on analog line one
#define  InvertAnalogTwoPedal 0 // Swap the max and min pedal voltages on analog line two

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;

/* Definitions for PedalControl */
osThreadId_t PedalControlHandle;
const osThreadAttr_t PedalControl_attributes = {
  .name = "PedalControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CANWatchdog */
osThreadId_t CANWatchdogHandle;
const osThreadAttr_t CANWatchdog_attributes = {
  .name = "CANWatchdog",
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
/* Definitions for TestCANSend */
osThreadId_t TestCANSendHandle;
const osThreadAttr_t TestCANSend_attributes = {
  .name = "TestCANSend",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DataDisplay */
osThreadId_t DataDisplayHandle;
const osThreadAttr_t DataDisplay_attributes = {
  .name = "DataDisplay",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */

HAL_StatusTypeDef status;
HAL_StatusTypeDef status2;
uint32_t txFreeLevel; // Space in TX FIFO left
uint32_t fdcanError;

FDCAN_TxHeaderTypeDef TxHeader; // Header containing the information of the transmitted frame
FDCAN_RxHeaderTypeDef RxHeader; // Header containing the information of the received frame

uint8_t TxData[8] = {0}; // Buffer of the data to send (8 bytes data)
uint8_t WatchDogTxData[8] = {1}; // Random Buffer data to simulate messages for Watchdog
uint8_t RxData[8]; // Buffer of the received data (8 bytes data)
uint32_t TxMailbox; // The number of the email box that transmitted the Tx message

uint8_t rxByte;

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
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
void ControlPedal(void *argument);
void StartCANWatchdog(void *argument);
void StartAPPSCalibration(void *argument);
void StartTestCANSend(void *argument);
void StartDataDisplay(void *argument);

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */

	HAL_NVIC_SetPriority(USART3_IRQn, 5, 0); // Configures interrupt priority for USART3
	HAL_NVIC_EnableIRQ(USART3_IRQn); // Enables the USART interrupt through NVIC


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

  /* creation of CANWatchdog */
  CANWatchdogHandle = osThreadNew(StartCANWatchdog, NULL, &CANWatchdog_attributes);

  /* creation of APPSCalibration */
  APPSCalibrationHandle = osThreadNew(StartAPPSCalibration, NULL, &APPSCalibration_attributes);

  /* creation of TestCANSend */
  TestCANSendHandle = osThreadNew(StartTestCANSend, NULL, &TestCANSend_attributes);

  /* creation of DataDisplay */
  DataDisplayHandle = osThreadNew(StartDataDisplay, NULL, &DataDisplay_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

	FDCAN_FilterTypeDef filter; // Declares filter

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 20;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 6;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 6;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  // Sets up filter for CAN messages; Allows all messages to pass

	  filter.IdType = FDCAN_STANDARD_ID;          // Apply this filter to 11-bit Standard IDs
	  filter.FilterIndex = 0;                     // First slot in the filter list (0 to 27)
	  filter.FilterType = FDCAN_FILTER_MASK;      // Classic ID + Mask mode (replaces CAN_FILTERMODE_IDMASK)
	  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // Route matching packets directly to FIFO0
	  filter.FilterID1 = 0x0000;                  // Base ID to match (accept all if 0)
	  filter.FilterID2 = 0x0000;                  // Mask value (0x0000 means "don't care", accepting all packets)


      if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) // checks to see if filter is configured correctly
      {
    	  // Filter configuration error
    	  Error_Handler();
      }
      // Starts CAN peripheral if filter config is good
      if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) // Tries to start CAN and checks for error
      {
    	  Error_Handler();
      }


      // Activate CAN RX notifications on FIFO0
      if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) // 0 means it only focuses on Rx interrupts
      {
    	  Error_Handler();
      }

      TxHeader.Identifier  = 0x123; // ID the STM is transmitting with

      // Describes how CAN frame should be transmitted

      TxHeader.TxFrameType  = FDCAN_DATA_FRAME; // Remote Transmission Request (RTR) tells CAN controller we are sending data
      TxHeader.IdType  = FDCAN_STANDARD_ID; // Identifies if we are using extended(29-bit) or standard (11-bit) CAN; It is currently set to standard
      TxHeader.DataLength  = FDCAN_DLC_BYTES_8; // Data Length Code (DLC) -- The number of bytes in the data frame

      TxHeader.FDFormat = FDCAN_CLASSIC_CAN;   // Configures behavior as standard CAN
      TxHeader.BitRateSwitch = FDCAN_BRS_OFF;  // No fast data baud-rate switching (this is for standard CAN)
      TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
      TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // Disables internal timestamp when sending data
      TxHeader.MessageMarker = 0; // Doesn't matter since FIFOControl is off


      TxData[0] = 0;
      TxData[7] = 0xFF;


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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Callback function for data received on UART3/terminal
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &hcom_uart[COM1]) // Checks which COM/UART triggered the callback function. In this case, COM1/UART3
    {
        printf("Received: %c\r\n", rxByte); // Echoes back what was inputed into the terminal

        HAL_UART_Receive_IT(&hcom_uart[COM1], &rxByte, 1); // Tells HAL to continue waiting for another byte of data from the terminal
    }
}

// Callback function for data received on FIFO0
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

	 if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	 {
		 /* Retrieve Rx messages from RX FIFO0 */
		 if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		 {
			 // Reception Error; error thrown if the receive fails
			 Error_Handler();
		 }
	 }


//    CAN_RxHeaderTypeDef RxHeader;
//    uint8_t RxData[8];


    // Determine standard vs extended CAN ID - NEW
	uint32_t rx_id = RxHeader.Identifier; // Message ID of received data
//	if (RxHeader.IdType  == FDCAN_STANDARD_ID) {
//		rx_id = RxHeader.StdId; // Stores CAN ID of standard CAN message
//	} else {
//		rx_id = RxHeader.ExtId; // Stores CAN ID of extended CAN message
//	}

    // Display the CAN message to the serial monitor
//    if(DISPLAY_CAN_MESSAGE){
//
//    	// Change Print syntax based on standard vs. extended
//    	if (RxHeader.IdType == FDCAN_STANDARD_ID){
//    		printf("RX STD ID: 0x%03lX | Data:", rx_id);
//    	}
//		else{
//			printf("RX EXT ID: 0x%08lX | Data:", rx_id);
//		}
//
//		for (int i = 0; i < RxHeader.DataLength; i++)
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
				if(DISPLAY_CAN_ERRORS){
					if(RxData[1] != 0x00){
						printf("AMS CAN Charging Message is Corrupt.\r\n");
						fflush(stdout);
					}
				}
			}
			else if(RxData[0] == 0x00){
				isCharging = 0;
				if(DISPLAY_CAN_ERRORS){
					if(RxData[1] != 0xFF){
						printf("AMS CAN Charging Message is Corrupt.\r\n");
						fflush(stdout);
					}
				}
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
HAL_StatusTypeDef CAN_Send(uint32_t id, uint8_t *data, uint32_t length)
{
//    CAN_TxHeaderTypeDef TxHeader;
//    uint32_t TxMailbox;
	  TxHeader.Identifier = id;
	  TxHeader.TxFrameType  = FDCAN_DATA_FRAME; // Remote Transmission Request (RTR) tells CAN controller we are sending data
	  TxHeader.IdType  = FDCAN_STANDARD_ID; // Identifies if we are using extended(29-bit) or standard (11-bit) CAN; It is currently set to standard

	  switch (length)
	  {
	      case 0: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break; // Data Length Code (DLC) -- The number of bytes in the data frame
	      case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
	      case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
	      case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
	      case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
	      case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
	      case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
	      case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
	      case 8: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;

	      default:
	          // Invalid CAN payload length
	          break;
	  }

	  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;   // Configures behavior as standard CAN
	  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;  // No fast data baud-rate switching (this is for standard CAN)
	  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // Disables internal timestamp when sending data
	  TxHeader.MessageMarker = 0; // Doesn't matter since FIFOControl is off


	  txFreeLevel = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1); // returns 0 -> FIFO is Full
	  	while ((status2 = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)) // Wait till a Tx mailbox is free.
	  	{
//	  		txFreeLevel = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1); // returns 0 -> FIFO is Full
//	  		fdcanError = hfdcan1.ErrorCode; // Returns 32 = 0x20 -> FIFO is Full
//	  		BSP_LED_Toggle(LED_YELLOW);
	  		osDelay(50); // Give back control to scheduler for 1ms

	  	}
//    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
	  	return 0;
}

// Function to check if a certain message was received over the last half second
uint8_t lastMessageSent(uint32_t lastMessage){
	if ((HAL_GetTick() - lastMessage) >= 3000) {
		return 1;
	}
	return 0;
}


// Function to override print
//int _write(int file, char *ptr, int len)
//{
//    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
//    return len;
//}

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
//	  BSP_LED_Toggle(LED_GREEN);    // Turns Green LED (PB0) ON
	  //	UBaseType_t highWaterMark;

	  	HAL_ADC_Start(&hadc1); // Starts ADC1 on STM32
	  //	HAL_ADC_PollForConversion(&hadc1, 20); // ADC data collected via polling with timeout of 20 units
	  	  if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK)
	  	  {
	  		  inputPedalVoltage = (HAL_ADC_GetValue(&hadc1)) * (3.3 / 4095);
//	  		  BSP_LED_Toggle(LED_YELLOW);

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
//	  	  snprintf(msg, sizeof(msg), "Voltage: \r\n");
////	  	  snprintf(CANBuffer, sizeof(CANBuffer), "CAN Message: %08X	", Rx_Id)
//	  	  status2 = HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), HAL_MAX_DELAY); // Need to be mindefule of using max delay

	  	inputPedalVoltage = 2.5;

	  	// Will need to think about what to do when pedal voltage is exactly the center voltage. However, this is where deadband may come in
	  	if (inputPedalVoltage > CenterPedalVoltage[0] && inputPedalVoltage <= MaxPedalVoltage[0]) // Checks if the car is trying to accelerate and is not faulted
	  	{
	  		//TxData[0]++; // Increment the first byte

	  		TxHeader.Identifier = CMD_SetRelativeCurrent; // ID the STM is transmitting with

	  		CalculatedValue = ((inputPedalVoltage - CenterPedalVoltage[0]) / (MaxPedalVoltage[0] - CenterPedalVoltage[0])) * 1000;


	  		// Splits 16 bit calculated value into two separate bytes. This is because the MC expects a 2 byte long data frame
	  		// Shifting order is because of Big Endian format on MC
	  		// 0xFF used to explicitly select the correct bits to be sent.

	  		TxData[0] = (CalculatedValue >> 8) & 0xFF;   // 0x00
	  		TxData[1] = CalculatedValue & 0xFF;          // 0xc8


	  	}
	  	else if (inputPedalVoltage < CenterPedalVoltage[0] && inputPedalVoltage >= MinPedalVoltage[0]) // Checks if the car is braking and is not faulted
	  	{
	  		TxHeader.Identifier = CMD_SetRelativeBrakeCurrent; // ID the STM is transmitting with

	  		CalculatedValue = ((inputPedalVoltage - MinPedalVoltage[0]) / (CenterPedalVoltage[0] - MinPedalVoltage[0])) * 1000;

	  		TxData[0] = (CalculatedValue >> 8) & 0xFF;   // 0x00
	  		TxData[1] = CalculatedValue & 0xFF;          // 0xc8
	  	}
	  	else
	  	{
	  		TxHeader.Identifier = 0x111;

	  		TxData[0] = 0xFF;   // 0x00
	  		TxData[1] = 0xFF;
	  	}


	  //	printf("CMD = 0x%X\r\n", CMD_SetRelativeCurrent);
	  //	printf("StdId = 0x%X\r\n", TxHeader.StdId);

  		txFreeLevel = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1); // returns 0 -> FIFO is Full
  		fdcanError = hfdcan1.ErrorCode; // Returns 32 = 0x20 -> FIFO is Full


	  	while ((status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)) // Wait till a Tx mailbox is free.
	  	{
//	  		txFreeLevel = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1); // returns 0 -> FIFO is Full
//	  		fdcanError = hfdcan1.ErrorCode; // Returns 32 = 0x20 -> FIFO is Full
	  		BSP_LED_Toggle(LED_YELLOW);
	  		osDelay(50); // Give back control to scheduler for 1ms
//	  		Error_Handler();

	  	}

	  //	highWaterMark = uxTaskGetStackHighWaterMark(NULL);
	  //
	  //	printf("Unused stack: %lu words\r\n", (uint32_t)highWaterMark); // Prints how much space a task is not using

	    osDelay(200);
  }
  /* USER CODE END 5 */
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
//	  // Send placeholder CAN messages
//	 CAN_Send(0x080, WatchDogTxData, 8); // TEM message
//	 CAN_Send(0x7E3, WatchDogTxData, 8); // AMS message
//	 CAN_Send(0x41A, WatchDogTxData, 8); // MC message


	 // Throw a lil delay to let the CAN message go thru
	 osDelay(100);

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
		 BSP_LED_On(LED_RED); // This turns the LED on
		 BSP_LED_Off(LED_GREEN);
//		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // This turns the LED on
//		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

		 // Displays to the console any errors
		 if(DISPLAY_CAN_ERRORS)
		 {
//			 if(heartBeatError)
//			 {
//				 printf("Heartbeat message not received within a second.\r\n");
//				 fflush(stdout);
//			 }
			 if(temError)
			 {
				 printf("TEM message not received within a second.\r\n");
				 fflush(stdout);
			 }
			 if(amsError)
			 {
				 printf("AMS message not received within a second.\r\n");
				 fflush(stdout);
			 }
			 if(mcError)
			 {
				 printf("MC message not received within a second.\r\n");
				 fflush(stdout);
			 }
			 if(chargerError)
			 {
				 printf("Charger message not received within a second.\r\n");
				 fflush(stdout);
		 	 }
	 	 }
	 }
	 else
	 {
//		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // This turns the LED off
//		 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

		 BSP_LED_Off(LED_RED);
		 BSP_LED_On(LED_GREEN);
   	 }

    osDelay(300);
  }
  /* USER CODE END StartCANWatchdog */
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

/* USER CODE BEGIN Header_StartTestCANSend */
/**
* @brief Function implementing the TestCANSend thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTestCANSend */
void StartTestCANSend(void *argument)
{
  /* USER CODE BEGIN StartTestCANSend */
  /* Infinite loop */
  for(;;)
  {

	 // Tells HAL to enable reception via interrupt
	 HAL_UART_Receive_IT(&hcom_uart[COM1], &rxByte, 1);  // Receives 1 byte of data and triggers interrupt


	  // Send placeholder CAN messages
	 CAN_Send(0x080, WatchDogTxData, 8); // TEM message
//	 osDelay(50);
	 CAN_Send(0x7E3, WatchDogTxData, 8); // AMS message
//	 osDelay(50);
	 CAN_Send(0x41A, WatchDogTxData, 8); // MC message

	 osDelay(50);
  }
  /* USER CODE END StartTestCANSend */
}

/* USER CODE BEGIN Header_StartDataDisplay */
/**
* @brief Function implementing the DataDisplay thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDataDisplay */
void StartDataDisplay(void *argument)
{
  /* USER CODE BEGIN StartDataDisplay */
  /* Infinite loop */
  for(;;)
  {

	uint32_t rx_id_data_display = RxHeader.Identifier; // Message ID of received data

	// Display the CAN message to the serial monitor
	if(DISPLAY_CAN_MESSAGE){

		// Change Print syntax based on standard vs. extended
		if (RxHeader.IdType == FDCAN_STANDARD_ID){
			printf("RX STD ID: 0x%03lX | Data:", rx_id_data_display);
		}
		else{
			printf("RX EXT ID: 0x%08lX | Data:", rx_id_data_display);
		}

		for (int i = 0; i < RxHeader.DataLength; i++)
			printf(" %02X", RxData[i]);

		printf("\r\n");

		osDelay(400);
	}
	else
	{
		osThreadSuspend(DataDisplayHandle);
	}
  }
  /* USER CODE END StartDataDisplay */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
