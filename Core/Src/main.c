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
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define DISPLAY_CAN_MESSAGE 1			// Turn this on/off if you want to print/ignore raw CAN Messages
//#define DISPLAY_CAN_ERRORS 1			// Turn this on/off if you want to print/ignore CAN Errors

// Pedal Calibration Inversion Constants
#define  InvertAnalogOnePedal 0 // Swap the max and min pedal voltages on analog line one
#define  InvertAnalogTwoPedal 0 // Swap the max and min pedal voltages on analog line two
#define TEST_MODE

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
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for CANWatchdog */
osThreadId_t CANWatchdogHandle;
const osThreadAttr_t CANWatchdog_attributes = {
  .name = "CANWatchdog",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for APPSCalibration */
osThreadId_t APPSCalibrationHandle;
const osThreadAttr_t APPSCalibration_attributes = {
  .name = "APPSCalibration",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
/* Definitions for Commands */
osThreadId_t CommandsHandle;
const osThreadAttr_t Commands_attributes = {
  .name = "Commands",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for readCANRX */
osThreadId_t readCANRXHandle;
const osThreadAttr_t readCANRX_attributes = {
  .name = "readCANRX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for pedalErrorCheck */
osThreadId_t pedalErrorCheckHandle;
const osThreadAttr_t pedalErrorCheck_attributes = {
  .name = "pedalErrorCheck",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CANRxQueue */
osMessageQueueId_t CANRxQueueHandle;
const osMessageQueueAttr_t CANRxQueue_attributes = {
  .name = "CANRxQueue"
};
/* Definitions for DisplayDataQueue */
osMessageQueueId_t DisplayDataQueueHandle;
const osMessageQueueAttr_t DisplayDataQueue_attributes = {
  .name = "DisplayDataQueue"
};
/* Definitions for processCAN */
osMessageQueueId_t processCANHandle;
const osMessageQueueAttr_t processCAN_attributes = {
  .name = "processCAN"
};
/* USER CODE BEGIN PV */

uint8_t DISPLAY_CAN_MESSAGE = 0;			// Turn this on/off if you want to print/ignore raw CAN Messages
uint8_t DISPLAY_CAN_ERRORS = 0;			// Turn this on/off if you want to print/ignore CAN Errors


///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// CAN Bus /////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


HAL_StatusTypeDef status;
HAL_StatusTypeDef status2;
HAL_StatusTypeDef status3;
uint32_t txFreeLevel; // Space in TX FIFO left
HAL_FDCAN_StateTypeDef fdcanError;


// global var used by many rtos tasks
// risk of race condition
FDCAN_TxHeaderTypeDef TxHeader; // Header containing the information of the transmitted frame
FDCAN_RxHeaderTypeDef RxHeader; // Header containing the information of the received frame

uint8_t TxData[8] = {0}; // Buffer of the data to send (8 bytes data)
uint8_t WatchDogTxData[8] = {1, 0, 0, 0, 0, 0, 0, 0}; // Random Buffer data to simulate messages for Watchdog
uint8_t RxData[8]; // Buffer of the received data (8 bytes data)
uint32_t TxMailbox; // The number of the email box that transmitted the Tx message

uint8_t serialByte; // Rx buffer for data received via USART3/terminal


///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Motor Controller ////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

const uint8_t mcNodeId = 0x1a; // Node ID of the MC = 26

// MC commands
const uint16_t cmdSetRelativeCurrent = ((0x05 << 5) + mcNodeId); // Set relative AC current message ID
const uint16_t cmdSetRelativeBrakeCurrent = ((0x06 << 5) + mcNodeId);  // Set relative AC brake current message ID
const uint16_t cmdSetMaxAcCurrent = ((0x06 << 5) + mcNodeId); // Set max AC current limit message ID
const uint16_t cmdSetMaxBrakeCurrent = ((0x09 << 5) + mcNodeId); // Set max AC brake current limit message ID
const uint16_t cmdSetMaxDcCurrent = ((0x0A << 5) + mcNodeId); // Set max DC current limit message ID
const uint16_t cmdSetMaxDcBrakeCurrent = ((0x0B << 5) + mcNodeId); // Set max DC brake current limit message ID
const uint16_t cmdDriveEnable = ((0x0C << 5) + mcNodeId); // Set max DC brake current limit message ID


// Data from MC
const uint16_t RCV_MC_GeneralDataOne = ((0x20 << 5) + mcNodeId); // Using | instead of + wouldn't work here bc there would be overlap between the node ID and the packet ID

// AMS Limits - This is only here temporarily, the limits will be controlled by the AMS
uint16_t DischargeCurrentLimit = 30; // AMS discharge current limit
uint16_t ChargeCurrentLimit = 20; // AMS charge current limit

// Pedal Calibration Constants
float maxPedalVoltage[2] = {3.3, 0.759}; // Max voltages on both analog lines for the pedal input
float minPedalVoltage[2] = {0.5, 0.517}; // Min voltages on both analog lines for the pedal input
float centerPedalVoltage[2] = {0, 0}; // Center voltages on both analog lines for the pedal input. This voltage corresponds position in which braking/throttle is swapped
float PedalDeadbandPercentage = 0.15; // Percent deadband of pedal

// Pedal Error Checking
float maxPedalVoltageDifferencePercentage = 10; //



uint32_t lastThrottleOrBrake = 0; // Gets the time at which a throttle or break CAN message is sent

float testPedalVoltage[2] = {2.0, 1.5}; // temporary test before circuit test implemented
float inputPedalVoltage[2] = {0.0, 0.0}; // voltage that the APPS is outputting to the STM
float inputPedalVoltagePercentages[2] = {0.0, 0.0}; // APPS output voltages as a percentage

char msg[20];
char CANBuffer[100]; // Buffer used to display CAN messages on terminal
uint16_t CalculatedValue = 0; // The value to be sent over to the MC over CAN



///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////CAN Watchdog /////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

// Initializes variables to be used to check when the last respective message was sent
// uint32_t lastHeartBeatMessage = 0;
 uint32_t lastTemMessage = 0;
 uint32_t lastAmsMessage = 0;
 uint32_t lastMcMessage = 0;
 uint32_t lastChargerMessage = 0;

 uint8_t temSeen = 0;
 uint8_t amsSeen = 0;
 uint8_t mcSeen = 0;
 uint8_t chargerSeen = 0;

// uint32_t lastMessage = 0;
 uint32_t now = 0;

 // Initializes flags to indicate when a certain ECU hasn't sent a message in 500ms, timing out
// uint8_t heartBeatError = 0;
 uint8_t temError = 0;
 uint8_t amsError = 0;
 uint8_t mcError = 0;
 uint8_t chargerError = 0;

 // Creates int to determine if we are charging or not
 uint8_t isCharging = 0;

 static const uint8_t MAX_CAN_MESSAGES = 10; // Maximum # of CAN messages CANWatchdog checks before osDelay


///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Message Display //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


// Toggle between displaying all or select CAN messages
uint8_t toggleCANRx = 0; // 0 = display all messages, 1 = display select messages
uint8_t packet;



///////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Queue Debugging //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


 uint32_t queueCount; // Amount of items in queue
 uint32_t queueSpace; // Amount of available space left in queue
 osStatus_t queueStatus; // Execution status of the queue


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC1_Init(void);
void ControlPedal(void *argument);
void StartCANWatchdog(void *argument);
void StartAPPSCalibration(void *argument);
void StartTestCANSend(void *argument);
void StartDataDisplay(void *argument);
void StartCommands(void *argument);
void StartReadingCANRX(void *argument);
void StartPedalErrorCheck(void *argument);

/* USER CODE BEGIN PFP */

// Prototype for CAN Transmission
HAL_StatusTypeDef CAN_Send(uint32_t id, uint8_t *data, uint32_t length);

// Prototype to see if a certain message hasn't been seen in over 500ms
uint8_t lastMessageSent(uint32_t lastMessage);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct
{
	uint32_t id;
	uint32_t idType;
	uint32_t dataLength;
	uint8_t data[8];

} CANRxMessage;

CANRxMessage queueBufferMessage; // Data being stored in the queue
CANRxMessage queueBufferRecieved; // Data being read from the queue


// System states
typedef enum
{
	PEDAL_CALIBRATION, // Assigned 0
	MOTOR_SPINNING,    // Assigned 1
	WATCHDOG_FAULTED,  // Assigned 2
	FAULT, 			   // Assigned 3
	READY,			   // Assigned 4
} State;

volatile State currentState = WATCHDOG_FAULTED; // init at fault on new boot, changed when safety conditions are met

/*  More realistic states for future implementation

typedef enum
{
  STATE_INIT,
  STATE_WAITING_FOR_ECUS,
  STATE_READY,
  STATE_RUNNING,
  STATE_APPS_CALIBRATION,
  STATE_CHARGING,
  STATE_FAULT
} SystemState;

SystemState currentState = STATE_INIT;

// helper function to change state for consistency
void setSysState(SystemState newState){
  currentState = newState;
}

*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

//	HAL_Delay(30000); // Delay for when the car starts up

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
  MX_FDCAN1_Init();
  MX_ADC1_Init();
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

  /* Create the queue(s) */
  /* creation of CANRxQueue */
  CANRxQueueHandle = osMessageQueueNew (16, sizeof(CANRxMessage), &CANRxQueue_attributes);

  /* creation of DisplayDataQueue */
  DisplayDataQueueHandle = osMessageQueueNew (16, sizeof(CANRxMessage), &DisplayDataQueue_attributes);

  /* creation of processCAN */
  processCANHandle = osMessageQueueNew (16, sizeof(CANRxMessage), &processCAN_attributes);

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

  /* creation of Commands */
  CommandsHandle = osThreadNew(StartCommands, NULL, &Commands_attributes);

  /* creation of readCANRX */
  readCANRXHandle = osThreadNew(StartReadingCANRX, NULL, &readCANRX_attributes);

  /* creation of pedalErrorCheck */
  pedalErrorCheckHandle = osThreadNew(StartPedalErrorCheck, NULL, &pedalErrorCheck_attributes);

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

	FDCAN_FilterTypeDef filter = {0}; // Declares filter

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
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 6;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 6;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  // Sets up filters for CAN messages;
  // Filter One
  // Allows all messages to pass

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

//      TxData[0] = 0;
//      TxData[7] = 0xFF;


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
  __HAL_RCC_GPIOD_CLK_ENABLE();
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
        printf("Received: %c\r\n", serialByte); // Echoes back what was inputed into the terminal

        HAL_UART_Receive_IT(&hcom_uart[COM1], &serialByte, 1); // Tells HAL to continue waiting for another byte of data from the terminal
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

//	 lastMessage = HAL_GetTick();
	 
   now = HAL_GetTick();
//   BSP_LED_On(LED_RED);
   CANRxMessage CANmsg;
	 CANmsg.id = RxHeader.Identifier;
	 CANmsg.idType = RxHeader.IdType;
	 CANmsg.dataLength = RxHeader.DataLength;
	 memcpy(CANmsg.data, RxData, 8); // Passes a copy of RxData to CANmsg.data; '=' would not work here since array assignment can't be done, plus it would become a pointer
//	 queueBufferMessage = CANmsg;

	 osMessageQueuePut(CANRxQueueHandle, &CANmsg, 0, 0); // Adds received CAN message to CANRx queue
	 osMessageQueuePut(processCANHandle, &CANmsg, 0, 0); // Adds received CAN message to CANRx queue
	 osMessageQueuePut(DisplayDataQueueHandle, &CANmsg, 0, 0);

}


// CAN Transmit Function
HAL_StatusTypeDef CAN_Send(uint32_t id, uint8_t *data, uint32_t length)
{
    // local to the function; see global declaration comments
    FDCAN_TxHeaderTypeDef txHeader = {0};
//    CAN_TxHeaderTypeDef txHeader;
//    uint32_t TxMailbox;
	  txHeader.Identifier = id;
	  txHeader.TxFrameType  = FDCAN_DATA_FRAME; // Remote Transmission Request (RTR) tells CAN controller we are sending data
	  txHeader.IdType  = FDCAN_STANDARD_ID; // Identifies if we are using extended(29-bit) or standard (11-bit) CAN; It is currently set to standard

	  switch (length)
	  {
	      case 0: txHeader.DataLength = FDCAN_DLC_BYTES_0; break; // Data Length Code (DLC) -- The number of bytes in the data frame
	      case 1: txHeader.DataLength = FDCAN_DLC_BYTES_1; break;
	      case 2: txHeader.DataLength = FDCAN_DLC_BYTES_2; break;
	      case 3: txHeader.DataLength = FDCAN_DLC_BYTES_3; break;
	      case 4: txHeader.DataLength = FDCAN_DLC_BYTES_4; break;
	      case 5: txHeader.DataLength = FDCAN_DLC_BYTES_5; break;
	      case 6: txHeader.DataLength = FDCAN_DLC_BYTES_6; break;
	      case 7: txHeader.DataLength = FDCAN_DLC_BYTES_7; break;
	      case 8: txHeader.DataLength = FDCAN_DLC_BYTES_8; break;

	      default:
	          // Invalid CAN payload length
	    	  txHeader.DataLength = FDCAN_DLC_BYTES_8; break;
	          break;
	  }

	  txHeader.FDFormat = FDCAN_CLASSIC_CAN;   // Configures behavior as standard CAN
	  txHeader.BitRateSwitch = FDCAN_BRS_OFF;  // No fast data baud-rate switching (this is for standard CAN)
	  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // Disables internal timestamp when sending data
	  txHeader.MessageMarker = 0; // Doesn't matter since FIFOControl is off


	  txFreeLevel = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1); // returns 0 -> FIFO is Full
	  	while ((status2 = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data)) != HAL_OK) // Wait till a Tx mailbox is free.
	  	{
//	  		txFreeLevel = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1); // returns 0 -> FIFO is Full
//	  		fdcanError = hfdcan1.ErrorCode; // Returns 32 = 0x20 -> FIFO is Full
	  		osDelay(50); // Give back control to scheduler for 1ms

	  	}
//    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
//           --> status2
	  	return status2;
}

// Function to check if a certain message was received over the last half second
uint8_t lastMessageSent(uint32_t lastMessage){
	if ((HAL_GetTick() - lastMessage) >= 500) { // 500ms = 0.5s
		return 1;
	}
	return 0;
}


// Function that performs the Watchdog checks
void watchdogCheck(CANRxMessage canMSG)
{
	 switch (canMSG.id){
		case 0x080:
		  lastTemMessage = now;
		  temSeen = 1;
		  break;

		case 0x7E3:
		  lastAmsMessage = now;
		  amsSeen = 1;

		  if (canMSG.data[0] == 0xFF){
			isCharging = 1;

			if (DISPLAY_CAN_ERRORS){
			  if (canMSG.data[1] != 0x00){
				printf("AMS CAN Charging Message is Corrupt.\r\n");
				fflush(stdout);
			  }
			}
		  } else if (canMSG.data[0] == 0x00){
			isCharging = 0;

			if (DISPLAY_CAN_ERRORS){
			  if (canMSG.data[1] != 0xFF){
				printf("AMS CAN Charging Message is Corrupt.\r\n");
				fflush(stdout);
			  }
			}
		  }

		  break;

		case 0x41A: // (0x20 << 5) + 0x1a
		  lastMcMessage = now;
		  mcSeen = 1;
		  break;

		case 0x18FF50E5:
		  lastChargerMessage = now;
		  chargerSeen = 1;
		  break;
	 }


	 // Check if each message has been received over the past half second
	 //heartBeatError = lastMessageSent(lastHeartBeatMessage);

	  temError = lastMessageSent(lastTemMessage);
	  amsError = lastMessageSent(lastAmsMessage);
	  mcError = lastMessageSent(lastMcMessage);

	  if (isCharging){
		chargerError = lastMessageSent(lastChargerMessage);
	  } else {
		chargerError = 0;
	  }

	  if (temError || amsError || mcError || chargerError){

		// Uses on board LED or external LEDs depending on whether the STM is in test mode
		// BSP LED syntax will not be highlighted if not in TEST mode, and vice versa
		#ifdef TEST_MODE

			BSP_LED_On(LED_RED);
			BSP_LED_Off(LED_GREEN);

		#else

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // This turns the LED on
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

		#endif



		// Displays to the console any errors
		if(DISPLAY_CAN_ERRORS){
 //			  if(heartBeatError)
 //			  {
 //				  printf("Heartbeat message not received within 500ms.\r\n");
 //				  fflush(stdout);
 //			  }
		  if(temError)
		  {
			printf("TEM message not received within 500ms.\r\n");
			fflush(stdout);
		  }
		  if(amsError)
		  {
			printf("AMS message not received within 500ms.\r\n");
			fflush(stdout);
		  }
		  if(mcError)
		  {
			printf("MC message not received within 500ms.\r\n");
			fflush(stdout);
		  }
		  if(chargerError)
		  {
			printf("Charger message not received within 500ms.\r\n");
			fflush(stdout);
		  }
		}
	  } else {

			#ifdef TEST_MODE

		        BSP_LED_Off(LED_RED);
		        BSP_LED_On(LED_GREEN);

			#else

				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // This turns the LED off
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

			#endif
	  }
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

	HAL_ADC_Start(&hadc1); // Starts ADC1 on STM32
//  osThreadSuspend(PedalControlHandle);
	centerPedalVoltage[0] = ((maxPedalVoltage[0] - minPedalVoltage[0]) / 2);
	centerPedalVoltage[1] = ((maxPedalVoltage[1] - minPedalVoltage[1]) / 2);

  FDCAN_TxHeaderTypeDef pedalTxHeader = {0};

  pedalTxHeader.TxFrameType = FDCAN_DATA_FRAME;
  pedalTxHeader.IdType = FDCAN_STANDARD_ID;
  pedalTxHeader.DataLength = FDCAN_DLC_BYTES_8;
  pedalTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  pedalTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  pedalTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  pedalTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  pedalTxHeader.MessageMarker = 0;

  /* Infinite loop */
  for(;;)
  {
	  memset(TxData, 0xFF, sizeof(TxData)); // Sets all the bytes in TxData to 0xFF

//	  pedalTxHeader.Identifier = CMD_DriveEnable;
//	  TxData[0] = (1 >> 8) & 0xFF;   // 0x00
//	  TxData[1] = 1 & 0xFF;          // 0xc8

#ifdef TEST_MODE
      inputPedalVoltage[0] = testPedalVoltage[0];
      inputPedalVoltage[1] = testPedalVoltage[1];
#else
	  //	UBaseType_t highWaterMark;

	  	  if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK) // ADC data collected via polling with timeout of 20 units
	  	  {
	  		  inputPedalVoltage[0] = (HAL_ADC_GetValue(&hadc1)) * (3.3 / 65535); // Pedal voltage in 3.3V range

	  	  }
#endif
	  	// Will need to think about what to do when pedal voltage is exactly the center voltage. However, this is where deadband may come in
	  	if (inputPedalVoltage[0] > centerPedalVoltage[0] && inputPedalVoltage[0] <= maxPedalVoltage[0]) // Checks if the car is trying to accelerate and is not faulted
	  	{

	  		pedalTxHeader.Identifier = cmdSetRelativeCurrent; // ID the STM is transmitting with

	  		CalculatedValue = ((inputPedalVoltage[0] - centerPedalVoltage[0]) / (maxPedalVoltage[0] - centerPedalVoltage[0])) * 1000;


	  		// Splits 16 bit calculated value into two separate bytes. This is because the MC expects a 2 byte long data frame
	  		// Shifting order is because of Big Endian format on MC
	  		// 0xFF used to explicitly select the correct bits to be sent.

	  		TxData[0] = (CalculatedValue >> 8) & 0xFF;   // 0x00
	  		TxData[1] = CalculatedValue & 0xFF;          // 0xc8



	  	}
	  	else if (inputPedalVoltage[0] < centerPedalVoltage[0] && inputPedalVoltage[0] >= minPedalVoltage[0]) // Checks if the car is braking and is not faulted
	  	{
	  		pedalTxHeader.Identifier = cmdSetRelativeBrakeCurrent; // ID the STM is transmitting with

	  		CalculatedValue = ((inputPedalVoltage[0] - minPedalVoltage[0]) / (centerPedalVoltage[0] - minPedalVoltage[0])) * 1000;

	  		TxData[0] = (CalculatedValue >> 8) & 0xFF;   // 0x00; Shifts the value to the right by 1 byte, allowing only 0s to be read with masking
	  		TxData[1] = CalculatedValue & 0xFF;          // 0xc8; Only reads the actual value
	  	}
	  	else
	  	{

//	  		pedalTxHeader.Identifier = cmdDriveEnable;
//		    TxData[0] = (0 >> 8) & 0xFF;   // 0x00
//		    TxData[1] = 0 & 0xFF;
	  	}

  		txFreeLevel = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1); // returns 0 -> FIFO is Full
  		//fdcanError = hfdcan1.ErrorCode; // Returns 32 = 0x20 -> FIFO is Full
  		fdcanError = HAL_FDCAN_GetError(&hfdcan1);


	  	while ((status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pedalTxHeader, TxData)) != HAL_OK) // Wait till a Tx mailbox is free.
	  	{
//	  		txFreeLevel = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1); // returns 0 -> FIFO is Full
//	  		fdcanError = hfdcan1.ErrorCode; // Returns 32 = 0x20 -> FIFO is Full
	  		BSP_LED_Toggle(LED_YELLOW);
	  		osDelay(50); // Give back control to scheduler for 1ms
//	  		Error_Handler();

	  	}

//	  //	highWaterMark = uxTaskGetStackHighWaterMark(NULL);
//	  //
//	  //	printf("Unused stack: %lu words\r\n", (uint32_t)highWaterMark); // Prints how much space a task is not using

	    osDelay(20); // Without delay the Tx Mailbox reaches an error, most likely it fills up to quick. Needs to be checked to confirm
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

	// Process the first CAN message in a batch of 10 CAN messages
	// the following watchdog relies on receiving messages from osMessageQueueGet()
	// if nothing arrives (i.e., all ECUs are dead) it will stop functioning

	 queueCount = osMessageQueueGetCount(CANRxQueueHandle);
	 queueSpace = osMessageQueueGetSpace(CANRxQueueHandle);

	 CANRxMessage canMSG;
	 osMessageQueueGet(CANRxQueueHandle, &canMSG, NULL, 100); // Gets and removes item from CANRx queue; puts task in blocked state for 100ms if queue is full


	 queueBufferRecieved = canMSG; // For testing

	// Based on the CAN ID, determines what ECU sent the message, and update the time that
	// the ECU sent it's message, as well as resetting any error
	 watchdogCheck(canMSG);

	  // Check the 9 other CAN messages in a batch
	  for (int i = 0; i < (MAX_CAN_MESSAGES - 1); i++)
	  {
		  if (osMessageQueueGet(CANRxQueueHandle, &canMSG, NULL, 100) == osOK) // Blocks for 100ms unless CAN message obtained from queue
		  {
			  watchdogCheck(canMSG);
		  }
		  else
		  {
			  break;
		  }
	  }


	  osDelay(50);
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
	printf("APPS Calibration Started \r\n");
	osThreadSuspend(APPSCalibrationHandle); // Suspends APPS calibration task
//	printf("APPS Calibration Started");

  /* Infinite loop */
  for(;;)
  {
	// Task should be suspended if the motor is spinning
//	  BSP_LED_Toggle(LED_YELLOW);
	  HAL_ADC_Start(&hadc1); // Starts ADC1 on STM32

	  if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK)
	  {
		  inputPedalVoltage[0] = (HAL_ADC_GetValue(&hadc1)) * (3.3 / 65535);
//		  BSP_LED_Toggle(LED_YELLOW);

	  }
	  else
	  {
//		  BSP_LED_Toggle(LED_RED);
	  }

	  if (inputPedalVoltage[0] > maxPedalVoltage[0])
	  {
		  maxPedalVoltage[0] = inputPedalVoltage[0];
	  }
	  else if (inputPedalVoltage[0] < minPedalVoltage[0])
	  {
		minPedalVoltage[0] = inputPedalVoltage[0];;
	  }

	  // centerPedalVoltage = minPedalVoltage + ((maxPedalVoltage - minPedalVoltage) / 2);

	  // printf("Max Pedal Voltage: %.3f ", maxPedalVoltage[0]);
	  // printf("Min Pedal Voltage: %.3f ", minPedalVoltage[0]);
	  printf("Reading Pedal Voltage... \r\n");



    osDelay(20);
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
	 HAL_UART_Receive_IT(&hcom_uart[COM1], &serialByte, 1);  // Receives 1 byte of data and triggers interrupt
//
//
//	  // Send placeholder CAN messages
	 CAN_Send(0x080, WatchDogTxData, 8); // TEM message
	 CAN_Send(0x7E3, WatchDogTxData, 8); // AMS message
//	 CAN_Send(cmdDriveEnable, WatchDogTxData, 8); // MC message

	 CAN_Send(RCV_MC_GeneralDataOne, WatchDogTxData, 8); // General MC data

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

	// Display the CAN message to the serial monitor
	if(DISPLAY_CAN_MESSAGE){

	    CANRxMessage CANPrintmsg;
	    osMessageQueueGet(DisplayDataQueueHandle, &CANPrintmsg, NULL, osWaitForever); // Pull data from DisplayData queue


		// Change Print syntax based on standard vs. extended
		if (CANPrintmsg.idType == FDCAN_STANDARD_ID){
			printf("RX STD ID: 0x%03lX | Data:", CANPrintmsg.id);
		}
		else{
			printf("RX EXT ID: 0x%08lX | Data:", CANPrintmsg.id);
		}

		for (int i = 0; i < CANPrintmsg.dataLength; i++)
			printf(" %02X", CANPrintmsg.data[i]);

		printf("\r\n");
//		osDelay(50);

	}
	else
	{
		osThreadSuspend(DataDisplayHandle); // Suspends the DataDisplay task
	}
  }
  /* USER CODE END StartDataDisplay */
}

/* USER CODE BEGIN Header_StartCommands */
/**
* @brief Function implementing the Commands thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommands */
void StartCommands(void *argument)
{
  /* USER CODE BEGIN StartCommands */
  /* Infinite loop */
  for(;;)
  {


	switch(serialByte) // Reads values as ascii characters; numbers need to be converted to char type before checking
	{
		case 'a':
		{
//			osThreadSuspend(DataDisplayHandle);
			DISPLAY_CAN_MESSAGE = 0;
//			DISPLAY_CAN_ERRORS = 0;
			break;
		}
		case 'd':
		{
//			osThreadResume(DataDisplayHandle);
			DISPLAY_CAN_MESSAGE = 1;
//			DISPLAY_CAN_ERRORS = 1;
			osThreadResume(DataDisplayHandle);
			break;
		}
		case 'c': // Begin pedal calibration
		{
			if (currentState == MOTOR_SPINNING) // Don't allow calibration to begin if the motor is still rotating
			{
				break;
			}

			DISPLAY_CAN_MESSAGE = 0;
			printf("Starting Pedal Calibration... \r\n");
			status3 = osThreadSuspend(PedalControlHandle); // Suspends pedal control task

//			osDelay(50);

      // start at the opposite end and calibrate towards the desired end 
      // 0.0 --up--> max 
      // 3.3 -down-> min
			maxPedalVoltage[0] = 0.0f;
			minPedalVoltage[0] = 3.3f;

			printf("Click enter to complete calibration \r\n");
			osThreadResume(APPSCalibrationHandle); // Resumes APPS calibration task

			break; // Without break, other cases below will all execute until a break statement occurs
		}
		case 'x': // Space bar is clicked; end pedal calibration
		{
			centerPedalVoltage[0] = ((maxPedalVoltage[0] - minPedalVoltage[0]) / 2);
			centerPedalVoltage[1] = ((maxPedalVoltage[1] - minPedalVoltage[1]) / 2);
			osThreadSuspend(APPSCalibrationHandle);
			printf("Ending Pedal Calibration...");
			DISPLAY_CAN_MESSAGE = 1;
			osThreadResume(PedalControlHandle);

//			osThreadResume(DataDisplayHandle);
			break;
		}
//		case 't': // Toggles display variable for displaying CAN messages
//		{
//			toggleCANRx = !toggleCANRx; // toggles the value of toggleCANRx; 0 -> 1, 1 -> 0
//			break;
//		}

		default: break;
	}
	serialByte = 0;

   osDelay(50);
  }
  /* USER CODE END StartCommands */
}

/* USER CODE BEGIN Header_StartReadingCANRX */
/**
* @brief Function implementing the readCANRX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadingCANRX */
void StartReadingCANRX(void *argument)
{
  /* USER CODE BEGIN StartReadingCANRX */
  /* Infinite loop */
  for(;;)
  {
//	CANRxMessage readCAN;
//	osMessageQueueGet(processCANHandle, &readCAN, NULL, osWaitForever); // Gets and removes item from CANRx queue; puts task in blocked state if queue is full
//
//	if((readCAN.id & 0xFF) == mcNodeId) // Checks if an MC message was received
//	{
//		// Relevant MC messages
//		switch(readCAN.id >> 5) // Need to get this to be the
//		{
//			case 0x20: // General MC message
//			{
//				BSP_LED_Toggle(LED_YELLOW);
//				break;
//			}
//			default:
//				break;
//		}
//	}

    osDelay(1000);
  }
  /* USER CODE END StartReadingCANRX */
}

/* USER CODE BEGIN Header_StartPedalErrorCheck */
/**
* @brief Function implementing the pedalErrorCheck thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPedalErrorCheck */
void StartPedalErrorCheck(void *argument)
{
  /* USER CODE BEGIN StartPedalErrorCheck */
  /* Infinite loop */
  for(;;)
  {
	  inputPedalVoltagePercentages[0] = (inputPedalVoltage[0])/(maxPedalVoltage[0]-minPedalVoltage[0]); // APPS output 1 voltage percentage
	  inputPedalVoltagePercentages[1] = (inputPedalVoltage[1])/(maxPedalVoltage[1]-minPedalVoltage[1]); // APPS output 2 voltage percentage

	  // Checks if the difference between both pedal outputs is greater than the max allowed percent difference
	if (fabsf(inputPedalVoltagePercentages[0] - inputPedalVoltagePercentages[1]) > (maxPedalVoltageDifferencePercentage/100)) // Use fabsf() for floats
	{
		currentState = FAULT;
//		BSP_LED_Toggle(LED_YELLOW);
	}
	else
	{
		currentState = READY;
	}
    osDelay(50);
  }
  /* USER CODE END StartPedalErrorCheck */
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
