/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
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
 CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
//AT commands to send to the Xbee Module
uint8_t AT_ENTER[] = {'+','+','+'}; //Command to enter AT mode
uint8_t AT_OK[] = {'O', 'K', 0x0D}; //AT OK response
uint8_t ATSD_R[] = {'A', 'T', 'S', 'D','1', 0x0D}; //AT soft reset command
uint8_t ATSD_S[] = {'A', 'T', 'S', 'D','0', 0x0D}; //AT shutdown command
uint8_t ATFR[] = {'A', 'T', 'F', 'R', 0x0D}; //Force reset command
uint8_t ATAI[] = {'A', 'T', 'A', 'I', 0x0D}; //Association indication command, tells state of connection
//ATAI command responses
uint8_t AT_CONNECTED[] = {'0', 0x0D}; //Connected to internet
uint8_t AT_REG[] = {'2', '2', 0x0D}; //Registerring to cellular network
uint8_t AT_CON[] = {'2', '3', 0x0D}; //Connecting to internet
uint8_t AT_COR[] = {'2', '4', 0x0D}; //Firmware corrupt or missing
uint8_t AT_DEN[] = {'2', '5', 0x0D}; //Connection denied; may be power consumption related
uint8_t AT_AIR[] = {'2', 'A', 0x0D}; //In airplane mode
uint8_t AT_USB[] = {'2', 'B', 0x0D}; //USB Direct Active
uint8_t AT_PSM[] = {'2', 'C', 0x0D}; //Cellular component in PSM
uint8_t AT_SD[] = {'2', 'D', 0x0D}; //Modem shut down
uint8_t AT_VOL[] = {'2', 'E', 0x0D}; // Low voltage shut down
uint8_t AT_BYP[] = {'2', 'F', 0x0D}; //Bypass mode active
uint8_t AT_UPD[] = {'3', '0', 0x0D}; //Update in progress
uint8_t AT_TST[] = {'3', '1', 0x0D}; //Regulatory testing enabled
uint8_t AT_INI[] = {'F', 'F', 0x0D}; //Initializing
//MQTT Messages
uint8_t mqtt_connect[] = {0x10, 0x10, 0x00, 0x04, 0x4D, 0x51, 0x54, 0x54, 0x04, 0x02, 0x00, 0x3C, 0x00, 0x04, 0x44, 0x49, 0x47, 0x49}; //MQTT connect message
uint8_t rx_transmit[2048] = {0x32, 0x8C, 0x00, 0x00, 0x08, 0x6D, 0x65, 0x73, 0x73, 0x61, 0x67, 0x65, 0x73, 0x00, 0x2A}; //MQTT data transmission message
//MQTT Responses
uint8_t MQTT_CONACK[] = {0x20,0x02,0x00,0x00}; //Connection acknowlegement response
uint8_t MQTT_PUBACK[] = {0x40,0x02,0x00,0x2A}; //Message published acknowlegement response

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t uart_rx[4]; //UART receive Buffer
uint16_t last_element = 14;	 //Last array element location of rx_transmit
uint8_t *id_array[128];  // Array of pointers indicating location of any given ID in rx_transmit
uint8_t timeout_count = 0; //Timeout counter

//State flags
struct sFlags
{
	bool connected_net;
	bool connected_mqtt;
	bool transmit_ready;
	bool at_enabled;
	bool xbee_fr;
	bool at_ok;
};
struct sFlags FLAG;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void CAN_SortAndHandle(CAN_HandleTypeDef *hcan, uint8_t FIFO);
void TransmitPacket();
void PacketLengthFormatter();
void ConnectingInternet();
void ConnectingMQTT();
void ResetXbee_Soft();
void ResetXbee_Hard();

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
  //Default flag states during startup
  FLAG.connected_net = false;
  FLAG.connected_mqtt = false;
  FLAG.transmit_ready = false;
  FLAG.xbee_fr = false;
  FLAG.at_ok = false;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  int a = 0; //Used to make debug mode work

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //Initial pointer to first End Of Array
  id_array[0] = &rx_transmit[last_element+1];
  HAL_Delay(30000);//Delay to wait for the xbee module to boot up and connect to network
  HAL_UARTEx_ReceiveToIdle_IT(&huart4, uart_rx, sizeof(uart_rx));
  //Starting CAN1 and CAN2 interrupts
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(!FLAG.connected_net)
		  ConnectingInternet();
	  if(!FLAG.connected_mqtt)
		  ConnectingMQTT();
	  if(FLAG.transmit_ready)
		  TransmitPacket();
	  if(FLAG.xbee_fr)
		  ResetXbee_Hard();
	  __WFI();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	CAN_FilterTypeDef sFilterConfig;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
  HAL_CAN_Start(&hcan1);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */
	CAN_FilterTypeDef sFilterConfig;
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  sFilterConfig.FilterBank = 14;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
  HAL_CAN_Start(&hcan2);
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 45000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /*if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	  Error_Handler();*/
  TIM2->CNT = 0;
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 230400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Red_Pin|Yellow_Pin|Green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Red_Pin Yellow_Pin Green_Pin */
  GPIO_InitStruct.Pin = Red_Pin|Yellow_Pin|Green_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_SortAndHandle(hcan, CAN_FILTER_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_SortAndHandle(hcan, CAN_FILTER_FIFO1);
}
//Magic function that takes received CAN messages and formats them to be put into a MQTT message
void CAN_SortAndHandle(CAN_HandleTypeDef *hcan, uint8_t FIFO)
{
	HAL_CAN_DeactivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_DeactivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
	uint8_t rx_buffer[8];
	//Get received CAN message, check if ok, and check if DLC is greater than zero
	if (HAL_CAN_GetRxMessage(hcan, FIFO, &RxHeader, rx_buffer) == HAL_OK && RxHeader.DLC)
	{
    bool id_found = false; // Indicates if the ID pre-exists or not
    static uint8_t id_count = 0;
    for (int i = 0; i <= id_count && !id_found; i++) //Scan array for ID matching the incoming ID
    {
      if (RxHeader.StdId == (id_array[i][0] << 8 | id_array[i][1]))
      {
        //If ID is matched we insert the new data into the correct array placement and toggle the boolean idFinder status to positive
        memcpy(id_array[i] + 3, rx_buffer, RxHeader.DLC);
        id_found = true;
      }
    }
    //if ID is not matched and we have not run out of space, then we add the new ID to the array
    if (!id_found)
    {
      //Add a pointer to location of the ID and increment id_count
      id_array[id_count++] = &rx_transmit[last_element+1];
      //First copy the ID
      rx_transmit[++last_element] = RxHeader.StdId >> 8;
      rx_transmit[++last_element] = RxHeader.StdId & 0xFF;
      //Then DLC
      rx_transmit[++last_element] = RxHeader.DLC;
      //And then all the data from the received frame
      memcpy(&rx_transmit[++last_element], rx_buffer, RxHeader.DLC);
      //Updating End Of Array value
      last_element += RxHeader.DLC-1;
      PacketLengthFormatter();
    }
  }
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);

}
//Formats packet length part of MQTT message, it works in a funky way. Google it
void PacketLengthFormatter()
{
	uint8_t array_length[2];
	uint16_t mqtt_length = last_element - 2;
	if(mqtt_length > 127)
	{
		array_length[0] = mqtt_length & 0x7F;
		array_length[0] += 0x80;
		array_length[1] = mqtt_length >> 7 & 0x7F;
	}
	else{
		array_length[0] = mqtt_length + 0x80;
		array_length[1] = 0;
	}
	rx_transmit[1] = array_length[0];
	rx_transmit[2] = array_length[1];

}
//MQTT message transmit function
void TransmitPacket()
{
	HAL_CAN_DeactivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_DeactivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_UART_Transmit_IT(&huart4, rx_transmit, last_element+1);
	FLAG.transmit_ready = false;
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
	TIM2->CNT = 0; //Resets TIM2 counter
}
//Handles received AT commands and MQTT replies from the Xbee module
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	switch(Size)
	{
	case 4:
		if(!memcmp(uart_rx, MQTT_PUBACK, sizeof(MQTT_PUBACK))){
			FLAG.transmit_ready = true;
			timeout_count = 0;
		}
		else if(!memcmp(uart_rx, MQTT_CONACK, sizeof(MQTT_CONACK))){
			FLAG.connected_mqtt = true;
			FLAG.transmit_ready = true;
		}
		FLAG.at_ok = false;
		memset(uart_rx, 0x00, sizeof(uart_rx));
		HAL_UARTEx_ReceiveToIdle_IT(&huart4, uart_rx, sizeof(uart_rx));
		break;
	case 3:
		if(!memcmp(uart_rx, AT_OK, sizeof(AT_OK)))
			FLAG.at_ok = true;
		else
			FLAG.at_ok = false;
		memset(uart_rx, 0x00, sizeof(uart_rx));
		HAL_UARTEx_ReceiveToIdle_IT(&huart4, uart_rx, sizeof(uart_rx));
		break;
	case 2:
		if(!memcmp(uart_rx, AT_CONNECTED, sizeof(AT_CONNECTED)))
			FLAG.connected_net = true;
		FLAG.at_ok = false;
		memset(uart_rx, 0x00, sizeof(uart_rx));
		HAL_UARTEx_ReceiveToIdle_IT(&huart4, uart_rx, sizeof(uart_rx));
		break;
	case 1:
		FLAG.at_ok = false;
		memset(uart_rx, 0x00, sizeof(uart_rx));
		HAL_UARTEx_ReceiveToIdle_IT(&huart4, uart_rx, sizeof(uart_rx));
		break;
	default:
		break;
	}

}
//Function that checks if internet connection is present
void ConnectingInternet()
{
	int count = 0;
	if(FLAG.xbee_fr == true)
		return; //Returns function if reset flag is true, or else it will be stuck in here for a while
	while(!FLAG.connected_net)
	{
		HAL_UART_Transmit_IT(&huart4, AT_ENTER,sizeof(AT_ENTER));
		while(!FLAG.at_ok)
			__WFI();
		HAL_UART_Transmit_IT(&huart4, ATAI,sizeof(ATAI));
		HAL_Delay(500);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
		HAL_Delay(500);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
		count++;
		if(count>5)//If over five AT AI messages has been sent without returning a connected value, there will be a 10 second delay until the next transmit
			HAL_Delay(10000);
		if(count>10){
			FLAG.xbee_fr = true;//If over 10 AT AI messages have been sent without a connected value, the xbee will be reset
			count = 0;
			return;
		}
	}
}
//Function to connect to the MQTT server
void ConnectingMQTT()
{
	int count = 0;
	if(FLAG.xbee_fr == true)
		return; //Returns function if reset flag is true, or else it will be stuck in here for a while
	while(!FLAG.connected_mqtt)
	{
	HAL_UART_Transmit_IT(&huart4, mqtt_connect,sizeof(mqtt_connect));
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	count++;
	if(count==4)//If over four MQTT connect messages has been sent without returning a connected value, there will be a 10 second delay until the next transmit
		HAL_Delay(10000);
	if(count>10){
		FLAG.xbee_fr = true;//If over 10 MQTT connect messages have been sent without a connected value, the xbee will be reset
		count = 0;
		return;
		}
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	  Error_Handler();
}
//TIM2 timer, coded to trigger every 10 seconds, but it only starts after the ConnectingMQTT is successfull, and the timer resets every time a new message is transmitted to the MQTT server
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//If no message acknowlegement message has been received for the past 10 seconds, the transmit_ready flag will be set to true and the telemetry board will try to transmit a new message
	timeout_count++;
	if(timeout_count < 4)
		FLAG.transmit_ready = true;
	else if (timeout_count > 4)
	{//If the telemetry board has tried to transmit messages more than 4 times with no acknowlegement message received, it will reset the xbee module
		FLAG.xbee_fr = true;
		timeout_count = 0;
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		if(HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK)
		  Error_Handler();

	}
}
//Soft reset of Xbee module, basically just resets the modem part of the Xbee and not the whole unit
void ResetXbee_Soft(void)
{
	HAL_UART_Transmit_IT(&huart4, AT_ENTER,sizeof(AT_ENTER));
	while(!FLAG.at_ok)
		__WFI();
	FLAG.at_ok = false;
	HAL_UART_Transmit_IT(&huart4, ATSD_R,sizeof(ATSD_R));
	while(!FLAG.at_ok)
		__WFI();
	FLAG.at_ok = false;
	FLAG.xbee_fr = false;
	HAL_Delay(30000);
}
//Hard reset of Xbee module, resets the entire device
void ResetXbee_Hard(void)
{
	FLAG.connected_mqtt = false;
	FLAG.connected_net = false;
	FLAG.transmit_ready = false;
	HAL_UART_Transmit_IT(&huart4, AT_ENTER,sizeof(AT_ENTER));
	while(!FLAG.at_ok)
		__WFI();
	FLAG.at_ok = false;
	HAL_UART_Transmit_IT(&huart4, ATSD_S,sizeof(ATSD_S));
	while(!FLAG.at_ok)
		__WFI();
	FLAG.at_ok = false;
	HAL_UART_Transmit_IT(&huart4, AT_ENTER,sizeof(AT_ENTER));
	while(!FLAG.at_ok)
		__WFI();
	FLAG.at_ok = false;
	HAL_UART_Transmit_IT(&huart4, ATFR,sizeof(ATFR));
	while(!FLAG.at_ok)
		__WFI();
	FLAG.at_ok = false;
	FLAG.xbee_fr = false;
	HAL_Delay(30000);
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
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	  HAL_Delay(300);
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
