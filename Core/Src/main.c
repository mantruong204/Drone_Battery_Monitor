/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* R1 resistance */
#define NTC_UP_R 12000.0f

/* constants of Steinhart-Hart equation */
#define A 0.0009053128946f
#define B 0.0002531319168f
#define C 0.0000001500959159f

#define ratio_VSS  0.1885964912f

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

osThreadId ActuatorTaskHandle;
osThreadId TxtoESPTaskHandle;
osThreadId getBattTaskHandle;
osThreadId SwitchTaskHandle;
osThreadId OLEDTaskHandle;
osMessageQId Queue_Temp_ESPHandle;
osMessageQId Queue_Temp_OLEDHandle;
osMessageQId Queue_Volt_ESPHandle;
osMessageQId Queue_Volt_OLEDHandle;
osMessageQId Queue_Volt_ActuatorHandle;
osMessageQId Queue_Curr_ESPHandle;
osMessageQId Queue_Curr_OLEDHandle;
/* USER CODE BEGIN PV */

/********************************************* ntc vars *******************************************/
float NTC_temp;
uint16_t Ntc_R;
float get_NTC_temp (uint32_t variable)
{
	/* calc. ntc resistance */
	Ntc_R = ((NTC_UP_R)/((4095.0/variable) - 1));

	/* temp */
	float Ntc_Ln = log(Ntc_R);
	/* calc. temperature */
	return ((1.0/(A + B*Ntc_Ln + C*Ntc_Ln*Ntc_Ln*Ntc_Ln)) - 273.15);
}
/**************************************************************************************************/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
void Actuator_Task(void const * argument);
void Tx_toESP_Task(void const * argument);
void getBatt_info_Task(void const * argument);
void Switch_Task(void const * argument);
void OLED_Task(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define NUM_CHANNELS 3
#define ADC_BUFFER_SIZE NUM_CHANNELS
uint16_t adc_value[ADC_BUFFER_SIZE];  // to store the adc values

#define MAX_RxLEN 128
#define MAX_TxLEN 8
uint8_t nRxData[MAX_RxLEN];
static uint8_t nTxData[MAX_TxLEN];
uint8_t Rx_buf_write_index = 0;

uint8_t nRxData3[3];

static uint8_t Tx_STX[]={0x02U};
static uint8_t Tx_ETX[]={0x03U};

static char rx_char;

static uint8_t Rx_ONRL[]={0x55U,0x01U,0xAAU};
static uint8_t Rx_OFRL[]={0x55U,0x00U,0xAAU};

//static uint8_t   ONRL[]={0x01};
//static uint8_t   OFRL[]={0x00};
bool Relay_ON_cmd = false;

static bool WriteComm(uint8_t *pBuff, uint8_t nSize)
{
	return HAL_UART_Transmit(&huart3, pBuff, nSize, 1);
}

//static bool ReadComm(uint8_t *pBuff, uint8_t nSize)
//{
//	if ((pBuff[0] == Rx_STX[0]) && (pBuff[2] == Rx_ETX[0]))
//	{
//		if (pBuff[1] == ONRL[0])
//			Relay_ON_cmd = false;
//		else if (pBuff[1] == OFRL[0])
//			Relay_ON_cmd = true;
//
//		return true;
//	}
//	else
//		return false;
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart3.Instance)
	{
//		  nRxData[Rx_buf_write_index] = rx_char;

//		  if (nRxData[Rx_buf_write_index] == Rx_ETX[0] || nRxData[Rx_buf_write_index-2] == Rx_STX[0] )
//			  if (nRxData[Rx_buf_write_index-1] == ONRL[1])
//				  Relay_ON_cmd = true;
//			  else if (nRxData[Rx_buf_write_index-1] == OFRL[1])
//				  Relay_ON_cmd = false;



//		  Rx_buf_write_index ++;
//		  Rx_buf_write_index = Rx_buf_write_index % MAX_RxLEN;

//		HAL_UART_Receive_IT(&huart3, (uint8_t *)nRxData, 3);


//				  if (memcmp(nRxData3,Rx_OFRL,3) == 1)
//					  Relay_ON_cmd = false;
//				  else if (memcmp(nRxData3,Rx_ONRL,3)==1)
//					  Relay_ON_cmd = true;

				  if (nRxData3[0] == Rx_OFRL[0] &&
					  nRxData3[1] == Rx_OFRL[1] &&
					  nRxData3[2] == Rx_OFRL[2])
					  Relay_ON_cmd = false;
				  else if (nRxData3[0] == Rx_ONRL[0] &&
					  nRxData3[1] == Rx_ONRL[1] &&
					  nRxData3[2] == Rx_ONRL[2])
					  Relay_ON_cmd = true;
	}
}

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  display_Bat_InitInfo();

  UARTStdioConfig(USART1,USART1_IRQn, true);
  UARTprintf("***UARTprintf configured***\n");

//  HAL_ADC_Start_DMA(&hadc1, adc_value, NUM_CHANNELS);

  HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, 0);
  HAL_GPIO_WritePin(LD_GPIO_Port, LD_Pin, 0);
  HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 0);

  HAL_UART_Receive_IT(&huart3, (uint8_t *)nRxData, 3);

  /* USER CODE END 2 */

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
  /* definition and creation of Queue_Temp_ESP */
  osMessageQDef(Queue_Temp_ESP, 16, uint16_t);
  Queue_Temp_ESPHandle = osMessageCreate(osMessageQ(Queue_Temp_ESP), NULL);

  /* definition and creation of Queue_Temp_OLED */
  osMessageQDef(Queue_Temp_OLED, 16, uint16_t);
  Queue_Temp_OLEDHandle = osMessageCreate(osMessageQ(Queue_Temp_OLED), NULL);

  /* definition and creation of Queue_Volt_ESP */
  osMessageQDef(Queue_Volt_ESP, 16, uint16_t);
  Queue_Volt_ESPHandle = osMessageCreate(osMessageQ(Queue_Volt_ESP), NULL);

  /* definition and creation of Queue_Volt_OLED */
  osMessageQDef(Queue_Volt_OLED, 16, uint16_t);
  Queue_Volt_OLEDHandle = osMessageCreate(osMessageQ(Queue_Volt_OLED), NULL);

  /* definition and creation of Queue_Volt_Actuator */
  osMessageQDef(Queue_Volt_Actuator, 16, uint16_t);
  Queue_Volt_ActuatorHandle = osMessageCreate(osMessageQ(Queue_Volt_Actuator), NULL);

  /* definition and creation of Queue_Curr_ESP */
  osMessageQDef(Queue_Curr_ESP, 16, uint16_t);
  Queue_Curr_ESPHandle = osMessageCreate(osMessageQ(Queue_Curr_ESP), NULL);

  /* definition and creation of Queue_Curr_OLED */
  osMessageQDef(Queue_Curr_OLED, 16, uint16_t);
  Queue_Curr_OLEDHandle = osMessageCreate(osMessageQ(Queue_Curr_OLED), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ActuatorTask */
  osThreadDef(ActuatorTask, Actuator_Task, osPriorityNormal, 0, 128);
  ActuatorTaskHandle = osThreadCreate(osThread(ActuatorTask), NULL);

  /* definition and creation of TxtoESPTask */
  osThreadDef(TxtoESPTask, Tx_toESP_Task, osPriorityNormal, 0, 128);
  TxtoESPTaskHandle = osThreadCreate(osThread(TxtoESPTask), NULL);

  /* definition and creation of getBattTask */
  osThreadDef(getBattTask, getBatt_info_Task, osPriorityNormal, 0, 128);
  getBattTaskHandle = osThreadCreate(osThread(getBattTask), NULL);

  /* definition and creation of SwitchTask */
  osThreadDef(SwitchTask, Switch_Task, osPriorityNormal, 0, 128);
  SwitchTaskHandle = osThreadCreate(osThread(SwitchTask), NULL);

  /* definition and creation of OLEDTask */
  osThreadDef(OLEDTask, OLED_Task, osPriorityNormal, 0, 128);
  OLEDTaskHandle = osThreadCreate(osThread(OLEDTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB6   ------> USART1_TX
  PB7   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_GPIO_AF_EnableRemap_USART1();

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD_Pin|Buzz_Pin|Relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SW_GPIO_Port, SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD_Pin Buzz_Pin Relay_Pin */
  GPIO_InitStruct.Pin = LD_Pin|Buzz_Pin|Relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Actuator_Task */
/**
  * @brief  Function implementing the ActuatorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Actuator_Task */
void Actuator_Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(LD_GPIO_Port, LD_Pin, 0);
	  HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 0);

//	  HAL_UART_Receive_IT(&huart3, &rx_char, 1);

//	  if (HAL_UART_Receive_IT(&huart3, &nRxData[Rx_buf_write_index], 3)==HAL_OK)
//		  Rx_buf_write_index = (Rx_buf_write_index + 3) % MAX_RxLEN;

//	  if (HAL_UART_Receive(&huart3, nRxData3, 3, 5)==HAL_OK)
//		  if (memcmp(nRxData3,Rx_OFRL,3) == 1)
//			  Relay_ON_cmd = false;
//		  else if (memcmp(nRxData3,Rx_ONRL,3)==1)
//			  Relay_ON_cmd = true;

	  HAL_UART_Receive_IT(&huart3, (uint8_t *)nRxData3, 3);

	  uint16_t Volt;
		xQueueReceive(Queue_Volt_ActuatorHandle, &Volt, 3); //get volt
		if (Volt < 1024)
		{
			HAL_GPIO_TogglePin(LD_GPIO_Port, LD_Pin);
			HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 1);
			HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, 0);
			UARTprintf("Actuator_Task: LOW POWER !!!\n");
		}
		else if (Relay_ON_cmd){
			HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, 1);
			UARTprintf("Actuator_Task: Relay ON\n");
		}
		else{
			HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, 0);
			UARTprintf("Actuator_Task: Relay OFF\n");
		}

//	  HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, 1);

		osDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Tx_toESP_Task */
/**
* @brief Function implementing the TxtoESPTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Tx_toESP_Task */
void Tx_toESP_Task(void const * argument)
{
  /* USER CODE BEGIN Tx_toESP_Task */
  /* Infinite loop */
  for(;;)
  {
		nTxData[0] = Tx_STX[0];
		uint16_t Temp;
		xQueueReceive(Queue_Temp_ESPHandle, &Temp, 3);
		nTxData[1] = (Temp>>8)&(0xFF);
		nTxData[2] = (Temp>>0)&(0xFF);

		uint16_t Volt;
		xQueueReceive(Queue_Volt_ESPHandle, &Volt, 3);
		nTxData[3] = (Volt>>8)&(0xFF);
		nTxData[4] = (Volt>>0)&(0xFF);

		uint16_t Curr;
		xQueueReceive(Queue_Curr_ESPHandle, &Curr, 3);
		nTxData[5] = (Curr>>8)&(0xFF);
		nTxData[6] = (Curr>>0)&(0xFF);


		nTxData[7] = Tx_ETX[0];

		taskENTER_CRITICAL();
		WriteComm(nTxData, MAX_TxLEN);
		taskEXIT_CRITICAL();


    osDelay(50);
  }
  /* USER CODE END Tx_toESP_Task */
}

/* USER CODE BEGIN Header_getBatt_info_Task */
/**
* @brief Function implementing the getBattTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_getBatt_info_Task */
void getBatt_info_Task(void const * argument)
{
  /* USER CODE BEGIN getBatt_info_Task */
  /* Infinite loop */
  for(;;)
  {
		while (HAL_ADC_Start_DMA(&hadc1, adc_value, NUM_CHANNELS)!=HAL_OK);
//		UARTprintf("ADC_Task: Got Temp.s\n");


		xQueueSend(Queue_Temp_ESPHandle, &adc_value[0], 3);
		xQueueSend(Queue_Volt_ESPHandle, &adc_value[1], 3);
		xQueueSend(Queue_Curr_ESPHandle, &adc_value[2], 3);

		xQueueSend(Queue_Temp_OLEDHandle, &adc_value[0], 3);
		xQueueSend(Queue_Volt_OLEDHandle, &adc_value[1], 3);
		xQueueSend(Queue_Curr_OLEDHandle, &adc_value[2], 3);

		xQueueSend(Queue_Volt_ActuatorHandle, &adc_value[1], 3);

    osDelay(200);
  }
  /* USER CODE END getBatt_info_Task */
}

/* USER CODE BEGIN Header_Switch_Task */
/**
* @brief Function implementing the SwitchTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Switch_Task */
void Switch_Task(void const * argument)
{
  /* USER CODE BEGIN Switch_Task */
  /* Infinite loop */
  for(;;)
  {
		if (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin)==1) {
			UARTprintf("SW_Task: Button Pressed\n");
//			HAL_GPIO_TogglePin(Relay_GPIO_Port, Relay_Pin);
		}
		else {
			UARTprintf("SW_Task: Button Released\n");
		}
    osDelay(50);
  }
  /* USER CODE END Switch_Task */
}

/* USER CODE BEGIN Header_OLED_Task */
/**
* @brief Function implementing the OLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLED_Task */
void OLED_Task(void const * argument)
{
  /* USER CODE BEGIN OLED_Task */

	float Temp_val;
	float Volt_val;
	float Curr_val;
	float Level_val;
	uint16_t Temp;
	uint16_t Volt;
	uint16_t Curr;
	char Temp_str[6];
	char Volt_str[6];
	char Curr_str[6];
	char Level_str[6];
  /* Infinite loop */
  for(;;)
  {


		initBff(Temp_str, 4);
		initBff(Volt_str, 4);
		initBff(Curr_str, 4);
		initBff(Level_str, 4);

		xQueueReceive(Queue_Temp_OLEDHandle, &Temp, 3);//get temp

		xQueueReceive(Queue_Volt_OLEDHandle, &Volt, 3);//get volt

		xQueueReceive(Queue_Curr_OLEDHandle, &Curr, 3);//get current



		Temp_val = get_NTC_temp(Temp);
		gcvt(Temp_val, 4, Temp_str);

		Volt_val = (float)((Volt*3.3/4020.0)/ratio_VSS);
		Volt_val = (Volt_val < 3.0) ? 0 : Volt_val;
		gcvt(Volt_val, 5, Volt_str);

		Level_val = (float)(71.35*((Volt*3.3/4020.0)/ratio_VSS) - 494.9);
		Level_val = (Level_val > 100.0) ? 100.0 : ((Level_val < 0.0) ? 0.00 : Level_val);
		gcvt(Level_val, 6, Level_str);


		Curr_val = (float)((Curr/4020.0*3.3)*4.2083 - 11.022);
//		Curr_val = (float)(Curr/4020.0*3.3);
		Curr_val = (Volt_val ==0) ? 0.0000 : Curr_val;
		gcvt(Curr_val, 5, Curr_str);


//		taskENTER_CRITICAL();
		display_Bat_Info(Level_str, Volt_str, Curr_str, Temp_str);
//		display_Bat_Info("100.0", "8.33", "2.67", "40.7");
//		taskEXIT_CRITICAL();


    osDelay(150);
  }
  /* USER CODE END OLED_Task */
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
