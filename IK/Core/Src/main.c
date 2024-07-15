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
#include <ISO646.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SDC_CLOSE() HAL_GPIO_WritePin(AS_Close_SDC_GPIO_Port,AS_Close_SDC_Pin,GPIO_PIN_SET);
#define SDC_OPEN() HAL_GPIO_WritePin(AS_Close_SDC_GPIO_Port,AS_Close_SDC_Pin,GPIO_PIN_RESET);
#define BRAKE_LIGHT_ON() HAL_GPIO_WritePin(BRAKE_LIGHT_SIGNAL_GPIO_Port,BRAKE_LIGHT_SIGNAL_Pin,GPIO_PIN_SET);
#define BRAKE_LIGHT_OFF() HAL_GPIO_WritePin(BRAKE_LIGHT_SIGNAL_GPIO_Port,BRAKE_LIGHT_SIGNAL_Pin,GPIO_PIN_RESET);

#define WD_READY_FALL_TIME 100  	// Time during which WD_is_ready is expected to fall, ms
#define WD_READY_RISE_TIME 100	 	// Time during which WD_is_ready is expected to rise, ms

#define STORAGE_PNEUMO_MIN_PRESSURE_BP1		6.5 	// bar
#define STORAGE_PNEUMO_MIN_PRESSURE_BP2 	5.5 	// bar
#define STORAGE_PNEUMO_MAX_PRESSURE_BP1 	8.5 	// bar
#define STORAGE_PNEUMO_MAX_PRESSURE_BP2 	6.5 	// bar
#define HYDRO_MIN_PRESSURE								10   	// bar
/* 
The pressure sensor cannot output 0 during operation. 
If the sensor outputs a value of 0, then it is defective. 
Therefore we must enter some minimum value PRESSURE_MIN. This value is found empirically.
If the value coming from the sensor is less than PRESSURE_MIN, then we assume that the sensor outputs zero.
We also enter the PRESSURE_ZERO value. 
If the value coming from the sensor is less than PRESSURE_ZERO, the sensor does not work.
*/
#define PNEUMO_MIN_PRESSURE 		1 	// bar
//#define PNEUMO_ZERO_PRESSURE		3
#define PRESSURE_MAX_DIFFERENCE 410 // Need found

#define TIME_FOR_SENSORS_CHECK 	100

#define DAC_PRESSURE_MIN 0x0
#define DAC_PRESSURE_MAX 0xFFF
#define CAN_PRESSURE_MIN 0x0
#define CAN_PRESSURE_MAX 0xFFFF
#define PNEUMO_BAR_PRESSURE_MIN 	0
#define PNEUMO_BAR_PRESSURE_MAX 	6
#define HYDRO_BAR_PRESSURE_MIN 		0
#define HYDRO_BAR_PRESSURE_MAX 		100

#define PI 3.141592

#define Alpha_BrakePreshure 0.2

#define DEBUG 0

#if (DEBUG == 0)
#define MEDIAN_ORDER 10
#endif

uint8_t SDC_STATUS = 1;
uint8_t BRAKE_LIGHT_STATE = 0;
volatile uint16_t adc[2] = {0,}; // у нас два канала поэтому массив из двух элементов
bool ADC_flag = 0;
float VAL_BrakeSensFront_f,VAL_BrakeSensRear_f = 0;
uint16_t ADC_BrakeSensFront,ADC_BrakeSensFront_prev,ADC_BrakeSensFront_f = 0;
uint16_t ADC_BrakeSensRear,ADC_BrakeSensRear_prev,ADC_BrakeSensRear_f = 0;
uint16_t indeX = 4095;

/*--------иницилизация CAN--------*/
CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t TxMailbox;
uint8_t Rx = 0;
float recieve = 0;
uint8_t TX_data[8], RX_data[8];
CAN_FilterTypeDef sFilterConfig;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim4;

osThreadId readSensorsHandle;
osThreadId supervisorHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_CAN2_Init(void);
static void MX_DAC_Init(void);
void StartReadSensorsTsk(void const * argument);
void supervisorTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float exponential_filter(float alpha, float currentValue, float previousValue){ //функция экспоненциального фильтра
	return alpha * currentValue + (1.0f - alpha) * previousValue;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){ //прерывание по окончанию преобразования АЦП
    if(hadc->Instance == ADC1)
    {
			HAL_ADC_Stop(&hadc1);
      ADC_flag = 1;
    }
}

void BrakeSensors(){ //считывание и фильтрация сигнала с датчиков давления гидравлики
	if(ADC_flag)
  {
    ADC_flag = 0;
		ADC_BrakeSensFront = adc[0];
		ADC_BrakeSensRear = adc[1];
		adc[0] = 0;
    adc[1] = 0;
		
		/*-----------Exponential filter-----------*/
		ADC_BrakeSensFront_f = exponential_filter(0.2f,ADC_BrakeSensFront,ADC_BrakeSensFront_prev);
		ADC_BrakeSensRear_f = exponential_filter(0.2f,ADC_BrakeSensRear,ADC_BrakeSensRear_prev);
		ADC_BrakeSensFront_prev = ADC_BrakeSensFront;
		ADC_BrakeSensRear_prev = ADC_BrakeSensRear;
		
		/*-----------ADC -> Bar-----------*/
		VAL_BrakeSensFront_f = (ADC_BrakeSensFront_f * 100.0f)/4095.0f;
		VAL_BrakeSensRear_f = (ADC_BrakeSensRear_f * 100.0f)/4095.0f;
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 2); // стартуем АЦП
  }
}

void BRAKE_LIGHT_FUNCTION(){ //функция свечения стопаря в зависимости от давления в гидравлике
	if(ADC_BrakeSensRear_f >= 15) {BRAKE_LIGHT_ON();}
	else if(ADC_BrakeSensRear_f <= 5) {BRAKE_LIGHT_OFF();}
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
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_CAN2_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	
  HAL_TIM_PWM_Start(&htim4, Watchdog_Pin); //тактирование Watchdog
	
  HAL_ADCEx_Calibration_Start(&hadc1); //калибруем АЦП
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 2); // стартуем АЦП
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of readSensors */
  osThreadDef(readSensors, StartReadSensorsTsk, osPriorityNormal, 0, 128);
  readSensorsHandle = osThreadCreate(osThread(readSensors), NULL);

  /* definition and creation of supervisor */
  osThreadDef(supervisor, supervisorTask, osPriorityAboveNormal, 0, 128);
  supervisorHandle = osThreadCreate(osThread(supervisor), NULL);

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
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
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
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	pTxHeader.DLC = 5; //Колличество передаваемых бит
pTxHeader.IDE = CAN_ID_STD;
pTxHeader.RTR = CAN_RTR_DATA;
pTxHeader.StdId = 0x302; //Nдентификатор
pTxHeader.TransmitGlobalTime = DISABLE;
//
sFilterConfig.FilterBank = 0;
sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST; //Режим работы: IDMASK/IDLIST
sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
sFilterConfig.SlaveStartFilterBank = 0;
sFilterConfig.FilterIdHigh = 0x100<<5;
sFilterConfig.FilterIdLow = 0x100<<5;
sFilterConfig.FilterMaskIdHigh = 0x100<<5;
sFilterConfig.FilterMaskIdLow = 0x100<<5;
sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
sFilterConfig.FilterActivation = ENABLE;

if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig)!= HAL_OK)
{
// Filter configuration Error
Error_Handler();
}

if(HAL_CAN_Start(&hcan2)!= HAL_OK)
{
//Start Error
Error_Handler();
}
/* Activate CAN RX notification */
if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
{
/* Notification Error */
Error_Handler();
}

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BRAKE_LIGHT_SIGNAL_GPIO_Port, BRAKE_LIGHT_SIGNAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AS_Close_SDC_GPIO_Port, AS_Close_SDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BRAKE_LIGHT_SIGNAL_Pin */
  GPIO_InitStruct.Pin = BRAKE_LIGHT_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BRAKE_LIGHT_SIGNAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WD_is_ready_Pin */
  GPIO_InitStruct.Pin = WD_is_ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WD_is_ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AS_Close_SDC_Pin */
  GPIO_InitStruct.Pin = AS_Close_SDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(AS_Close_SDC_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, RX_data);
 if(pRxHeader.StdId == 0x100){
 //*(uint32_t*)&PWM_SET = *((uint32_t*)&RX_data[0]);
 Rx = RX_data[0];
 //DIR = RX_data[4];
 }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartReadSensorsTsk */
/**
  * @brief  Function implementing the readSensors thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartReadSensorsTsk */
void StartReadSensorsTsk(void const * argument)
{
  /* USER CODE BEGIN 5 */
	
  /* Infinite loop */
  for(;;)
  {
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, indeX);
		if(Rx == 0) SDC_OPEN();
		if(Rx == 1) SDC_CLOSE();
		
//		if(BRAKE_LIGHT_STATE == 0) BRAKE_LIGHT_OFF();
//		if(BRAKE_LIGHT_STATE == 1) BRAKE_LIGHT_ON();
		
		BrakeSensors();
		BRAKE_LIGHT_FUNCTION();
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_supervisorTask */
/**
* @brief Function implementing the supervisor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_supervisorTask */
void supervisorTask(void const * argument)
{
  /* USER CODE BEGIN supervisorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END supervisorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
