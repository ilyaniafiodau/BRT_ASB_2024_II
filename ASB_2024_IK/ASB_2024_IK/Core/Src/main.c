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
typedef enum {
	ASB_RESET,                           
	MANUAL_DRIVING_MODE,							
	INIT_CHECK_REQUEST,
	GO_SIGNAL,                          //	unused, yet
	EXT_EMERGENCY,
	EMERGENCY_HANDLER_ERROR,              // error that occurs when the Watchdog wasn't stopped
	EMERGENCY_HANDLER_COMPLETE,
	MONITORING_FINISHED,
	MONITORING_ERROR,
	INIT_CHECK_ERROR,
	INIT_CHECK_COMPLETE,
} ASBSignal_TypeDef; 
ASBSignal_TypeDef ASBSignal;

typedef enum {
	NO_ERRORS = 0,
	ENERGY_STORAGE_NOT_FILLED,
	OVERPRESSURE_OR_IMPAUSIBILITY,
	DRIVING_MODE_ERROR,
	BRAKES_NOT_WORKING,
	EBS_ENERGY_STORAGE_ERROR,
	UNEXPECTED_SIGNAL_ERROR,
} ErrorFlag_TypeDef;
ErrorFlag_TypeDef ErrorFlag = NO_ERRORS;

typedef enum {
	AS_OFF = 0,
	INIT_CHECK,
	READY,
	DRIVING,
	EMERGENCY,
	SAFE_STATE,
} ASBState_TypeDef;
ASBState_TypeDef ASBState = AS_OFF;

// status(OFF=0x0, MANUAL=0x1, READY=0x2, DRIVING=0x3, EMERGENCY=0x4)
typedef enum {
	_OFF = 0,
	_MANUAL,
	_READY,
	_DRIVING,
	_EMERGENCY,
} ASBStatus_TypeDef;
ASBStatus_TypeDef ASBStatus = _OFF; 

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SDC_CLOSE() HAL_GPIO_WritePin(AS_Close_SDC_GPIO_Port,AS_Close_SDC_Pin,GPIO_PIN_SET);
#define SDC_OPEN() HAL_GPIO_WritePin(AS_Close_SDC_GPIO_Port,AS_Close_SDC_Pin,GPIO_PIN_RESET);
#define BRAKE_LIGHT_ON() HAL_GPIO_WritePin(BRAKE_LIGHT_SIGNAL_GPIO_Port,BRAKE_LIGHT_SIGNAL_Pin,GPIO_PIN_SET);
#define BRAKE_LIGHT_OFF() HAL_GPIO_WritePin(BRAKE_LIGHT_SIGNAL_GPIO_Port,BRAKE_LIGHT_SIGNAL_Pin,GPIO_PIN_RESET);

#define EBS_QM1_ON() HAL_GPIO_WritePin(EBSActuator_GPIO_Port, EBSActuator_Pin, GPIO_PIN_SET);
#define Redundant_QM2_ON() HAL_GPIO_WritePin(RedundantActuator_GPIO_Port, RedundantActuator_Pin, GPIO_PIN_SET);
#define EBS_QM1_OFF() HAL_GPIO_WritePin(EBSActuator_GPIO_Port, EBSActuator_Pin, GPIO_PIN_RESET);
#define Redundant_QM2_OFF() HAL_GPIO_WritePin(RedundantActuator_GPIO_Port, RedundantActuator_Pin, GPIO_PIN_RESET);

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

#define T 0.001; //sample time (discretization)

#define k 4018.9f //коэффициент для пересчета давления в гидравлике(бар) в тормозной момент(Н*м)

#define DEBUG 0

#if (DEBUG == 0)
#define MEDIAN_ORDER 10
#endif

/*~~~~~~~~~~~~~~~~ADC~~~~~~~~~~~~~~~~*/
volatile uint16_t adc[4] = {0,}; // у нас четыре канала поэтому массив из двух элементов
bool ADC_flag = 0; //флаг окончания преобразования ADC
/*~~~~~~~~~~~~~~~~ADC~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~HydroPressure~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*BrakeSensorFront*/
float VAL_BrakeSensFront, VAL_BrakeSensFront_prev, VAL_BrakeSensFront_f = 0;
int16_t ADC_BrakeSensFront = 0;

/*BrakeSensorRear*/
float VAL_BrakeSensRear, VAL_BrakeSensRear_prev, VAL_BrakeSensRear_f = 0;
int16_t ADC_BrakeSensRear = 0;

/*BrakeSensorsVariables*/
float VAL_HydroPressure = 0; //VAL_HydroPressure = VAL_BrakeSensRear_f + VAL_BrakeSensFront_f;
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~HydroPressure~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~PneumoPressure~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*Pressure BP1*/
float VAL_Pressure_BP1, VAL_Pressure_BP1_prev, VAL_Pressure_BP1_f = 0;
int16_t ADC_Pressure_BP1 = 0;
/*Pressure BP1*/

/*Pressure BP2*/
float VAL_Pressure_BP2, VAL_Pressure_BP2_prev, VAL_Pressure_BP2_f = 0;
int16_t ADC_Pressure_BP2 = 0;
/*Pressure BP2*/
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~PneumoPressure~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Brake Moment~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
float VAL_MomentBrake  = 0;
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Brake Moment~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~PID value~~~~~~~~~~~~~~~~*/
static float outPIDpne = 0;
static float outPIDhyd = 0;
static float outPIDmoment = 0;
float setPneumoValue = 0;
float setHydroValue = 0;
float setMomentValue = 0;
/*~~~~~~~~~~~~~~~~PID value~~~~~~~~~~~~~~~~*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim4;

osThreadId readSensorsHandle;
osThreadId supervisorHandle;
osThreadId ManageSysTaskHandle;
osThreadId SendCANTaskHandle;
/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t TxMailbox;
uint8_t i = 0;
uint8_t recieve = 0;
float transmit = 0;
uint8_t TX_data[8], RX_data[8];
CAN_FilterTypeDef sFilterConfig;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_DAC_Init(void);
static void MX_CAN1_Init(void);
void StartReadSensorsTsk(void const * argument);
void supervisorTask(void const * argument);
void ManageTask(void const * argument);
void SendControlCANTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float convert(float value, float From1, float From2, float To1, float To2) {
  return ((value - From1) / (From2 - From1) * (To2 - To1) + To1);
}

float exponential_filter(float alpha, float currentValue, float previousValue){ //функция экспоненциального фильтра
	return alpha * currentValue + (1.0f - alpha) * previousValue;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
    {
      adc[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1); //BP 3
      adc[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2); //BP 4
			adc[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3); //BP 1
      adc[3] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4); //BP 2
			ADC_flag = 1;
      HAL_ADCEx_InjectedStart_IT(&hadc1);
    }
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){ //прерывание по окончанию преобразования АЦП
//    if(hadc->Instance == ADC1)
//    {
//      ADC_flag = 1;
//    }
//}

void SensorsRead(){ //считывание и фильтрация сигнала с датчиков давления
	if(ADC_flag)
  {
    ADC_flag = 0;
		ADC_BrakeSensFront = adc[0];
		ADC_BrakeSensRear = adc[1];
		ADC_Pressure_BP1 = adc[2];
		ADC_Pressure_BP2 = adc[3];
		adc[0] = 0;
    adc[1] = 0;
		adc[2] = 0;
		adc[3] = 0;
		
		/*-----------ADC -> Bar for Brake Sensors-----------*/
		VAL_BrakeSensFront = (ADC_BrakeSensFront * 100.0f)/4095.0f;
		VAL_BrakeSensRear = (ADC_BrakeSensRear * 100.0f)/4095.0f;
		
		/*-----------ADC -> Bar for BP1 and BP2-----------*/
//		VAL_Pressure_BP1 = (ADC_Pressure_BP1 * 10.0f)/4095.0f;
//		VAL_Pressure_BP2 = (ADC_Pressure_BP2 * 10.0f)/4095.0f;
		VAL_Pressure_BP1 = convert(ADC_Pressure_BP1, 0, 0xFFF, 0, 10);
		VAL_Pressure_BP2 = convert(ADC_Pressure_BP2, 0, 0xFFF, 0, 10);

		/*-----------Exponential filter for brake sensors-----------*/
		VAL_BrakeSensFront_f = exponential_filter(0.2f, VAL_BrakeSensFront, VAL_BrakeSensFront_prev);
		VAL_BrakeSensRear_f = exponential_filter(0.2f, VAL_BrakeSensRear, VAL_BrakeSensRear_prev);
		VAL_BrakeSensFront_prev = ADC_BrakeSensFront;
		VAL_BrakeSensRear_prev = ADC_BrakeSensRear;
		
		/*-----------Exponential filter for BP1 and BP2-----------*/
		VAL_Pressure_BP1_f = exponential_filter(0.2f, VAL_Pressure_BP1, VAL_Pressure_BP1_prev);
		VAL_Pressure_BP2_f = exponential_filter(0.2f, VAL_Pressure_BP2, VAL_Pressure_BP2_prev);
		VAL_Pressure_BP1_prev = VAL_Pressure_BP1;
    VAL_Pressure_BP2_prev = VAL_Pressure_BP2;		
  }
}

void BRAKE_LIGHT_FUNCTION(){ //функция свечения стопаря в зависимости от давления в гидравлике
	if(VAL_BrakeSensRear_f >= 0.4f) {BRAKE_LIGHT_ON();}
	else if(VAL_BrakeSensRear_f <= 0.1f) {BRAKE_LIGHT_OFF();}
}

float PID_Moment(float setVal, float val, float up_lim, float low_lim, float Kp, float Ki, float Kd) { //PID-регулятор
	static float error_PID, error_PID_prev = 0;
	static float Prop, Int, Dif = 0;
	static float Int_prev = 0;
	static float out_PID = 0;
	/*--------вычисление ошибки регулирования--------*/
	error_PID = setVal - val;

	/*--------Пропорциональная составляющая--------*/
	Prop = Kp * error_PID;

	/*--------интегральная составляющая--------*/
	Int = Int_prev + Ki * error_PID * T;

	/*--------Дифференциальная составляющая--------*/
	Dif = Kd * (error_PID - error_PID_prev) / T;

	/*--------Обновление предыдущих значений--------*/
	error_PID_prev = error_PID;
	Int_prev = Int;

	/*--------Выход PID регулятора--------*/
	out_PID = Prop + Int + Dif;
	out_PID = fminf(fmaxf(out_PID, low_lim), up_lim); //ограничиваем выход PID-регулятора
  return out_PID;
}

float PID_Hydraulic(float setVal, float val, float up_lim, float low_lim, float Kp, float Ki, float Kd) { //PID-регулятор
	static float error_PID, error_PID_prev = 0;
	static float Prop, Int, Dif = 0;
	static float Int_prev = 0;
	static float out_PID = 0;
	/*--------вычисление ошибки регулирования--------*/
	error_PID = setVal - val;

	/*--------Пропорциональная составляющая--------*/
	Prop = Kp * error_PID;

	/*--------интегральная составляющая--------*/
	Int = Int_prev + Ki * error_PID * T;

	/*--------Дифференциальная составляющая--------*/
	Dif = Kd * (error_PID - error_PID_prev) / T;

	/*--------Обновление предыдущих значений--------*/
	error_PID_prev = error_PID;
	Int_prev = Int;

	/*--------Выход PID регулятора--------*/
	out_PID = Prop + Int + Dif;
	out_PID = fminf(fmaxf(out_PID, low_lim), up_lim); //ограничиваем выход PID-регулятора
  return out_PID;
}

float PID_Pneumatic(float setVal, float val, float up_lim, float low_lim, float Kp, float Ki, float Kd) { //PID-регулятор
	static float error_PID, error_PID_prev = 0;
	static float Prop, Int, Dif = 0;
	static float Int_prev = 0;
	static float out_PID = 0;
	/*--------вычисление ошибки регулирования--------*/
	error_PID = setVal - val;

	/*--------Пропорциональная составляющая--------*/
	Prop = Kp * error_PID;

	/*--------интегральная составляющая--------*/
	Int = Int_prev + Ki * error_PID * T;

	/*--------Дифференциальная составляющая--------*/
	Dif = Kd * (error_PID - error_PID_prev) / T;

	/*--------Обновление предыдущих значений--------*/
	error_PID_prev = error_PID;
	Int_prev = Int;

	/*--------Выход PID регулятора--------*/
	out_PID = Prop + Int + Dif;
	out_PID = fminf(fmaxf(out_PID, low_lim), up_lim); //ограничиваем выход PID-регулятора
  return out_PID;
}

void DAC_voltageControl(float PID_output){
	uint16_t setVoltage = 0;
	setVoltage = (PID_output * 4095.0f)/8.0f;
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, setVoltage);
}

void MANUALDrivingModeHandler(void) {
	HAL_TIM_PWM_Start(&htim4, Watchdog_Pin);
	if(HAL_GPIO_ReadPin(WD_is_ready_GPIO_Port, WD_is_ready_Pin) == 1){
		SDC_CLOSE();                             //Enable TS activation through AS_close_SDC
	  BRAKE_LIGHT_FUNCTION();
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
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_DAC_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	SDC_CLOSE();
  HAL_TIM_PWM_Start(&htim4, Watchdog_Pin); //тактирование Watchdog
  HAL_ADCEx_Calibration_Start(&hadc1);     //калибруем АЦП
	HAL_ADCEx_InjectedStart_IT(&hadc1);      //Запуск инжектированных каналов АЦП с прерыанием
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);     //Запуск DAC
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
  osThreadDef(readSensors, StartReadSensorsTsk, osPriorityAboveNormal, 0, 128);
  readSensorsHandle = osThreadCreate(osThread(readSensors), NULL);

  /* definition and creation of supervisor */
  osThreadDef(supervisor, supervisorTask, osPriorityHigh, 0, 128);
  supervisorHandle = osThreadCreate(osThread(supervisor), NULL);

  /* definition and creation of ManageSysTask */
  osThreadDef(ManageSysTask, ManageTask, osPriorityNormal, 0, 256);
  ManageSysTaskHandle = osThreadCreate(osThread(ManageSysTask), NULL);

  /* definition and creation of SendCANTask */
  osThreadDef(SendCANTask, SendControlCANTask, osPriorityNormal, 0, 128);
  SendCANTaskHandle = osThreadCreate(osThread(SendCANTask), NULL);

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

  ADC_InjectionConfTypeDef sConfigInjected = {0};

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
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
pTxHeader.DLC = 8;                          //Колличество передаваемых бит
pTxHeader.IDE = CAN_ID_STD;
pTxHeader.RTR = CAN_RTR_DATA;
pTxHeader.StdId = 0x300;
pTxHeader.TransmitGlobalTime = DISABLE;

sFilterConfig.FilterBank = 0;
sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;   //Режим работы: IDMASK/IDLIST
sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
sFilterConfig.SlaveStartFilterBank = 0;
sFilterConfig.FilterIdHigh = 0x333<<5;
sFilterConfig.FilterIdLow = 0x333<<5;
sFilterConfig.FilterMaskIdHigh = 0x333<<5;
sFilterConfig.FilterMaskIdLow = 0x333<<5;
sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
sFilterConfig.FilterActivation = ENABLE;
  
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig)!= HAL_OK)
    {
      // Filter configuration Error 
      Error_Handler();
    }
  
  if(HAL_CAN_Start(&hcan1)!= HAL_OK)
    {
      //Start Error 
      Error_Handler();
    } 
  /* Activate CAN RX notification */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
   {
     /* Notification Error */
     Error_Handler();
   }
  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BRAKE_LIGHT_SIGNAL_GPIO_Port, BRAKE_LIGHT_SIGNAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EBSActuator_Pin|RedundantActuator_Pin|AS_Close_SDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BRAKE_LIGHT_SIGNAL_Pin */
  GPIO_InitStruct.Pin = BRAKE_LIGHT_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BRAKE_LIGHT_SIGNAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WD_is_ready_Pin */
  GPIO_InitStruct.Pin = WD_is_ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(WD_is_ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EBSActuator_Pin RedundantActuator_Pin AS_Close_SDC_Pin */
  GPIO_InitStruct.Pin = EBSActuator_Pin|RedundantActuator_Pin|AS_Close_SDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, RX_data);
	if(pRxHeader.StdId == 0x300){
		*(uint32_t*)&setMomentValue = *((uint32_t*)&RX_data[0]);
	}
	else if(pRxHeader.StdId == 0x301){
		switch(RX_data[0]){
			
			case 0x00:
				ASBSignal = ASB_RESET;
			break;
			
			case 0x01:
				ASBSignal = MANUAL_DRIVING_MODE;
			break;
			
			case 0x02:
				ASBSignal = INIT_CHECK_REQUEST;
			break;
			
			case 0x03:
				ASBSignal = EXT_EMERGENCY;
			break;
		}
//		if(RX_data[0] == 0x00){
//         currentASBSignal = ASB_Reset;	
//		}
//		if(RX_data[0] == 1){ 
//				 currentASBSignal = Manual_Driving_Mode;
//		}
//		if(RX_data[0] == 2){ 
//				 currentASBSignal = Init_Check_Request;
//		}
//		if (RX_data[0] == 3) {
//				 currentASBSignal = Emergency_External;
//				// emergencyHandler();
//		}
	}
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef*hcan) {
}

void EmergencyHandler(void) {
	ASBSignal_TypeDef someASBSignal;
	
  EBS_QM1_OFF();                                               // Disable EBS actuator (Turn OFF valve QM1)
  
	Redundant_QM2_OFF();                                         // Disable Redundant actuator (Turn OFF valve QM2)
	
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);  // Break pressure filled to Proportional valve
	
	SDC_OPEN();                                                  // AS opened SDC
	
	// Stop toggling watchdog => SDC will be opened
	if (HAL_TIM_PWM_Stop(&htim4, Watchdog_Pin) != HAL_OK) {
		someASBSignal = EMERGENCY_HANDLER_ERROR;
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
		SensorsRead();
		VAL_BrakeSensFront_f = VAL_BrakeSensRear_f * 1.191f;             //т.к. передний не работает
		VAL_HydroPressure = VAL_BrakeSensRear_f + VAL_BrakeSensFront_f;  //суммарное давление в гидравлике
		VAL_MomentBrake = VAL_HydroPressure * k;                         //считаем тормозной момент
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
		switch (ASBState) {
				case AS_OFF:
					switch (ASBSignal) {
						
						case ASB_RESET:
							break;
						
						case EXT_EMERGENCY:
							break;
						
						case MANUAL_DRIVING_MODE:
							break;
						
						case INIT_CHECK_REQUEST:
							break;
						
						default:
							break;
					}
					break;
	
				case INIT_CHECK:
					switch (ASBSignal) {
						
						case EXT_EMERGENCY:
							break;
						
						case INIT_CHECK_ERROR:
							break;
						
						case INIT_CHECK_COMPLETE:
							break;
						
						default:
							break;
					}
					break;
								
				case READY:
					switch (ASBSignal) {
						
						case EXT_EMERGENCY:
							break;
						
						case ASB_RESET:
							break;
						
						case MONITORING_ERROR:
							break;
						
						case GO_SIGNAL:
							break;
						
						default:
							break;
					}
					break;
					
				case DRIVING:
					switch (ASBSignal) {
						case EXT_EMERGENCY:
							break;
						
						case MONITORING_ERROR:
							break;
						
						case MONITORING_FINISHED: // Which actions should be done in AS_FINISHED status?
							break;
						
						default:
							break;
					}
					break;
					
				case EMERGENCY:
					switch (ASBSignal) {
						
						case EXT_EMERGENCY:
							break;
						
						case EMERGENCY_HANDLER_ERROR:
							break;
						
						case EMERGENCY_HANDLER_COMPLETE:
							break;
						
						default:
							break;
					}
					break;
					
				case SAFE_STATE:
					switch (ASBSignal) {
						
						case ASB_RESET:
//							HAL_NVIC_SystemReset();
							break;
						
						default:
							break;
					}
					break;
			}
    osDelay(1);
  }
  /* USER CODE END supervisorTask */
}

/* USER CODE BEGIN Header_ManageTask */
/**
* @brief Function implementing the ManageSysTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ManageTask */
void ManageTask(void const * argument)
{
  /* USER CODE BEGIN ManageTask */
  /* Infinite loop */
  for(;;)
  {
//		setHydroValue = PID_Moment(setMomentValue, VAL_MomentBrake, 6000.0f, -6000.0f, 1.0f, 0, 0);
//		setPneumoValue = PID_Hydraulic(setHydroValue, VAL_HydroPressure, 22.0f, -22.0f, 1.0f, 0, 0);
//		outPIDpne = PID_Pneumatic(setPneumoValue, VAL_Pressure_BP2_f, 8.0f, -8.0f, 1.0f, 0, 0);
		DAC_voltageControl(outPIDpne);
    osDelay(1);
  }
  /* USER CODE END ManageTask */
}

/* USER CODE BEGIN Header_SendControlCANTask */
/**
* @brief Function implementing the SendCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendControlCANTask */
void SendControlCANTask(void const * argument)
{
  /* USER CODE BEGIN SendControlCANTask */
  /* Infinite loop */
  for(;;)
  {
		transmit = VAL_MomentBrake;
		*((uint32_t*)&TX_data[0]) = *(uint32_t*)&transmit;
		TX_data[4] = ASBStatus;                                      // status(OFF=0x0, MANUAL=0x1, READY=0x2, DRIVING=0x3, EMERGENCY=0x4)
		HAL_CAN_AddTxMessage(&hcan1,&pTxHeader,TX_data,&TxMailbox);  //отправка тормозного момента и статуса на VCDU
    osDelay(10);
  }
  /* USER CODE END SendControlCANTask */
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
