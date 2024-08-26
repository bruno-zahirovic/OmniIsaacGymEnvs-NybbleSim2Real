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
#include "app_x-cube-ai.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "math.h"
#include "ai_datatypes_defines.h"
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
#include "observation_data.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_LOW_VAL						(400u)
#define SERVO_HIGH_VAL						(1950u)
#define SERVO_TICKS_PER_DEG					(8.611f)
#define SERVO_MAX_DEG						(90u)


#define SERVO_GROUP_SHOULDERS				(htim2)
#define SERVO_GROUP_KNEES					(htim3)

#define SERVO_CHANNEL_FRONT_LEFT			(TIM_CHANNEL_1)
#define SERVO_CHANNEL_REAR_LEFT				(TIM_CHANNEL_2)
#define SERVO_CHANNEL_FRONT_RIGHT			(TIM_CHANNEL_3)
#define SERVO_CHANNEL_REAR_RIGHT			(TIM_CHANNEL_4)

#define SERVO_ID_FRONT_LEFT					(1u)
#define SERVO_ID_REAR_LEFT					(2u)
#define SERVO_ID_FRONT_RIGHT				(3u)
#define SERVO_ID_REAR_RIGHT					(4u)

#define SERVO_TIMER_COUNT					(2u)
#define SERVO_CHANNEL_COUNT					(4u)

/*STARTING POSITIONS*/
#define FL_SHOULDER_STARTING_POSITION		(0.0f)
#define RL_SHOULDER_STARTING_POSITION		(0.0f)
#define FR_SHOULDER_STARTING_POSITION		(0.0f)
#define	RR_SHOULDER_STARTING_POSITION		(0.0f)

#define FL_KNEE_STARTING_POSITION			(0.0f)
#define RL_KNEE_STARTING_POSITION			(0.0f)
#define FR_KNEE_STARTING_POSITION			(0.0f)
#define RR_KNEE_STARTING_POSITION			(0.0f)

#define FL_SHOULDER_SIM_OFFSET_RAD			(0.35f)
#define RL_SHOULDER_SIM_OFFSET_RAD			(0.35f)
#define FR_SHOULDER_SIM_OFFSET_RAD			(0.35f)
#define RR_SHOULDER_SIM_OFFSET_RAD			(0.35f)

#define FL_KNEE_SIM_OFFSET_RAD				(-0.785f)
#define RL_KNEE_SIM_OFFSET_RAD				(0.785f)
#define FR_KNEE_SIM_OFFSET_RAD				(-0.785f)
#define RR_KNEE_SIM_OFFSET_RAD				(0.785f)

#define FL_SHOULDER_SIM_FACTOR 				(-1)
#define RL_SHOULDER_SIM_FACTOR				(1)
#define FR_SHOULDER_SIM_FACTOR				(1)
#define RR_SHOULDER_SIM_FACTOR				(-1)


#define FL_KNEE_SIM_FACTOR					(1)
#define RL_KNEE_SIM_FACTOR					(1)
#define FR_KNEE_SIM_FACTOR					(-1)
#define RR_KNEE_SIM_FACTOR					(-1)

/*MPU6050*/
#define MPU_DEVICE_ADDRESS 					(0xD0)

#define MPU_WHO_AM_I_REG					(0x75)
#define MPU_WHO_AM_I_SUCCESS_VALUE 			(0x68)

#define MPU_POWER_MANAGEMENT_ONE_REG		(0x6B)
#define MPU_WAKE_UP_VALUE					(uint8_t)0x0

#define MPU_SAMPLE_RATE_DIV_REG				(0x19)
#define MPU_DATA_RATE						(uint8_t)0x7

#define MPU_GYRO_CONFIG_REG					(0x1B)
#define MPU_GYRO_CONFIG_VALUE				(uint8_t)0x0

#define MPU_ACCEL_CONFIG_REG				(0x1c)
#define MPU_ACCEL_CONFIG_VALUE				(uint8_t)0x0

#define MPU_ACCEL_DATA_START 				(0x3B)
#define MPU_ACCEL_DATA_CONVERSION_FACT		(16384.0f)

#define MPU_GYRO_DATA_START					(0x43)
#define MPU_GYRO_DATA_CONVERSION_FACT		(131.0f)

#define MPU_FILTERING_CONFIG_REG			(0x1A)
#define MPU_FILTERING_CONFIG_VALUE 			(0x03)

#define GYRO_TRAVELED_1MS_FACT				(1.0f/(1000.0f * MPU_GYRO_DATA_CONVERSION_FACT))


#define RAD_TO_DEG							(57.295779513082320876798154814105f)
#define SIM_DT								(0.016666666666f)
#define SIM_ACTION_SCALE					(10u)
#define SIM_DOF_UPPER_LIMIT_RAD				(180.0f / RAD_TO_DEG)
#define SIM_DOF_LOWER_LIMIT_RAD				(-180.0f / RAD_TO_DEG)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t			servoIDs[4]								= 		{
																		SERVO_ID_FRONT_LEFT,
																		SERVO_ID_REAR_LEFT,
																		SERVO_ID_FRONT_RIGHT,
																		SERVO_ID_REAR_RIGHT,
																	};
float_t				nybbleStartingPosition[8]				= 		{
																		FL_SHOULDER_STARTING_POSITION,
																		RL_SHOULDER_STARTING_POSITION,
																		FR_SHOULDER_STARTING_POSITION,
																		RR_SHOULDER_STARTING_POSITION,
																		FL_KNEE_STARTING_POSITION,
																		RL_KNEE_STARTING_POSITION,
																		FR_KNEE_STARTING_POSITION,
																		RR_KNEE_STARTING_POSITION,
																	};

float_t				nybbleSimOffsetRad[8]	 				=		{
																		FL_SHOULDER_SIM_OFFSET_RAD - FL_SHOULDER_STARTING_POSITION / RAD_TO_DEG,
																		RL_SHOULDER_SIM_OFFSET_RAD - RL_SHOULDER_STARTING_POSITION / RAD_TO_DEG,
																		FR_SHOULDER_SIM_OFFSET_RAD - FR_SHOULDER_STARTING_POSITION / RAD_TO_DEG,
																		RR_SHOULDER_SIM_OFFSET_RAD - RR_SHOULDER_STARTING_POSITION / RAD_TO_DEG,
																		FL_KNEE_SIM_OFFSET_RAD - FL_KNEE_STARTING_POSITION / RAD_TO_DEG,
																		RL_KNEE_SIM_OFFSET_RAD - RL_KNEE_STARTING_POSITION / RAD_TO_DEG,
																		FR_KNEE_SIM_OFFSET_RAD - FR_KNEE_STARTING_POSITION / RAD_TO_DEG,
																		RR_KNEE_SIM_OFFSET_RAD - RR_KNEE_STARTING_POSITION / RAD_TO_DEG
																	};

int8_t				nybbleSimFactor[8]						= 		{
																		FL_SHOULDER_SIM_FACTOR,
																		RL_SHOULDER_SIM_FACTOR,
																		FR_SHOULDER_SIM_FACTOR,
																		RR_SHOULDER_SIM_FACTOR,
																		FL_KNEE_SIM_FACTOR,
																		RL_KNEE_SIM_FACTOR,
																		FR_KNEE_SIM_FACTOR,
																		RR_KNEE_SIM_FACTOR
																	};

static float_t 		startingTargets[AI_NETWORK_OUT_1_SIZE] = 		{
																		FL_SHOULDER_SIM_OFFSET_RAD,
																		RL_SHOULDER_SIM_OFFSET_RAD,
																		FR_SHOULDER_SIM_OFFSET_RAD,
																		RR_SHOULDER_SIM_OFFSET_RAD,
																		FL_KNEE_SIM_OFFSET_RAD,
																		RL_KNEE_SIM_OFFSET_RAD,
																		FR_KNEE_SIM_OFFSET_RAD,
																		RR_KNEE_SIM_OFFSET_RAD
																	};

static float_t 		servoTargets[AI_NETWORK_OUT_1_SIZE];
static float_t 		currentTargets[AI_NETWORK_OUT_1_SIZE];


/*****AI****/
/* Global handle to reference the instantiated C-model */
static ai_handle network = AI_HANDLE_NULL;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(32)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

/* Array to store the data of the input tensor */
AI_ALIGNED(32)
static ai_float in_data[AI_NETWORK_IN_1_SIZE] = {0};


/* c-array to store the data of the output tensor */
AI_ALIGNED(32)
static ai_float out_data[AI_NETWORK_OUT_1_SIZE];


/* Array of pointer to manage the model's input/output tensors */
static ai_buffer *ai_input;
static ai_buffer *ai_output;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/*SERVO CONTROL*/
static void 		SERVO_Init(TIM_HandleTypeDef* servoGroups, uint32_t* servoChannels);
static void 		SERVO_SetPosition(TIM_HandleTypeDef servoGroup, uint32_t servoID, float_t deg);
static void 		NYBBLE_SetPose(TIM_HandleTypeDef* servoGroups, uint32_t* servoIDs, float_t* pose);

/*GENERAL*/

void 				HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/*AI*/
static uint32_t 	AI_Init(void);
static uint32_t 	AI_Run(const void *in_data, void *out_data);
static void			AI_PostProcessOutput(ai_float *out_data);
static void 		AI_CalculateServoTargets(ai_float *out_data, float_t *currentTargets, float_t *servoTargets);

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
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_X_CUBE_AI_Init();
  /* USER CODE BEGIN 2 */
TIM_HandleTypeDef	servoGroups[2] 		= 	{
  												SERVO_GROUP_SHOULDERS,
  												SERVO_GROUP_KNEES,
  											};
  uint32_t			servoChannels[4] 	= 	{
  												SERVO_CHANNEL_FRONT_LEFT,
  												SERVO_CHANNEL_REAR_LEFT,
  												SERVO_CHANNEL_FRONT_RIGHT,
  												SERVO_CHANNEL_REAR_RIGHT,
  											};

  uint8_t 			observationCounter 	= 	0;

  SERVO_Init(servoGroups, servoChannels);
  AI_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

  MX_X_CUBE_AI_Process();
    /* USER CODE BEGIN 3 */


	  if (0 == observationCounter){
		  NYBBLE_SetPose(servoGroups, servoIDs, nybbleStartingPosition);
		  
		  //Reset targets to sim starting targets
		  for (int i = 0; i< AI_NETWORK_OUT_1_SIZE; i++){
			  currentTargets[i] = startingTargets[i];
		  }
		  HAL_Delay(3000);
	  }
	  
	  //Copy observations from simulation into network input array
	  memcpy(in_data, observation_data[observationCounter], 36 * sizeof(ai_float));
	  
	  //Inference
	  AI_Run(in_data, out_data);
	  
	  observationCounter++;

	  //Limit outputs to [-1.0, 1.0]
	  AI_PostProcessOutput(out_data);

	  //Calculate servos angles in degrees based on output from NN and currently stored sim targets
	  AI_CalculateServoTargets(out_data, currentTargets, servoTargets);

	  NYBBLE_SetPose(servoGroups, servoIDs, servoTargets);

	  //
	  HAL_Delay(50);

	  if (observationCounter >= 200){
		  observationCounter = 0;
	  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100 -1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16000 - 1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100 -1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 16000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void SERVO_Init(TIM_HandleTypeDef* servoGroups, uint32_t* servoChannels)
{
	for (int i=0; i<SERVO_TIMER_COUNT; i++){
		for (int j=0; j<SERVO_CHANNEL_COUNT; j++){
			HAL_TIM_PWM_Start(&servoGroups[i], servoChannels[j]);
		}
	}
}

static void SERVO_SetPosition(TIM_HandleTypeDef servoGroup, uint32_t servoID, float_t deg)
{
	uint8_t isNegative = deg < 0;
	if (deg >= (int8_t)SERVO_MAX_DEG){
		deg = (int8_t)SERVO_MAX_DEG;
	}
	if (deg <= -(int8_t)SERVO_MAX_DEG){
		deg = (int8_t)-SERVO_MAX_DEG;
	}
	if (isNegative) {
		deg *= -1;
	}

	uint32_t degToServoTicks = 0;
	if (isNegative){
		degToServoTicks = (uint32_t)(SERVO_LOW_VAL + (SERVO_MAX_DEG - deg) * SERVO_TICKS_PER_DEG);
	} else {
		degToServoTicks = (uint32_t)(SERVO_HIGH_VAL - (SERVO_MAX_DEG - deg) * SERVO_TICKS_PER_DEG);
	}
	switch (servoID){
	case 1:
		servoGroup.Instance->CCR1 = degToServoTicks;
		break;
	case 2:
		servoGroup.Instance->CCR2 = degToServoTicks;
		break;
	case 3:
		servoGroup.Instance->CCR3 = degToServoTicks;
		break;
	case 4:
		servoGroup.Instance->CCR4 = degToServoTicks;
		break;
	default:
		break;
	}


}

static void NYBBLE_SetPose(TIM_HandleTypeDef* servoGroups, uint32_t* servoIDs, float_t* pose)
{

	for(int i=0; i<SERVO_TIMER_COUNT; i++){
		for(int j=0; j<SERVO_CHANNEL_COUNT; j++){
			SERVO_SetPosition(servoGroups[i], servoIDs[j], pose[j+i*SERVO_CHANNEL_COUNT]);
		}
	}
}

static uint32_t AI_Init(void) {
  ai_error err;

  /* Create and initialize the c-model */
  const ai_handle acts[] = { activations };
  err = ai_network_create_and_init(&network, acts, NULL);
  if (err.type != AI_ERROR_NONE) { while(1); };

  /* Reteive pointers to the model's input/output tensors */
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);

  return 0;
}

static uint32_t AI_Run(const void *in_data, void *out_data) {
  ai_i32 n_batch;
  ai_error err;

  /* 1 - Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(in_data);
  ai_output[0].data = AI_HANDLE_PTR(out_data);

  /* 2 - Perform the inference */
  n_batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
  if (n_batch != 1) {
      err = ai_network_get_error(network);
  };

  return 0;
}

static void AI_PostProcessOutput(ai_float *out_data){
	for (int idx=0; idx < AI_NETWORK_OUT_1_SIZE; idx++ )
	{
		if(out_data[idx] > 1.0f){
			out_data[idx] = 1.0f;
		}
		if (out_data[idx] < -1.0f){
			out_data[idx] = -1.0f;
		}
	}

}

static void AI_CalculateServoTargets(ai_float *out_data, float_t *currentTargets, float_t *servoTargets){
	for (int outCnt = 0; outCnt < AI_NETWORK_OUT_1_SIZE; outCnt++){
	  currentTargets[outCnt] = currentTargets[outCnt] + SIM_ACTION_SCALE * out_data[outCnt] * SIM_DT;

	  if (currentTargets[outCnt] > SIM_DOF_UPPER_LIMIT_RAD){
		  currentTargets[outCnt] = SIM_DOF_UPPER_LIMIT_RAD;
	  } else if (currentTargets[outCnt] < SIM_DOF_LOWER_LIMIT_RAD){
		  currentTargets[outCnt] = SIM_DOF_LOWER_LIMIT_RAD;
	  }

	  servoTargets[outCnt] = ((currentTargets[outCnt] - nybbleSimOffsetRad[outCnt]) * RAD_TO_DEG) * nybbleSimFactor[outCnt];
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
