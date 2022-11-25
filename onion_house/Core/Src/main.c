/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "encoder.h"
#include "ktir_sensor.h"
#include "motor.h"
#include "servo.h"
#include "TCS34725.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_RIGHT_CHANNEL TIM_CHANNEL_3
#define SERVO_RIGHT_TIMER htim3
#define SERVO_LEFT_CHANNEL TIM_CHANNEL_4
#define SERVO_LEFT_TIMER htim3

#define MOTOR_RIGHT_CHANNEL_A TIM_CHANNEL_3
#define MOTOR_RIGHT_CHANNEL_B TIM_CHANNEL_4
#define MOTOR_RIGHT_TIMER htim1

#define MOTOR_BASE_SPEED 45

#define MOTOR_LEFT_CHANNEL_A TIM_CHANNEL_2
#define MOTOR_LEFT_CHANNEL_B TIM_CHANNEL_1
#define MOTOR_LEFT_TIMER htim1

#define COLOR_LEFT_I2C hi2c1
#define COLOR_RIGHT_I2C hi2c2
#define COLOR_EMPTY_CRITICAL 600

#define KTIR_ADC hadc1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Servo left_servo, right_servo;

Motor left_motor, right_motor;

ColorSensor left_color, right_color;

Encoder left_encoder, right_encoder;

volatile uint16_t ktir_results[NUMBER_OF_SENSORS];

const uint16_t KTIR_CRITICAL_VALUES[NUMBER_OF_SENSORS] = 	{
		2000u,
		2000u,
		2000u,
		2000u
};


uint32_t tick_open_gates = 0;
uint8_t opened_gates = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

uint16_t get_H_from_RGB(uint32_t, uint32_t, uint32_t);
uint8_t is_red_onion(ColorSensor*);
uint8_t is_black(uint16_t value, uint16_t critical_value);
uint8_t is_crossing();
void turn_slight(Motor* slower, Motor* faster, uint8_t value);
void turn(Motor* slower, Motor* faster, uint8_t value);
void go_straight(int8_t speed);
void show_for_calibration();
void show_color_for_calibration(ColorSensor* sensor);
void open_close_gates();
void follow_the_line();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  servo_init(&right_servo, &SERVO_RIGHT_TIMER, SERVO_RIGHT_CHANNEL);
  servo_init(&left_servo, &SERVO_LEFT_TIMER, SERVO_LEFT_CHANNEL);

  motor_init(&right_motor, &MOTOR_RIGHT_TIMER, MOTOR_RIGHT_CHANNEL_A, MOTOR_RIGHT_CHANNEL_B);
  motor_init(&left_motor, &MOTOR_LEFT_TIMER, MOTOR_LEFT_CHANNEL_A, MOTOR_LEFT_CHANNEL_B);

  //ENCODER_init(&left_encoder, &htim3);

  //ENCODER_init(&right_encoder, &htim4);

  TCS34725_init(&left_color, &COLOR_LEFT_I2C);
  TCS34725_init(&right_color, &COLOR_RIGHT_I2C);

  KTIR_Init(&KTIR_ADC, ktir_results);
  //uint8_t position = 180;
  //int8_t count = 1;

  uint8_t rotating = 0;
  uint32_t tick_start_rotating;
  uint8_t STOP = 1;
  //ENCODER_get_value(&left_encoder);
  //ENCODER_get_value(&right_encoder);

  servo_set_angle(&right_servo, 70);

  servo_set_angle(&left_servo, 110);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /**
	   * PROGRAM MUSI:
	   * - dojechać z bazy do planszy
	   * - śledzić linię
	   * - gdy wykryje skrzyżowanie dodać jeden, jeśli to ostatnie to 90 stopni
	   * 	- i potem ruch do następnego skrzyżowania i znowu 90 stopni
	   * - sprawdzać kolor i w zależności od tego odpowiednio poruszać bramkami (open_close_gates)
	   * - liczyć czas - jeżeli już mało to długa do bazy
	   * -
	   */
	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0){
		STOP = STOP == 0 ? 1:0;
		while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0){
			HAL_Delay(10);
		}
	}
	KTIR_read();
	//printf("-----\nPrawy\n");
	//show_color_for_calibration(&right_color);
	//printf("Lewy\n");
	//show_color_for_calibration(&left_color);

	show_for_calibration();


	if(!STOP){

		//servo_set_angle(&right_servo, 180);

		//servo_set_angle(&left_servo, 0);

		if(!rotating){
			follow_the_line();
		}
		// Jeśli wykryje skryżowanie to sprawdź czas
		if(is_crossing()){
			rotating = 1;
			tick_start_rotating = HAL_GetTick();
		}
		//Po upływie pewnego czasu zacznij skręcać
		if(rotating && HAL_GetTick() - tick_start_rotating > 200){
			motor_run(&right_motor, MOTOR_BASE_SPEED-20);
			motor_run(&left_motor, -MOTOR_BASE_SPEED-35);
		}
		//Jeśli wrócisz na linie to zakończ skręcanie
		if(!is_black(ktir_results[0], KTIR_CRITICAL_VALUES[0]) && is_black(ktir_results[1], KTIR_CRITICAL_VALUES[1])
				&& is_black(ktir_results[2], KTIR_CRITICAL_VALUES[2]) && !is_black(ktir_results[3], KTIR_CRITICAL_VALUES[3]) && rotating){
			//turn(&left_motor, &right_motor, 5);
			rotating = 0;
			//motor_brake(&right_motor);
			//motor_brake(&left_motor);
		}

		open_close_gates();
	/*
		servo_set_angle(&right_servo, position);

		servo_set_angle(&left_servo, 180 - position);

		if (position <= 0 || position >= 180) count = -count;
		position += 2*count;

		go_straight(&left_motor, &right_motor);



		/*
		if(is_black(ktir_results[1], KTIR_CRITICAL_VALUES[1]) && is_black(ktir_results[2], KTIR_CRITICAL_VALUES[2])){
			go_straight(&left_motor, &right_motor);
		}
		if(!is_black(ktir_results[1], KTIR_CRITICAL_VALUES[1]) && is_black(ktir_results[2], KTIR_CRITICAL_VALUES[2])){
			turn(&left_motor, &right_motor, 5);
		}
		if(is_black(ktir_results[1], KTIR_CRITICAL_VALUES[1]) && !is_black(ktir_results[2], KTIR_CRITICAL_VALUES[2])){
			turn(&right_motor, &left_motor, 5);
		}


		*/
		HAL_Delay(5);
	}
	else {
		motor_brake(&right_motor);
		motor_brake(&left_motor);
	}
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
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 10-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief Get hue from RGB values
 * @param red: Red value (0, 255)
 * @param green: Green value (0, 255)
 * @param blue: Blue value (0, 255)
 */

uint16_t get_H_from_RGB(uint32_t red, uint32_t green, uint32_t blue){
	uint16_t c_max = red > green ? (red > blue ? red : blue) : (green > blue ? green : blue);
	uint16_t c_min = red < green ? (red < blue ? red : blue) : (green < blue ? green : blue);

	float d = (float)(c_max - c_min);
	d /= 255.0;
	uint16_t h = 0;
	float r = red/255.0;
	float g = green/255.0;
	float b = blue/255.0;
	if (d == 0){
	  h = 0;
	}
	else if(c_max == red){
	  h = (int)(60*(g - b)/d + 360)%360;
	}
	else if(c_max == green){
	  h = (60*(b - r)/d) + 120;
	}
	else {
	  h = (60*(r - g)/d) + 240;
	}
	return h;
}

uint8_t is_red_onion(ColorSensor *sensor){
	uint16_t red = 0;
	unlock(sensor);
	get_red(sensor, &red);
	uint16_t color = 0;
	get_light(sensor, &color);
	uint16_t green = 0;
	get_green(sensor, &green);
	uint16_t blue = 0;
	get_blue(sensor, &blue);
	lock(sensor);

	uint16_t h = get_H_from_RGB(red, green, blue);

	if(h > 230 || h < 12) {
		return 1;
	}
	else {
		return 0;
	}
}

uint8_t is_black(uint16_t value, uint16_t critical_value){
	return value > critical_value ? 1:0;
}

void show_for_calibration(){
	/**
   * Poniżej fragment do kalibracji krytycznych wartości
   */
	printf("Sensor[0]: %d\tSensor[1]: %d\tSensor[2]: %d\tSensor[3]: %d\n", ktir_results[0], ktir_results[1], ktir_results[2], ktir_results[3]);

  	printf("------\nCzarne[0]: %d\tCzarne[1]: %d\tCzarne[2]: %d\tCzarne[3]: %d\n",
	is_black(ktir_results[0], KTIR_CRITICAL_VALUES[0]), is_black(ktir_results[1], KTIR_CRITICAL_VALUES[1]),
	is_black(ktir_results[2], KTIR_CRITICAL_VALUES[2]), is_black(ktir_results[3], KTIR_CRITICAL_VALUES[0]));
}


void turn_slight(Motor* slower, Motor* faster, uint8_t value){
	motor_run(slower, MOTOR_BASE_SPEED - value);
	motor_run(faster, MOTOR_BASE_SPEED);
}

void go_straight(int8_t speed){
	motor_run(&left_motor, speed);
	motor_run(&right_motor, speed);

}

void turn(Motor* slower, Motor* faster, uint8_t value){
	motor_run(slower, MOTOR_BASE_SPEED - value);
	motor_run(faster, MOTOR_BASE_SPEED + value);
}

void show_color_for_calibration(ColorSensor* sensor){
	uint16_t red = 0;
		unlock(sensor);
		get_red(sensor, &red);
		uint16_t color = 0;
		get_light(sensor, &color);
		uint16_t green = 0;
		get_green(sensor, &green);
		uint16_t blue = 0;
		get_blue(sensor, &blue);
		lock(sensor);
		uint16_t h = get_H_from_RGB(red, green, blue);
		printf("R: %d\tG: %d\tB: %d\tC: %d \t H = %d\n", red, green, blue, color, h);

}

void open_close_gates(){
	if ((is_red_onion(&left_color) || is_red_onion(&right_color)) && opened_gates == 0){
			servo_set_angle(&right_servo, 0);
			servo_set_angle(&left_servo, 180);
			opened_gates = 1;
			tick_open_gates = HAL_GetTick();
		} else {
			uint16_t right = 0, left = 0;
			get_light(&right_color, &right);
			get_light(&left_color, &left);
			if((right > COLOR_EMPTY_CRITICAL || left > COLOR_EMPTY_CRITICAL) && opened_gates == 0){
				servo_set_angle(&right_servo, 170);
				servo_set_angle(&left_servo, 10);
				opened_gates = 2;
				tick_open_gates = HAL_GetTick();
			} else if (opened_gates && HAL_GetTick() - tick_open_gates > 2000){

				  servo_set_angle(&right_servo, 70);

				  servo_set_angle(&left_servo, 110);
				  opened_gates = 0;
			}
		}
}

void follow_the_line(){
	if(is_black(ktir_results[1], KTIR_CRITICAL_VALUES[1]) && is_black(ktir_results[2], KTIR_CRITICAL_VALUES[2])){
		go_straight(MOTOR_BASE_SPEED);
	}
	if(!is_black(ktir_results[1], KTIR_CRITICAL_VALUES[1]) && is_black(ktir_results[2], KTIR_CRITICAL_VALUES[2])){
		turn(&left_motor, &right_motor, 7);
	}
	if(is_black(ktir_results[1], KTIR_CRITICAL_VALUES[1]) && !is_black(ktir_results[2], KTIR_CRITICAL_VALUES[2])){
		turn(&right_motor, &left_motor, 7);
	}
}

uint8_t is_crossing(){
	return is_black(ktir_results[0], KTIR_CRITICAL_VALUES[0]) && is_black(ktir_results[1], KTIR_CRITICAL_VALUES[1])
			&& is_black(ktir_results[2], KTIR_CRITICAL_VALUES[2]) && is_black(ktir_results[3], KTIR_CRITICAL_VALUES[3]);
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

