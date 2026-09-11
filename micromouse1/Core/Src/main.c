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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tb6612fng.h"
#include "encoder.h"
#include "bt_debug.h"
#include "vl53l0x.h"
#include "mpu6500.h"
#include "sensor_fusion.h"
#include "motion_controller.h"
#include "pid.h"
#include "flash.h"
#include "maze_solver.h"
#include <stdio.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    SYS_STATE_IDLE,
    SYS_STATE_EXPLORE,
    SYS_STATE_FAST_RUN,
    SYS_STATE_FINISHED,
    SYS_STATE_ERROR
} SystemState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_LOOP_PERIOD_MS 10 // Chu ki vong lap 10ms (100Hz)
#define DEBOUNCE_DELAY_MS      200 // Thoi gian chong doi nut nhan
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
static SystemState current_sys_state = SYS_STATE_IDLE;
static uint32_t last_tick = 0;
static uint32_t last_button_tick = 0;

//Them 2 bien luu du lieu cam bien cho Sensor Fusion
static VL53L0X_Data vl_sensor_data;
static MPU6500_Data mpu_sensor_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void Process_FSM(float dt);
uint8_t Is_Button_Pressed(void);
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
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	// kich hoat motor driver TB6612 (chuyen chan SSTBY tu RESET sang SET)
	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  // 1. Khoi tao cac module phan cung
	BT_Init_Rx(&huart6); // Khoi tao Bluetooth UART & bat ngat nhan du lieu
	Encoder_Init(&htim2, &htim3);
	TB6612_Init(&htim1);
	LOG_SYS("--- MICROMOUSE STARTUP ---\r\n");
	// 2. Khoi tao cam bien
	if (VL53L0X_Init_All(&hi2c1) != HAL_OK) {
      LOG_SYS("[ERROR] VL53L0X Init Failed!\r\n");
  } else {
      LOG_SYS("[OK] VL53L0X Initialized.\r\n");
  }

  if (MPU6500_Init(&hi2c2) != HAL_OK) {
      LOG_SYS("[ERROR] MPU6500 Init Failed!\r\n");
  } else {
      LOG_SYS("[OK] MPU6500 Initialized.\r\n");
  }
	//3. Khoi tao thuat toan & bo dieu khien
	SensorFusion_Init();
  Motion_Init();
  Maze_Init();

  LOG_SYS("System Ready. Press Switch to Start Exploration.\r\n");
  last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint32_t current_tick = HAL_GetTick();
		
		//Dinh thoi chu ky 10ms (100Hz) cho PID va Sensor Fusion
		if (current_tick - last_tick >= CONTROL_LOOP_PERIOD_MS) {
			float dt = (current_tick - last_tick) / 1000.0f;
        last_tick = current_tick;
			
			//1. Khai bao bien va doc khoang cach tu Encoder
			float dist_l = 0.0f;
			float dist_r = 0.0f;
			Encoder_GetDistanceMM(&dist_l, &dist_r);
			//2. Doc du lieu tu cam bien V53L0X va MPU6500
			VL53L0X_Read_All(&vl_sensor_data);
      MPU6500_Read_GyroZ(dt);
			
			//Copy du lieu Gyro moi nhat tu driver vao mpu_sensor_data
			MPU6500_Data *mpu_ptr = MPU6500_GetData();
      if (mpu_ptr != NULL) {
          mpu_sensor_data = *mpu_ptr;
      }
			
			// 3.cap nhat cam bien & bo dieu khien dong co 
			SensorFusion_Update(&vl_sensor_data, &mpu_sensor_data, dist_l, dist_r, dt);
      Motion_Update(dt);
			
			// 4.Chay may trang thai he thong
			Process_FSM(dt);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  hi2c2.Init.ClockSpeed = 400000;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, XSHUT_L_Pin|XSHUT_FL_Pin|STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, XSHUT_FR_Pin|XSHUT_R_Pin|AIN1_Pin|AIN2_Pin
                          |BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_STAT_Pin */
  GPIO_InitStruct.Pin = LED_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SWITCH_PIN_Pin */
  GPIO_InitStruct.Pin = SWITCH_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SWITCH_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : XSHUT_L_Pin XSHUT_FL_Pin STBY_Pin */
  GPIO_InitStruct.Pin = XSHUT_L_Pin|XSHUT_FL_Pin|STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : XSHUT_FR_Pin XSHUT_R_Pin AIN1_Pin AIN2_Pin
                           BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = XSHUT_FR_Pin|XSHUT_R_Pin|AIN1_Pin|AIN2_Pin
                          |BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
* @brief Kiem tra nut nhan chong doi va bat canh xuong (One-shot / Press Event)
*/
uint8_t Is_Button_Pressed(void) {
    static uint8_t btn_last_state = 1; // 1: Nha nut (GPIO_PIN_SET), 0: Nhan nut (GPIO_PIN_RESET)
    uint8_t current_state = HAL_GPIO_ReadPin(SWITCH_PIN_GPIO_Port, SWITCH_PIN_Pin);
		uint32_t now = HAL_GetTick();
	
    // Phat hien thoi diem bat dau nhan nut (chi kich hoat khi truoc do nut dang nha)
    if (current_state == GPIO_PIN_RESET && btn_last_state == 1) {
        if (HAL_GetTick() - last_button_tick > DEBOUNCE_DELAY_MS) {
            last_button_tick = now;
            btn_last_state = 0; // Danh dau nat dang duoc giu
            return 1;           // Chi tra ve 1 dung mot lan duy nhat cho moi lan nhan
        }
    } 
    // Khi nguoi dung nha nut ra hoan toan
    else if (current_state == GPIO_PIN_SET && btn_last_state == 0) {
        if (now - last_button_tick > DEBOUNCE_DELAY_MS) {
				btn_last_state = 1;     // Ðat lai trang thai cho cho lan nhan tiep theo
    }
	}
    return 0;
}
void Process_FSM(float dt) {
    static Action next_action = ACTION_STOP;

    switch (current_sys_state) {
        case SYS_STATE_IDLE:
            if (Is_Button_Pressed()) {
                LOG_SYS("Starting Exploration Run...\r\n");
                Maze_Init(); // Reset ban do và vi tri robot ve (0,0)
								SensorFusion_ResetAngle();
								next_action = ACTION_STOP;
                current_sys_state = SYS_STATE_EXPLORE;
            }
            break;

        case SYS_STATE_EXPLORE:
            if (Motion_IsFinished()) {
                // Buoc 1: Cap nhat toa do ao tuong ung voi buoc vua chay xong
                Maze_UpdatePosition(next_action);

                // Buoc 2: Kiem tra da den dich chua
                if (Maze_IsGoalReached()) {
                    LOG_SYS("Goal Reached! Saving Map to Flash...\r\n");
                    Motion_Stop();
                    Maze_SaveToFlash();
                    HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_SET);
                    current_sys_state = SYS_STATE_FINISHED;
                    break;
                }

                // Buoc 3: Ðoc tuong tai toa do MOI và tinh toan Floodfill
                SensorFusion_Data *sf = SensorFusion_GetData();
                Maze_UpdateWalls(sf);
                Maze_ComputeFloodfill();

                // Buoc 4: Lay hanh dong tiep theo dua trên bang Floodfill moi
                next_action = Maze_GetNextAction();

                // Buoc 5: Thuc thi chuyen dong
                switch (next_action) {
                    case ACTION_FORWARD:
                        Motion_MoveForward(180.0f);
                        break;
                    case ACTION_TURN_LEFT:
                        Motion_Rotate(90.0f);
                        break;
                    case ACTION_TURN_RIGHT:
                        Motion_Rotate(-90.0f);
                        break;
                    case ACTION_TURN_AROUND:
                        Motion_Rotate(180.0f);
                        break;
                    default:
                        Motion_Stop();
                        current_sys_state = SYS_STATE_FINISHED;
                        break;
                }
            }
            break;

        case SYS_STATE_FAST_RUN:
            if (Motion_IsFinished()) {
                // Cap nhat vi tri tu hanh dong vua xong
                Maze_UpdatePosition(next_action);

                if (Maze_IsGoalReached()) {
                    Motion_Stop();
                    LOG_SYS("Fast Run Completed Successfully!\r\n");
                    current_sys_state = SYS_STATE_FINISHED;
                    break;
                }

                // Chay Fast Run chi tinh duong ngan nhat (không can quét tuong lai)
                next_action = Maze_GetNextAction();

                switch (next_action) {
                    case ACTION_FORWARD:
                        Motion_MoveForward(180.0f);
                        break;
                    case ACTION_TURN_LEFT:
                        Motion_Rotate(90.0f);
                        break;
                    case ACTION_TURN_RIGHT:
                        Motion_Rotate(-90.0f);
                        break;
                    case ACTION_TURN_AROUND:
                        Motion_Rotate(180.0f);
                        break;
                    default:
                        Motion_Stop();
                        current_sys_state = SYS_STATE_ERROR;
                        break;
                }
            }
            break;

        case SYS_STATE_FINISHED:
            if (Is_Button_Pressed()) {
                LOG_SYS("Loading Map from Flash & Starting Fast Run...\r\n");
                // Reset vi tri xe ve (0,0) truoc, sau dó load ma tran tuong tu Flash
                Maze_Init();
                Maze_LoadFromFlash();
                Maze_ComputeFloodfill(); // Tinh duong toi uu
                SensorFusion_ResetAngle(); // Reset goc moc ve 0 khi bat dau Fast Run
                next_action = ACTION_STOP;
                current_sys_state = SYS_STATE_FAST_RUN;
            }
            break;

        case SYS_STATE_ERROR:
        default:
            Motion_Stop();
            break;
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
