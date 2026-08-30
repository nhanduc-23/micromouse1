/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Micromouse STM32F411
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
#include "vl53l0x.h"
// #include "mpu6500.h"
// #include "pid.h"
// #include "floodfill.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
volatile uint8_t system_mode = 0; // 0: Standby, 1: Do duong, 2: Speedrun, 3: Lock

// Bien theo doi xung Encoder
int16_t pulse_left = 0;
int16_t pulse_right = 0;

// Toa do va huong di cua robot trong me cung
int current_x = 0, current_y = 0;
int orient = 0; // 0: FORWARD, 1: RIGHT, 2: BACKWARD, 3: LEFT
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  // Khoi tao khoi dong co, Encoder va 4 cam bien Laser
  Motor_Init();        // Kich hoat PWM TIM1 dieu khien motor
  Encoder_Init();      // Kich hoat TIM2 & TIM3 doc xung banh xe
  VL53L0X_Init_All();  // Doi dia chi I2C va bat 4 cam bien Laser
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Cap nhat gia tri xung Encoder lien tuc
    pulse_left = Encoder_Get_Left();
    pulse_right = Encoder_Get_Right();
    
    if (system_mode == 0) {
      // Che do 0: Standby - Dung motor, nhap nhay LED PC13
      Motor_Stop();
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(500);
    }
    else if (system_mode == 1) {
      // Che do 1: DO DUONG (Flood Fill + PID bam tuong)
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Bat LED
      
      // 1. Doc khoang cach tu 4 cam bien Laser
      VL53L0X_Read_All(&sensor_dist);
      
      // 2. Kiem tra vat can bang cac API
      if (wallFront()) {
        // Phat hien vach ngan phia truoc (2 mat cheo)
      }
      if (wallLeft()) {
        // Phat hien vach ngan ben trai
      }
      if (wallRight()) {
        // Phat hien vach ngan ben phai
      }
    }
    else if (system_mode == 2) {
      // Che do 2: SPEEDRUN
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    else if (system_mode == 3) {
      // Che do 3: KHOA HE THONG
      Motor_Stop();
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      HAL_Delay(100);
    }
    /* USER CODE END WHILE */
  }
}

/* SystemClock_Config va MX_GPIO_Init giu nguyen */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
// Ham ngat ngoai xu ly nut nhan PA2 chuyen che do (Duy nhat 1 ham o day)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == BTN_MODE_PIN) {
    if (system_mode < 2) {
      system_mode++; // Nhan lan 1: Do duong (1), Lan 2: Speedrun (2)
    } else {
      system_mode = 3; // Nhan lan 3: Khoa xe (3)
    }
  }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}