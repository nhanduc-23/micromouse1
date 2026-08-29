/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "encoder.h"
// #include "vl53l0x.h"
// #include "mpu6500.h"
// #include "pid.h"
// #include "floodfill.h"
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

/* USER CODE BEGIN PV */
volatile uint8_t system_mode = 0; //0 : Standby, 1: Do duong, 2: Speedrun, 3: Lock
//Bien theo doi xung Encoder phuc vu kiem tra va tinh toan
int16_t pulse_left = 0;
int16_t pulse_right = 0;


// Toa do va huong di cua robot
int current_x = 0, current_y = 0;
int orient = 0; // 0: FORWARD, 1: RIGHT, 2: BACKWARD, 3: LEFT
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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
  
  /* USER CODE BEGIN 2 */
  // Khoi tao khoi dong co va kich hoat PWM TIM1
  Motor_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (system_mode == 0) {
      // Che do Cho (Standby): Dung motor, nhap nhay LED PC13 bao hieu
      Motor_Stop();
      HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
      HAL_Delay(500);
    }
    else if (system_mode == 1) {
      // Che do 1: DO DUONG (Flood Fill + PID bam tuong)
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // Bat LED bao dang chay[cite: 1, 2]
      
      // 1. Doc khoang cach tu 4 con VL53L0X
      // 2. Chay thuat toan PID va Flood Fill
      // 3. Xuat toc do: Motor_SetSpeed(speed_L, speed_R);
    }
    else if (system_mode == 2) {
      // Che do 2: SPEEDRUN (Chay toc do cao theo map da luu)
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
      
      // Th?c thi luong chay Speedrun
    }
    else if (system_mode == 3) {
      // Che do 3: KHOA HE THONG (Treo xe ngung hoat dong)[cite: 1]
      Motor_Stop();
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // Tat LED PC13[cite: 1, 2]
      // Vong lap khoa he thong den khi bam nut Hardware Reset (NRST)[cite: 1, 2]
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
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
// Ham ngat ngoai xu ly nut nhan PA2 chuyen che do[cite: 1, 2]
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == BTN_MODE_PIN) {
    if (system_mode < 2) {
      system_mode++; // Nh?n l?n 1: Dò du?ng (1), L?n 2: Speedrun (2)[cite: 1]
    } else {
      system_mode = 3; // Nh?n l?n 3 tr? di: Khóa xe (3)[cite: 1]
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