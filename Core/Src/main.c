/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Chuong trinh chinh dieu khien Robot Micromouse (Pure C)
  *                   Toi uu hoa cho sa ban thi dau 16x16:
  *                   - Tien sat tuong truoc (43-44mm) roi quay ngoat nhanh (Snap Turn)
  *                   - Can chinh lai tam o ngay lap tuc sau khi re (Post-Turn Centering)
  *                   - Bao ve chu ky 100Hz (kep dt <= 25ms, ngat log UART khi xe dang chay)
  *                   - Nut RESET: Xoa sach RAM + Flash; Nut MODE: Do duong / Fast Run / Dung khan cap
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    SYS_STATE_IDLE,       // Dung cho lenh xuat phat (sau khi cap nguon / nhan RESET)
    SYS_STATE_COUNTDOWN,  // Dem nguoc 2 giay cho on dinh co khi va ToF
    SYS_STATE_EXPLORE,    // Luot chay tham do me cung (Search Run den tam dich)
    SYS_STATE_DOCKING,    // Can chinh co hoc 2 vach vuong goc triet tieu sai so
    SYS_STATE_RETURN,     // Luot chay tro ve diem xuat phat (Return Run)
    SYS_STATE_FAST_RUN,   // Luot chay toc do cao dua vao ban do Flash (Fast Run)
    SYS_STATE_FINISHED,   // Hoan thanh luot chay / San sang cho luot Speedrun moi
    SYS_STATE_ERROR       // Trang thai loi he thong
} SystemState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_LOOP_PERIOD_MS 10     // Chu ky dieu khien 10ms (100Hz)
#define MAX_CONTROL_DT_SEC     0.025f // Gioi han dt toi da tranh soc tich phan IMU/PID
#define DEBOUNCE_DELAY_MS      200    // Thoi gian chong doi nut nhan (ms)
#define DOCK_PRESS_TIME_MS     450    // Thoi gian ep lui ty vach voi xung 220 PWM
#define MAX_EXPLORE_PASSES     1      // 1 = Dung cho Speedrun ngay sau 1 luot di + 1 luot ve
/* USER CODE END PD */

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
static bool has_valid_speedrun_map = false;

static VL53L0X_Data vl_sensor_data;

extern bool g_bt_stream_enabled;
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
static void Process_FSM(float dt);
static uint8_t Is_Button_Pressed(void);
/* USER CODE END PFP */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
  BT_Init_Rx(&huart6);
  LOG_SYS("\r\n==================================\r\n");
  LOG_SYS("   MICROMOUSE STM32 (16x16 PRO)   \r\n");
  LOG_SYS("==================================\r\n");

  TB6612_Init(&htim1);
  Encoder_Init(&htim2, &htim3);
  LOG_SYS("[OK] Motor & Encoders Initialized.\r\n");

  if (VL53L0X_Init_All(&hi2c1) != HAL_OK) {
      LOG_SYS("[WARN] VL53L0X Init that bai hoac thieu mat cam bien!\r\n");
  } else {
      LOG_SYS("[OK] VL53L0X (4 Sensors) Ready.\r\n");
  }

  if (MPU6500_Init(&hi2c2) != HAL_OK) {
      LOG_SYS("[WARN] MPU6500 Init that bai! Chay che do du phong Odometry.\r\n");
  } else {
      LOG_SYS("[OK] MPU6500 IMU (Gyroscope Z) Ready.\r\n");
  }

  // Khi nhan nut RESET: Xoa sach bo nho RAM & vo hieu hoa map cu trong Flash
  uint32_t invalid_header = 0x00000000;
  Flash_SaveData((uint8_t *)&invalid_header, sizeof(invalid_header));
  has_valid_speedrun_map = false;

  SensorFusion_Init();
  Motion_Init();
  Maze_Init();

  LOG_SYS("[RESET] Bo nho da xoa sach! Nhan nhanh MODE de bat dau do duong 16x16.\r\n");
  last_tick = HAL_GetTick();
  /* USER CODE END 2 */

  while (1)
  {
      uint32_t current_tick = HAL_GetTick();
      BT_Process();

      if (current_tick - last_tick >= CONTROL_LOOP_PERIOD_MS) {
          float dt = (float)(current_tick - last_tick) * 0.001f;
          if (dt > MAX_CONTROL_DT_SEC) {
              dt = (float)CONTROL_LOOP_PERIOD_MS * 0.001f;
          }
          last_tick = current_tick;

          static uint16_t heartbeat_cnt = 0;
          if (current_sys_state != SYS_STATE_FINISHED) {
              if (++heartbeat_cnt >= 50) {
                  heartbeat_cnt = 0;
                  HAL_GPIO_TogglePin(LED_STAT_GPIO_Port, LED_STAT_Pin);
              }
          } else {
              // Sang LED co dinh khi dang giu map & cho chay Speedrun
              HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_RESET);
          }

          float dist_l = 0.0f, dist_r = 0.0f;
          Encoder_GetDistanceMM(&dist_l, &dist_r);

          VL53L0X_Read_All(&vl_sensor_data);

          MPU6500_Read_GyroZ(dt);
          MPU6500_Data *mpu_data = MPU6500_GetData();

          SensorFusion_Update(&vl_sensor_data, mpu_data, dist_l, dist_r, dt);
          Motion_Update(dt);

          // Chi stream log UART 9600 baud khi dung cho o IDLE/ERROR (tranh nghen chu ky 10ms)
          static uint8_t print_cnt = 0;
          if (++print_cnt >= 10) {
              print_cnt = 0;
              if (g_bt_stream_enabled &&
                  (current_sys_state == SYS_STATE_IDLE || current_sys_state == SYS_STATE_ERROR)) {
                  LOG_SYS("FL:%4d | DL:%4d | DR:%4d | FR:%4d\r\n",
                          vl_sensor_data.front_l,
                          vl_sensor_data.diag_l,
                          vl_sensor_data.diag_r,
                          vl_sensor_data.front_r);
              }
          }

          Process_FSM(dt);
      }
  }
}

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
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

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
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
}

static void MX_I2C2_Init(void)
{
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
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 4;
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
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
}

static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
}

static void MX_TIM5_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
}

static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, XSHUT_L_Pin|XSHUT_FL_Pin|STBY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, XSHUT_FR_Pin|XSHUT_R_Pin|AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LED_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STAT_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SWITCH_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SWITCH_PIN_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = XSHUT_L_Pin|XSHUT_FL_Pin|STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = XSHUT_FR_Pin|XSHUT_R_Pin|AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
static uint8_t Is_Button_Pressed(void) {
    static uint8_t btn_last_state = 1;
    uint8_t current_state = HAL_GPIO_ReadPin(SWITCH_PIN_GPIO_Port, SWITCH_PIN_Pin);
    uint32_t now = HAL_GetTick();

    if (current_state == GPIO_PIN_RESET && btn_last_state == 1) {
        if (now - last_button_tick > DEBOUNCE_DELAY_MS) {
            last_button_tick = now;
            btn_last_state = 0;
            return 1;
        }
    } else if (current_state == GPIO_PIN_SET && btn_last_state == 0) {
        if (now - last_button_tick > DEBOUNCE_DELAY_MS) {
            btn_last_state = 1;
        }
    }
    return 0;
}

static void Process_FSM(float dt) {
    static Action current_exec_action = ACTION_STOP; // Lenh dang thuc thi / vua hoan tat
    static Action planned_action      = ACTION_STOP; // Lenh da chot tai moc 135mm cho tam o
    static bool cell_settled = false;
    static uint32_t cell_settle_tick = 0;
    static uint32_t countdown_start_tick = 0;
    static bool start_docking_done = false; // Co xac nhan da ty tuong 2 vach tai (0,0)
    static bool is_start_docking   = false; // Che do ty tuong dac biet tai (0,0): Tay -> Nam
    static uint8_t explore_passes  = 0;     // Dem so luot kham pha

    static uint8_t steps_since_dock = 0;
    static uint8_t turns_since_dock = 0;
    static bool need_docking = false;
    static bool pending_docking = false;

    static uint8_t dock_step = 0;
    static bool dock_side_left = true;
    static uint32_t dock_timer = 0;

    static SystemState dock_return_state = SYS_STATE_EXPLORE;
    static uint8_t fwd_steps_executed = 1;

    SensorFusion_Data *sf = SensorFusion_GetData();

    switch (current_sys_state) {
        case SYS_STATE_IDLE:
            if (Is_Button_Pressed()) {
                LOG_SYS("Nut MODE da nhan! Bat dau dem nguoc 2 giay on dinh...\r\n");
                countdown_start_tick = HAL_GetTick();
                current_sys_state = SYS_STATE_COUNTDOWN;
            }
            break;

        case SYS_STATE_COUNTDOWN:
            if (((HAL_GetTick() - countdown_start_tick) / 100) % 2 == 0) {
                HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_SET);
            }

            if (HAL_GetTick() - countdown_start_tick >= 2000) {
                LOG_SYS("Xe da on dinh! Dang can chinh sai so tinh Gyro-Z...\r\n");
                MPU6500_Calibrate();
                LOG_SYS("Xuat phat tham do sa ban 16x16 (Quay ngoat nhanh + Can tam ngay)!\r\n");

                Maze_Init();
                Encoder_Reset();
                SensorFusion_ResetEncoders();
                SensorFusion_ResetAngle();
                Motion_Init();
                Motion_SetFastMode(false);

                steps_since_dock = 0;
                turns_since_dock = 0;
                need_docking = false;
                pending_docking = false;
                dock_step = 0;
                explore_passes = 0;
                has_valid_speedrun_map = false;

                current_exec_action = ACTION_STOP;
                planned_action = ACTION_STOP;
                start_docking_done = false;
                is_start_docking = false;
                cell_settled = false;
                last_tick = HAL_GetTick();
                current_sys_state = SYS_STATE_EXPLORE;
            }
            break;

        // ==============================================================
        // CHE DO DO DUONG DEN TAM (EXPLORE) - TIEN SAT TUONG & QUAY NGOAT
        // ==============================================================
        case SYS_STATE_EXPLORE:
            if (Motion_IsCellCheckpointReady()) {
                Maze_UpdatePosition(ACTION_FORWARD);
                steps_since_dock++;
                if (steps_since_dock >= 12 || turns_since_dock >= 4) {
                    need_docking = true;
                }

                bool has_l = false, has_r = false;
                Motion_GetCellWallVotes(&has_l, &has_r);
                sf->wall_left  = has_l;
                sf->wall_right = has_r;
                Maze_UpdateWalls(sf);

                if (Maze_IsGoalReached()) {
                    Maze_OpenCenterGoalBox();
                    planned_action = ACTION_STOP;
                    current_exec_action = ACTION_FORWARD;
                    Motion_CommitStopAtCenter();
                    break;
                }

                Action next = Maze_GetNextAction();

                bool front_confirmed = (vl_sensor_data.front_l > 10 && vl_sensor_data.front_l <= 125 &&
                                        vl_sensor_data.front_r > 10 && vl_sensor_data.front_r <= 125 &&
                                        fabsf((float)vl_sensor_data.front_l - (float)vl_sensor_data.front_r) <= 35.0f);

                if (need_docking && front_confirmed && !Maze_IsGoalReached()) {
                    if (sf->wall_left && sf->wall_right && next == ACTION_TURN_AROUND) {
                        dock_side_left = true;
                        pending_docking = true;
                        planned_action = ACTION_STOP;
                        current_exec_action = ACTION_FORWARD;
                        Motion_CommitStopAtCenter();
                        break;
                    } else if (sf->wall_left && next == ACTION_TURN_RIGHT) {
                        dock_side_left = true;
                        pending_docking = true;
                        planned_action = ACTION_STOP;
                        current_exec_action = ACTION_FORWARD;
                        Motion_CommitStopAtCenter();
                        break;
                    } else if (sf->wall_right && next == ACTION_TURN_LEFT) {
                        dock_side_left = false;
                        pending_docking = true;
                        planned_action = ACTION_STOP;
                        current_exec_action = ACTION_FORWARD;
                        Motion_CommitStopAtCenter();
                        break;
                    }
                }

                if (next == ACTION_FORWARD) {
                    current_exec_action = ACTION_FORWARD;
                    planned_action = ACTION_STOP;
                    Motion_ContinueNextCell();
                } else {
                    // Tien sat tuong truoc (44mm) / tam o roi quay ngoat nhanh
                    planned_action = next;
                    current_exec_action = ACTION_FORWARD;
                    Motion_CommitStopAtCenter();
                }
                break;
            }

            if (Motion_IsFinished()) {
                uint32_t settle_wait = (current_exec_action == ACTION_TURN_LEFT ||
                                        current_exec_action == ACTION_TURN_RIGHT) ? 10 : 20;
                if (!cell_settled) {
                    cell_settled = true;
                    cell_settle_tick = HAL_GetTick();
                    break;
                }
                if (HAL_GetTick() - cell_settle_tick < settle_wait) {
                    break;
                }
                cell_settled = false;

                if (Maze_IsGoalReached()) {
                    Motion_Stop();
                    Maze_OpenCenterGoalBox();
                    explore_passes++;

                    bool optimal_now = Maze_IsOptimalPathFound();
                    LOG_SYS("[GOAL] Cham tam 16x16 (Luot %d)! Lo trinh toi uu: %s\r\n",
                            explore_passes, optimal_now ? "DA DAT" : "CHUA - tiep tuc quet chieu ve");

                    for (int i = 0; i < 10; i++) {
                        HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_RESET);
                        HAL_Delay(80);
                        HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_SET);
                        HAL_Delay(80);
                    }
                    last_tick = HAL_GetTick();

                    Maze_SetTargetGoal(false);
                    current_sys_state = SYS_STATE_RETURN;
                    cell_settled = false;
                    need_docking = true;
                    pending_docking = false;

                    planned_action = Maze_GetNextAction();
                    current_exec_action = planned_action;
                    planned_action = ACTION_STOP;

                    switch (current_exec_action) {
                        case ACTION_FORWARD:     Motion_StartContinuousForward(); break;
                        case ACTION_TURN_LEFT:    Motion_Rotate(90.0f); break;
                        case ACTION_TURN_RIGHT:   Motion_Rotate(-90.0f); break;
                        case ACTION_TURN_AROUND:  Motion_Rotate(-180.0f); break;
                        default: break;
                    }
                    break;
                }

                if (pending_docking) {
                    pending_docking = false;
                    need_docking = false;
                    steps_since_dock = 0;
                    turns_since_dock = 0;
                    dock_step = 1;
                    dock_return_state = SYS_STATE_EXPLORE;
                    current_sys_state = SYS_STATE_DOCKING;

                    RobotPosition cur_pos = Maze_GetPosition();
                    LOG_SYS("[DOCK] Kich hoat ty 2 vach tai (%d,%d).\r\n", cur_pos.x, cur_pos.y);
                    Motion_Rotate(dock_side_left ? 180.0f : -180.0f);
                    break;
                }

                if (current_exec_action == ACTION_TURN_LEFT ||
                    current_exec_action == ACTION_TURN_RIGHT ||
                    current_exec_action == ACTION_TURN_AROUND) {
                    Maze_UpdatePosition(current_exec_action);
                    if (current_exec_action != ACTION_TURN_AROUND) {
                        turns_since_dock++;
                        if (turns_since_dock >= 4) need_docking = true;
                    }
                    if (sf->wall_front) {
                        Maze_UpdateWalls(sf);
                    }
                    planned_action = Maze_GetNextAction();
                } else if (current_exec_action == ACTION_STOP) {
                    Maze_UpdateWalls(sf);
                    planned_action = Maze_GetNextAction();
                } else if (current_exec_action == ACTION_FORWARD && planned_action == ACTION_STOP) {
                    Maze_UpdateWalls(sf);
                    planned_action = Maze_GetNextAction();
                }

                current_exec_action = planned_action;
                planned_action = ACTION_STOP;

                switch (current_exec_action) {
                    case ACTION_FORWARD:
                        Motion_StartContinuousForward();
                        break;
                    case ACTION_TURN_LEFT:
                        Motion_Rotate(90.0f);
                        break;
                    case ACTION_TURN_RIGHT:
                        Motion_Rotate(-90.0f);
                        break;
                    case ACTION_TURN_AROUND:
                        Motion_Rotate(-180.0f);
                        break;
                    case ACTION_STOP:
                    default:
                        if (!Maze_IsGoalReached()) {
                            current_exec_action = ACTION_TURN_AROUND;
                            Motion_Rotate(-180.0f);
                        } else {
                            Motion_Stop();
                            current_sys_state = SYS_STATE_FINISHED;
                        }
                        break;
                }
            }
            break;

        // ==============================================================
        // CHE DO CAN CHINH CO HOC 2 VACH (DUAL-WALL DOCKING)
        // ==============================================================
        case SYS_STATE_DOCKING:
            switch (dock_step) {
                case 1:
                    if (Motion_IsFinished()) {
                        TB6612_SetSpeed(-(int16_t)DOCK_REVERSE_PWM, -(int16_t)DOCK_REVERSE_PWM);
                        dock_timer = HAL_GetTick();
                        dock_step = 2;
                    }
                    break;

                case 2:
                    if (HAL_GetTick() - dock_timer >= DOCK_PRESS_TIME_MS) {
                        TB6612_Brake();
                        dock_timer = HAL_GetTick();
                        dock_step = 25;
                    }
                    break;

                case 25:
                    TB6612_Brake();
                    if (HAL_GetTick() - dock_timer >= 65) {
                        Motion_SnapAngleToCardinal();
                        Motion_MoveForward(DOCK_RETURN_CENTER_MM);
                        dock_step = 3;
                    }
                    break;

                case 3:
                    if (Motion_IsFinished()) {
                        if (is_start_docking) {
                            Motion_Rotate(90.0f);
                        } else {
                            Motion_Rotate(dock_side_left ? 90.0f : -90.0f);
                        }
                        dock_step = 4;
                    }
                    break;

                case 4:
                    if (Motion_IsFinished()) {
                        TB6612_SetSpeed(-(int16_t)DOCK_REVERSE_PWM, -(int16_t)DOCK_REVERSE_PWM);
                        dock_timer = HAL_GetTick();
                        dock_step = 5;
                    }
                    break;

                case 5:
                    if (HAL_GetTick() - dock_timer >= DOCK_PRESS_TIME_MS) {
                        TB6612_Brake();
                        dock_timer = HAL_GetTick();
                        dock_step = 55;
                    }
                    break;

                case 55:
                    TB6612_Brake();
                    if (HAL_GetTick() - dock_timer >= 65) {
                        Motion_SnapAngleToCardinal();
                        Motion_MoveForward(DOCK_RETURN_CENTER_MM);
                        dock_step = 6;
                    }
                    break;

                case 6:
                    if (Motion_IsFinished()) {
                        static uint32_t dock_align_start = 0;
                        if (dock_align_start == 0) {
                            dock_align_start = HAL_GetTick();
                        }

                        if (!Motion_AlignHeadingStatic() && (HAL_GetTick() - dock_align_start < 200)) {
                            break;
                        }
                        dock_align_start = 0;

                        TB6612_Brake();
                        dock_timer = HAL_GetTick();
                        dock_step = 7;
                    }
                    break;

                case 7:
                    TB6612_Brake();
                    if (HAL_GetTick() - dock_timer >= 120) {
                        Motion_ResetEncodersAtCenter();

                        if (is_start_docking) {
                            is_start_docking = false;
                            RobotPosition cur_r = Maze_GetPosition();
                            if (cur_r.dir == DIR_SOUTH)      Maze_UpdatePosition(ACTION_TURN_AROUND);
                            else if (cur_r.dir == DIR_EAST)  Maze_UpdatePosition(ACTION_TURN_LEFT);
                            else if (cur_r.dir == DIR_WEST)  Maze_UpdatePosition(ACTION_TURN_RIGHT);

                            LOG_SYS("[START-DOCK] Da can chuan 2 vach Tay & Nam tai (0,0) huong BAC!\r\n");

                            current_sys_state = SYS_STATE_RETURN;
                            cell_settled = true;
                            cell_settle_tick = HAL_GetTick() - 30;
                            dock_step = 0;
                            current_exec_action = ACTION_STOP;
                            planned_action = ACTION_STOP;
                            break;
                        }

                        Maze_UpdatePosition(dock_side_left ? ACTION_TURN_RIGHT : ACTION_TURN_LEFT);

                        if (sf->wall_front) {
                            Maze_UpdateWalls(sf);
                        }

                        LOG_SYS("[DOCK] Da can 2 vach & on dinh tai tam o! Xuat phat...\r\n");

                        current_sys_state = dock_return_state;
                        cell_settled = false;
                        dock_step = 0;

                        current_exec_action = Maze_GetNextAction();
                        planned_action = ACTION_STOP;

                        switch (current_exec_action) {
                            case ACTION_FORWARD:     Motion_StartContinuousForward(); break;
                            case ACTION_TURN_LEFT:    Motion_Rotate(90.0f); break;
                            case ACTION_TURN_RIGHT:   Motion_Rotate(-90.0f); break;
                            case ACTION_TURN_AROUND:  Motion_Rotate(-180.0f); break;
                            default:                  Motion_Stop(); break;
                        }
                    }
                    break;
            }
            break;

        // ==============================================================
        // CHE DO DO DUONG CHIEU VE (0,0) - TIEN SAT TUONG & QUAY NGOAT
        // ==============================================================
        case SYS_STATE_RETURN:
            if (Motion_IsCellCheckpointReady()) {
                Maze_UpdatePosition(ACTION_FORWARD);
                steps_since_dock++;
                if (steps_since_dock >= 8 || turns_since_dock >= 3) {
                    need_docking = true;
                }

                bool has_l = false, has_r = false;
                Motion_GetCellWallVotes(&has_l, &has_r);
                sf->wall_left  = has_l;
                sf->wall_right = has_r;
                Maze_UpdateWalls(sf);

                if (Maze_IsAtStart()) {
                    planned_action = ACTION_STOP;
                    current_exec_action = ACTION_FORWARD;
                    Motion_CommitStopAtCenter();
                    break;
                }

                Action next = Maze_GetNextAction();

                RobotPosition ret_pos = Maze_GetPosition();
                bool in_center_box = (ret_pos.x >= 7 && ret_pos.x <= 8 && ret_pos.y >= 7 && ret_pos.y <= 8);

                bool front_confirmed = (vl_sensor_data.front_l > 10 && vl_sensor_data.front_l <= 125 &&
                                        vl_sensor_data.front_r > 10 && vl_sensor_data.front_r <= 125 &&
                                        fabsf((float)vl_sensor_data.front_l - (float)vl_sensor_data.front_r) <= 35.0f);

                if (need_docking && front_confirmed && !in_center_box) {
                    if (sf->wall_left && sf->wall_right && next == ACTION_TURN_AROUND) {
                        dock_side_left = true;
                        pending_docking = true;
                        planned_action = ACTION_STOP;
                        current_exec_action = ACTION_FORWARD;
                        Motion_CommitStopAtCenter();
                        break;
                    } else if (sf->wall_left && next == ACTION_TURN_RIGHT) {
                        dock_side_left = true;
                        pending_docking = true;
                        planned_action = ACTION_STOP;
                        current_exec_action = ACTION_FORWARD;
                        Motion_CommitStopAtCenter();
                        break;
                    } else if (sf->wall_right && next == ACTION_TURN_LEFT) {
                        dock_side_left = false;
                        pending_docking = true;
                        planned_action = ACTION_STOP;
                        current_exec_action = ACTION_FORWARD;
                        Motion_CommitStopAtCenter();
                        break;
                    }
                }

                if (next == ACTION_FORWARD) {
                    current_exec_action = ACTION_FORWARD;
                    planned_action = ACTION_STOP;
                    Motion_ContinueNextCell();
                } else {
                    planned_action = next;
                    current_exec_action = ACTION_FORWARD;
                    Motion_CommitStopAtCenter();
                }
                break;
            }

            if (Motion_IsFinished()) {
                uint32_t settle_wait = (current_exec_action == ACTION_TURN_LEFT ||
                                        current_exec_action == ACTION_TURN_RIGHT) ? 10 : 20;
                if (!cell_settled) {
                    cell_settled = true;
                    cell_settle_tick = HAL_GetTick();
                    break;
                }
                if (HAL_GetTick() - cell_settle_tick < settle_wait) {
                    break;
                }
                cell_settled = false;

                if (Maze_IsAtStart()) {
                    if (current_exec_action == ACTION_TURN_LEFT ||
                        current_exec_action == ACTION_TURN_RIGHT ||
                        current_exec_action == ACTION_TURN_AROUND) {
                        Maze_UpdatePosition(current_exec_action);
                        current_exec_action = ACTION_STOP;
                    }

                    RobotPosition r_pos = Maze_GetPosition();

                    if (!start_docking_done) {
                        start_docking_done = true;
                        is_start_docking = true;
                        planned_action = ACTION_STOP;
                        dock_step = 1;
                        dock_return_state = SYS_STATE_RETURN;
                        current_sys_state = SYS_STATE_DOCKING;

                        LOG_SYS("[START] Da ve (0,0)! Bat dau ty tuong Tay -> Nam chuan bi Speedrun...\r\n");

                        if (r_pos.dir == DIR_SOUTH) {
                            Motion_Rotate(90.0f);
                        } else if (r_pos.dir == DIR_NORTH) {
                            Motion_Rotate(-90.0f);
                        } else if (r_pos.dir == DIR_WEST) {
                            Motion_Rotate(180.0f);
                        }
                        break;
                    }

                    if (!Maze_IsOptimalPathFound() && explore_passes < MAX_EXPLORE_PASSES) {
                        LOG_SYS("[START] Con nhanh tat chua quet! Chay luot kiem tra 2...\r\n");
                        start_docking_done = false;
                        steps_since_dock = 0;
                        turns_since_dock = 0;
                        need_docking = false;
                        pending_docking = false;

                        Maze_SetTargetGoal(true);
                        current_sys_state = SYS_STATE_EXPLORE;

                        planned_action = Maze_GetNextAction();
                        current_exec_action = planned_action;
                        planned_action = ACTION_STOP;

                        switch (current_exec_action) {
                            case ACTION_FORWARD:     Motion_StartContinuousForward(); break;
                            case ACTION_TURN_LEFT:    Motion_Rotate(90.0f); break;
                            case ACTION_TURN_RIGHT:   Motion_Rotate(-90.0f); break;
                            case ACTION_TURN_AROUND:  Motion_Rotate(-180.0f); break;
                            default: break;
                        }
                        break;
                    }

                    LOG_SYS("[START] San sang Speedrun 16x16! Dang tinh Turn-Penalty & luu Flash...\r\n");
                    Maze_SetTargetGoal(true);
                    Maze_ComputeTurnPenaltyFloodfill();
                    if (Maze_SaveToFlash()) {
                        has_valid_speedrun_map = true;
                        LOG_SYS("[FLASH] Da luu map thanh cong! Nhan nhanh MODE de chay Speedrun.\r\n");
                    } else {
                        LOG_SYS("[ERR] Loi luu Flash!\r\n");
                    }

                    Motion_Stop();
                    HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_RESET);
                    current_sys_state = SYS_STATE_FINISHED;
                    break;
                }

                if (pending_docking) {
                    pending_docking = false;
                    need_docking = false;
                    steps_since_dock = 0;
                    turns_since_dock = 0;
                    dock_step = 1;
                    dock_return_state = SYS_STATE_RETURN;
                    current_sys_state = SYS_STATE_DOCKING;

                    Motion_Rotate(dock_side_left ? 180.0f : -180.0f);
                    break;
                }

                if (current_exec_action == ACTION_TURN_LEFT ||
                    current_exec_action == ACTION_TURN_RIGHT ||
                    current_exec_action == ACTION_TURN_AROUND) {
                    Maze_UpdatePosition(current_exec_action);
                    if (current_exec_action != ACTION_TURN_AROUND) {
                        turns_since_dock++;
                        if (turns_since_dock >= 3) need_docking = true;
                    }
                    if (sf->wall_front) {
                        Maze_UpdateWalls(sf);
                    }
                    planned_action = Maze_GetNextAction();
                } else if (current_exec_action == ACTION_FORWARD && planned_action == ACTION_STOP) {
                    Maze_UpdateWalls(sf);
                    planned_action = Maze_GetNextAction();
                }

                current_exec_action = planned_action;
                planned_action = ACTION_STOP;

                switch (current_exec_action) {
                    case ACTION_FORWARD:     Motion_StartContinuousForward(); break;
                    case ACTION_TURN_LEFT:    Motion_Rotate(90.0f); break;
                    case ACTION_TURN_RIGHT:   Motion_Rotate(-90.0f); break;
                    case ACTION_TURN_AROUND:  Motion_Rotate(-180.0f); break;
                    default:
                        Motion_Stop();
                        current_sys_state = SYS_STATE_FINISHED;
                        break;
                }
            }
            break;

        // ==============================================================
        // CHE DO CHAY TOC DO CAO (FAST RUN 16x16 - QUAY NGOAT & CAN TAM)
        // ==============================================================
        case SYS_STATE_FAST_RUN:
            if (Is_Button_Pressed()) {
                Motion_Stop();
                Motion_SetFastMode(false);
                current_exec_action = ACTION_STOP;
                planned_action = ACTION_STOP;
                cell_settled = false;
                fwd_steps_executed = 1;
                HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_RESET);
                LOG_SYS("[FAST-STOP] Da dung luot Speedrun! Map van duoc bao toan trong Flash.\r\n");
                LOG_SYS("-> Dat xe ve (0,0) huong BAC va nhan nhanh MODE de chay luot Speedrun moi.\r\n");
                current_sys_state = SYS_STATE_FINISHED;
                break;
            }

            if (current_exec_action != ACTION_STOP &&
                sf->front_dist > 10.0f && sf->front_dist <= CRASH_DIST_LIMIT_MM) {
                Motion_Stop();
                Motion_SetFastMode(false);
                current_exec_action = ACTION_STOP;
                planned_action = ACTION_STOP;
                cell_settled = false;
                fwd_steps_executed = 1;
                HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_RESET);
                LOG_SYS("[FAST-CRASH] Phat hien vat can gan (%.0fmm)! Da ngat luot & giu map Flash.\r\n", sf->front_dist);
                current_sys_state = SYS_STATE_FINISHED;
                break;
            }

            if (Motion_IsFinished()) {
                // Chi nghi 10ms (1 chu ky) sau quay ngoat 90 do de xe lao thang ngay vao tim o moi
                uint32_t fast_settle = (current_exec_action == ACTION_TURN_AROUND) ? 20 : 10;
                if (!cell_settled) {
                    cell_settled = true;
                    cell_settle_tick = HAL_GetTick();
                    break;
                }
                if (HAL_GetTick() - cell_settle_tick < fast_settle) {
                    break;
                }
                cell_settled = false;

                if (current_exec_action == ACTION_FORWARD) {
                    for (uint8_t i = 0; i < fwd_steps_executed; i++) {
                        Maze_UpdatePosition(ACTION_FORWARD);
                    }
                } else {
                    Maze_UpdatePosition(current_exec_action);
                }

                if (Maze_IsGoalReached()) {
                    Motion_Stop();
                    Motion_SetFastMode(false);
                    current_exec_action = ACTION_STOP;
                    planned_action = ACTION_STOP;
                    cell_settled = false;
                    fwd_steps_executed = 1;
                    LOG_SYS("Hoan thanh luot Fast Run 16x16 thanh cong! Nhan MODE neu muon chay lai.\r\n");
                    HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_RESET);
                    current_sys_state = SYS_STATE_FINISHED;
                    break;
                }

                current_exec_action = Maze_GetNextAction();

                switch (current_exec_action) {
                    case ACTION_FORWARD:
                        fwd_steps_executed = Maze_GetConsecutiveForwardSteps();
                        Motion_MoveForward(FULL_CELL * fwd_steps_executed);
                        break;
                    case ACTION_TURN_LEFT:
                        fwd_steps_executed = 1;
                        Motion_Rotate(90.0f);
                        break;
                    case ACTION_TURN_RIGHT:
                        fwd_steps_executed = 1;
                        Motion_Rotate(-90.0f);
                        break;
                    case ACTION_TURN_AROUND:
                        fwd_steps_executed = 1;
                        Motion_Rotate(-180.0f);
                        break;
                    default:
                        Motion_Stop();
                        Motion_SetFastMode(false);
                        current_sys_state = SYS_STATE_ERROR;
                        break;
                }
            }
            break;

        // ==============================================================
        // CHE DO SAN SANG SPEEDRUN (GIU MAP TRONG FLASH)
        // ==============================================================
        case SYS_STATE_FINISHED:
            if (Is_Button_Pressed()) {
                LOG_SYS("Nut MODE da nhan! Chuan bi Speedrun 16x16 trong 1 giay...\r\n");
                for (int i = 0; i < 5; i++) {
                    HAL_GPIO_TogglePin(LED_STAT_GPIO_Port, LED_STAT_Pin);
                    HAL_Delay(100);
                    HAL_GPIO_TogglePin(LED_STAT_GPIO_Port, LED_STAT_Pin);
                    HAL_Delay(100);
                }

                Maze_Init();
                if (Maze_LoadFromFlash()) {
                    has_valid_speedrun_map = true;
                    Encoder_Reset();
                    SensorFusion_ResetEncoders();
                    SensorFusion_ResetAngle();
                    MPU6500_Calibrate();
                    Motion_Init();
                    Motion_SetFastMode(true);

                    current_exec_action = ACTION_STOP;
                    planned_action = ACTION_STOP;
                    cell_settled = false;
                    fwd_steps_executed = 1;
                    last_tick = HAL_GetTick();
                    current_sys_state = SYS_STATE_FAST_RUN;
                } else {
                    LOG_SYS("[ERR] Khong tim thay map hop le trong Flash! Chuyen ve IDLE.\r\n");
                    has_valid_speedrun_map = false;
                    current_sys_state = SYS_STATE_IDLE;
                }
            }
            break;

        case SYS_STATE_ERROR:
        default:
            Motion_Stop();
            Motion_SetFastMode(false);
            if (has_valid_speedrun_map && Is_Button_Pressed()) {
                current_exec_action = ACTION_STOP;
                planned_action = ACTION_STOP;
                cell_settled = false;
                fwd_steps_executed = 1;
                HAL_GPIO_WritePin(LED_STAT_GPIO_Port, LED_STAT_Pin, GPIO_PIN_RESET);
                LOG_SYS("[RECOVER] Da phuc hoi ve che do chuan bi Speedrun (con luu map).\r\n");
                current_sys_state = SYS_STATE_FINISHED;
            }
            break;
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

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif