#include "tb6612fng.h"

static TIM_HandleTypeDef *pwmtim;

void TB6612_Init(TIM_HandleTypeDef *htim){
		 pwmtim = htim;
	// 1. Kich hoat xuat xung PWM tren Ch1(PA8) va chan Ch 2 (PA9) cua TIM 1
		 HAL_TIM_PWM_Start(pwmtim, TIM_CHANNEL_1);
	   HAL_TIM_PWM_Start(pwmtim, TIM_CHANNEL_2);
	
	//2. Keo chan STBY (PA5) len High de thoat che do cho tiet kiem dien
		 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}
void TB6612_SetSpeed(int16_t speed_left, int16_t speed_right){
	// banh trai (motor A : AIN1 = PB12, AIN2 = PB13, PWM = PA8)
	if(speed_left >= 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);	
		
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		speed_left = -speed_left; // doi dau am thnah duong de nap do rong xung
	}
	//gioi han cong suat toi da theo chu ky ARR
	if(speed_left > 1000) speed_left = 1000;
	__HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_1, (uint16_t)speed_left);
	
	
	//Banh phai (motor B: BIN1=PB14, BIN2 = PB15, PWM = PA9)
	if(speed_right > 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		speed_right = - speed_right;
	}
	if(speed_right > 1000) speed_right = 1000;
		__HAL_TIM_SET_COMPARE(pwmtim, TIM_CHANNEL_2, (uint16_t)speed_right);
	
	
}

void TB6612_Stop(void){
	TB6612_SetSpeed(0, 0);
}

