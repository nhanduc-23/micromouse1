#include "encoder.h"
#include "config.h"

static TIM_HandleTypeDef *tim_l;
static TIM_HandleTypeDef *tim_r;

void Encoder_Init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right) {
    tim_l = htim_left;
    tim_r = htim_right;
	
	//Kich hoat che do dem xung phan cung Encoder tren ca 2 kenh A va kenh B
	HAL_TIM_Encoder_Start(tim_l, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(tim_r, TIM_CHANNEL_ALL);
	Encoder_Reset();
}


void Encoder_GetCounters(int16_t *cnt_left, int16_t *cnt_right) {

	//Ep kieu thanh ghi TIM --> CNY 16 - bit ve int16_t de xu ly so am khi banh quay lui
	*cnt_left = (int16_t)__HAL_TIM_GET_COUNTER(tim_l);
	*cnt_right = (int16_t)__HAL_TIM_GET_COUNTER(tim_r);
}
void Encoder_Reset(void){
	__HAL_TIM_SET_COUNTER(tim_l, 0);
  __HAL_TIM_SET_COUNTER(tim_r, 0);
}

// Bo sung ham doi xung sang mm dua tren cau hinh co khi
void Encoder_GetDistanceMM(float *dist_left_mm, float *dist_right_mm) {
    int16_t cnt_l = 0, cnt_r = 0;
    Encoder_GetCounters(&cnt_l, &cnt_r);
    
    *dist_left_mm  = (float)cnt_l * MM_PER_COUNT;
    *dist_right_mm = (float)cnt_r * MM_PER_COUNT;
}