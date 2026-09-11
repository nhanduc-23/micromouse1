#include "encoder.h"
#include "config.h"

static TIM_HandleTypeDef *tim_l;
static TIM_HandleTypeDef *tim_r;

// Bien cong don 32-bit chong tran so tuyet doi cho ca 2 banh
static int32_t total_cnt_l = 0;
static int32_t total_cnt_r = 0;
static uint16_t last_raw_l = 0;
static uint16_t last_raw_r = 0;

void Encoder_Init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right) {
    tim_l = htim_left;
    tim_r = htim_right;

    HAL_TIM_Encoder_Start(tim_l, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(tim_r, TIM_CHANNEL_ALL);

    Encoder_Reset();
}

void Encoder_GetCounters(int32_t *cnt_left, int32_t *cnt_right) {
    uint16_t cur_raw_l = (uint16_t)__HAL_TIM_GET_COUNTER(tim_l);
    uint16_t cur_raw_r = (uint16_t)__HAL_TIM_GET_COUNTER(tim_r);

    // Ep kieu (int16_t) chenh lech se tu dong bu tran so quay vòog 16-bit (0 <-> 65535)
    int16_t delta_l = (int16_t)(cur_raw_l - last_raw_l);
    int16_t delta_r = (int16_t)(cur_raw_r - last_raw_r);

    total_cnt_l += delta_l;
    total_cnt_r += delta_r;

    last_raw_l = cur_raw_l;
    last_raw_r = cur_raw_r;

    *cnt_left  = total_cnt_l;
    *cnt_right = total_cnt_r;
}

void Encoder_Reset(void) {
    // Reset phan mem: Chi dat lai goc toa do bien cong don, khong can thiep thanh ghi Timer
    last_raw_l = (uint16_t)__HAL_TIM_GET_COUNTER(tim_l);
    last_raw_r = (uint16_t)__HAL_TIM_GET_COUNTER(tim_r);
    total_cnt_l = 0;
    total_cnt_r = 0;
}

void Encoder_GetDistanceMM(float *dist_left_mm, float *dist_right_mm) {
    int32_t cnt_l = 0, cnt_r = 0;
    Encoder_GetCounters(&cnt_l, &cnt_r);

    *dist_left_mm  = (float)cnt_l * MM_PER_COUNT;
    *dist_right_mm = (float)cnt_r * MM_PER_COUNT;
}