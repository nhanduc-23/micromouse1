#include "encoder.h"
#include "config.h"

/* ========================================================================== */
/* [ĐIỂM SỬA 1]: HỆ SỐ CỰC TÍNH CHIỀU ĐẾM ENCODER KHI XE TIẾN                 */
/* (+1: Đếm tăng khi bánh lăn tiến, -1: Đảo chiều đếm nếu bị đếm lùi)          */
/* ========================================================================== */
#define ENCODER_L_POLARITY   (-1)   // Bánh Trái (Động cơ cũ bên phải cắm dây TIM3)
#define ENCODER_R_POLARITY   ( 1)   // Bánh Phải (Động cơ cũ bên trái cắm dây TIM2)

static TIM_HandleTypeDef *tim_l;
static TIM_HandleTypeDef *tim_r;

// Biến đếm quãng đường tích lũy (xung)
static int32_t total_cnt_l = 0;
static int32_t total_cnt_r = 0;

/* ========================================================================== */
/* [ĐIỂM SỬA 2]: HOÁN ĐỔI KIỂU DỮ LIỆU THANH GHI CHO ĐÚNG ĐỘ RỘNG BIT        */
/* - Bánh Trái nay là TIM3 (16-bit -> uint16_t)                               */
/* - Bánh Phải nay là TIM2 (32-bit -> uint32_t)                               */
/* ========================================================================== */
static uint16_t last_raw_l = 0; // TIM3 (16-bit)
static uint32_t last_raw_r = 0; // TIM2 (32-bit)

static int32_t last_delta_l = 0;
static int32_t last_delta_r = 0;

void Encoder_Init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right) {
    /* ====================================================================== */
    /* [ĐIỂM SỬA 3]: HOÁN ĐỔI TIMER PHẦN CỨNG                                 */
    /* Trong main.c: truyền vào htim2 (dây Trái cũ) và htim3 (dây Phải cũ).   */
    /* Do tráo động cơ: Bánh Trái mới dùng htim3, Bánh Phải mới dùng htim2.   */
    /* ====================================================================== */
    tim_l = htim_right; // TIM3 (dây cắm cũ bên phải nay là bánh Trái)
    tim_r = htim_left;  // TIM2 (dây cắm cũ bên trái nay là bánh Phải)

    HAL_TIM_Encoder_Start(tim_l, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(tim_r, TIM_CHANNEL_ALL);

    Encoder_Reset();
}

static void Encoder_Update(void) {
    // Đọc trực tiếp thanh ghi CNT từ 2 Timer phần cứng
    uint16_t cur_raw_l = (uint16_t)__HAL_TIM_GET_COUNTER(tim_l); // TIM3 (16-bit)
    uint32_t cur_raw_r = __HAL_TIM_GET_COUNTER(tim_r);           // TIM2 (32-bit)

    /* ====================================================================== */
    /* [ĐIỂM SỬA 4]: XỬ LÝ TRÀN SỐ PHÙ HỢP VỚI ĐỘ RỘNG THANH GHI MỚI          */
    /* TIM3 là 16-bit: ép kiểu (int16_t) để tự động bù tràn số khi qua 65535  */
    /* TIM2 là 32-bit: ép kiểu (int32_t) để tự động bù tràn số khi qua 2^32-1 */
    /* ====================================================================== */
    int16_t delta_l = (int16_t)(cur_raw_l - last_raw_l) * ENCODER_L_POLARITY;
    int32_t delta_r = (int32_t)(cur_raw_r - last_raw_r) * ENCODER_R_POLARITY;

    total_cnt_l += (int32_t)delta_l;
    total_cnt_r += delta_r;

    last_delta_l = (int32_t)delta_l;
    last_delta_r = delta_r;

    last_raw_l = cur_raw_l;
    last_raw_r = cur_raw_r;
}

void Encoder_GetCounters(int32_t *cnt_left, int32_t *cnt_right) {
    Encoder_Update();
    if (cnt_left)  *cnt_left  = total_cnt_l;
    if (cnt_right) *cnt_right = total_cnt_r;
}

void Encoder_GetDelta(int32_t *delta_left, int32_t *delta_right) {
    Encoder_Update();
    if (delta_left)  *delta_left  = last_delta_l;
    if (delta_right) *delta_right = last_delta_r;
}

void Encoder_GetDeltaMM(float *d_left_mm, float *d_right_mm) {
    int32_t d_l = 0, d_r = 0;
    Encoder_GetDelta(&d_l, &d_r);

    if (d_left_mm)  *d_left_mm  = (float)d_l * MM_PER_COUNT;
    if (d_right_mm) *d_right_mm = (float)d_r * MM_PER_COUNT;
}

void Encoder_GetDistanceMM(float *dist_left_mm, float *dist_right_mm) {
    int32_t cnt_l = 0, cnt_r = 0;
    Encoder_GetCounters(&cnt_l, &cnt_r);

    if (dist_left_mm)  *dist_left_mm  = (float)cnt_l * MM_PER_COUNT;
    if (dist_right_mm) *dist_right_mm = (float)cnt_r * MM_PER_COUNT;
}

void Encoder_Reset(void) {
    last_raw_l = (uint16_t)__HAL_TIM_GET_COUNTER(tim_l);
    last_raw_r = __HAL_TIM_GET_COUNTER(tim_r);
    total_cnt_l  = 0;
    total_cnt_r  = 0;
    last_delta_l = 0;
    last_delta_r = 0;
}