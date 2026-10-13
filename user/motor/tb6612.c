#include "tb6612.h"
#include "tim.h"
#include <math.h>

#define REVERSE_PAUSE_MS 500U

static uint8_t s_ready = 0U;
static uint8_t s_stopped = 1U;
static int8_t s_last_direction = 0;
static uint32_t s_stop_tick = 0U;

void TB6612_Stop(void)
{
    /* STBY low releases both outputs; the motor coasts. */
    HAL_GPIO_WritePin(TB_STBY_GPIO_Port, TB_STBY_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    HAL_GPIO_WritePin(TB_AIN1_GPIO_Port, TB_AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TB_AIN2_GPIO_Port, TB_AIN2_Pin, GPIO_PIN_RESET);

    /* Do not restart the pause timer on every stopped task cycle. */
    if (s_stopped == 0U) {
        s_stop_tick = HAL_GetTick();
        s_stopped = 1U;
    }
}

HAL_StatusTypeDef TB6612_Init(void)
{
    s_ready = 0U;
    s_stopped = 1U;
    s_last_direction = 0;
    s_stop_tick = HAL_GetTick();

    TB6612_Stop();
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    HAL_StatusTypeDef status = HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    if (status != HAL_OK) return status;

    status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    if (status == HAL_OK) {
        s_ready = 1U;
    }
    return status;
}

float TB6612_SetDuty(float duty)
{
    if ((s_ready == 0U) || !isfinite(duty)) {
        TB6612_Stop();
        return 0.0f;
    }

    if (duty > TB6612_MAX_DUTY) duty = TB6612_MAX_DUTY;
    if (duty < -TB6612_MAX_DUTY) duty = -TB6612_MAX_DUTY;

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    float magnitude = (duty >= 0.0f) ? duty : -duty;
    uint32_t compare = (uint32_t)(magnitude * (float)period + 0.5f);

    if (compare == 0U) {
        TB6612_Stop();
        return 0.0f;
    }

    int8_t direction = (duty > 0.0f) ? 1 : -1;
    if ((s_last_direction != 0) && (direction != s_last_direction)) {
        TB6612_Stop();
        if ((uint32_t)(HAL_GetTick() - s_stop_tick) < REVERSE_PAUSE_MS) {
            return 0.0f;
        }
    }

    if (s_stopped != 0U) {
        /* Change direction while STBY is still low. */
        HAL_GPIO_WritePin(TB_AIN1_GPIO_Port, TB_AIN1_Pin,
                          (direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TB_AIN2_GPIO_Port, TB_AIN2_Pin,
                          (direction > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
        HAL_GPIO_WritePin(TB_STBY_GPIO_Port, TB_STBY_Pin, GPIO_PIN_SET);
        s_last_direction = direction;
        s_stopped = 0U;
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
    }

    return ((float)compare / (float)period) * (float)direction;
}