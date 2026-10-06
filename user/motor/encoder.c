#include "encoder.h"
#include <stddef.h>

HAL_StatusTypeDef Encoder_Start(Encoder_t *encoder, TIM_HandleTypeDef *tim,
                                float counts_per_rev, int8_t direction)
{
    if (encoder == NULL || tim == NULL || counts_per_rev <= 0.0f) {
        return HAL_ERROR;
    }

    encoder->tim = tim;
    encoder->last_count = 0;
    encoder->total_count = 0;
    encoder->speed_rpm = 0.0f;
    encoder->counts_per_rev = counts_per_rev;
    encoder->direction = direction < 0 ? -1 : 1;

    HAL_StatusTypeDef result = HAL_TIM_Encoder_Start(tim, TIM_CHANNEL_ALL);
    if (result == HAL_OK) {
        encoder->last_count = (uint16_t)__HAL_TIM_GET_COUNTER(tim);
    }
    return result;
}

void Encoder_Update(Encoder_t *encoder, float dt_s)
{
    if (encoder == NULL || encoder->tim == NULL || dt_s <= 0.0f) {
        return;
    }

    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(encoder->tim);
    int32_t delta = (int32_t)now - (int32_t)encoder->last_count;
    encoder->last_count = now;

    /* TIM2 的计数周期已在 CubeMX 设为 65535。 */
    if (delta > 32767) {
        delta -= 65536;
    } else if (delta < -32768) {
        delta += 65536;
    }

    delta *= encoder->direction;
    encoder->total_count += delta;
    encoder->speed_rpm =
        (float)delta * 60.0f / (encoder->counts_per_rev * dt_s);
}