#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "tim.h"

typedef struct {
    TIM_HandleTypeDef *tim;
    uint16_t last_count;
    int32_t total_count;
    float speed_rpm;
    float counts_per_rev;
    int8_t direction;
} Encoder_t;

HAL_StatusTypeDef Encoder_Start(Encoder_t *encoder, TIM_HandleTypeDef *tim,
                                float counts_per_rev, int8_t direction);
void Encoder_Update(Encoder_t *encoder, float dt_s);

#endif