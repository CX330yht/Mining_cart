#ifndef TB6612_H
#define TB6612_H

#include "main.h"

#define TB6612_MOTOR_COUNT 4U
#define TB6612_MAX_DUTY 0.30f

HAL_StatusTypeDef TB6612_Init(void);
void TB6612_StopAll(void);
float TB6612_SetDuty(uint32_t motor_index, float duty);

#endif