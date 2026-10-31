#ifndef TB6612_H
#define TB6612_H

#include "main.h"

/* Four chassis wheels only. The independent arm uses the third module. */
#define TB6612_MOTOR_COUNT 4U
#define TB6612_BOARD_COUNT 2U
#define TB6612_MAX_DUTY 1.00f

typedef struct {
    uint32_t ready;
    uint32_t enabled_board_mask; /* bit0=left module, bit1=right module */
    uint32_t active_mask;        /* bit0..3=M1..M4; software state only */
} Tb6612Debug_t;

extern volatile Tb6612Debug_t g_tb6612_debug;

HAL_StatusTypeDef TB6612_Init(void);
void TB6612_StopAll(void);
HAL_StatusTypeDef TB6612_Apply(
    const float duty[TB6612_MOTOR_COUNT],
    float applied[TB6612_MOTOR_COUNT]);

#endif