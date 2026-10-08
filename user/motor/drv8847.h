#ifndef DRV8847_H
#define DRV8847_H

#include "main.h"

#define DRV8847_MOTOR_COUNT 4U
#define DRV8847_BOARD_COUNT 2U
#define DRV8847_MAX_DUTY 1.00f

typedef struct {
    uint32_t ready;
    uint32_t awake_mask;      /* bit0=D1, bit1=D2 */
    uint32_t active_mask;     /* bit0..3=M1..M4, logical output state */
    uint32_t raw_fault_mask;  /* raw low level; not qualified during sleep/wake */
    uint32_t fault_mask;      /* latched fault: bit0=D1, bit1=D2 */
} Drv8847Debug_t;

extern volatile Drv8847Debug_t g_drv8847_debug;

HAL_StatusTypeDef DRV8847_Init(void);
void DRV8847_StopAll(void);
void DRV8847_PollFaults(void);
void DRV8847_ClearFaults(void);
HAL_StatusTypeDef DRV8847_Apply(
    const float duty[DRV8847_MOTOR_COUNT],
    float applied[DRV8847_MOTOR_COUNT]);

#endif
