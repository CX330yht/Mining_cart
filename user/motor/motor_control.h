#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "tb6612.h"

typedef struct {
    uint32_t ready;
    uint32_t start_status;
    uint32_t update_count;
    uint32_t pwm_compare[TB6612_MOTOR_COUNT];
    float applied_duty[TB6612_MOTOR_COUNT];
} MotorDebug_t;

extern volatile uint32_t g_motor_enable;
extern volatile float g_motor_duty[TB6612_MOTOR_COUNT];
extern volatile MotorDebug_t g_motor_debug;

void MotorControl_Task(void);

#endif