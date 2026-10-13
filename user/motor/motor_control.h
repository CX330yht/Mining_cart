#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

typedef struct {
    uint32_t ready;
    uint32_t start_status;
    uint32_t update_count;
    uint32_t pwm_compare;
    float applied_duty;
} MotorDebug_t;

extern volatile uint32_t g_motor_enable;
extern volatile float g_motor_duty;
extern volatile MotorDebug_t g_motor_debug;

void MotorControl_Task(void);

#endif