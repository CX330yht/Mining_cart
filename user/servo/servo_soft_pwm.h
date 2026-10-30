#ifndef SERVO_SOFT_PWM_H
#define SERVO_SOFT_PWM_H

#include "main.h"

typedef struct {
    uint32_t ready;
    uint32_t running;
    uint32_t irq_count;
    uint32_t frame_count;
    uint32_t pulse_us[2];
    uint32_t active_pulse_us[2];
    uint32_t start_status;
} ServoSoftPwmDebug_t;

extern volatile ServoSoftPwmDebug_t g_servo_soft_pwm_debug;

HAL_StatusTypeDef ServoSoftPwm_Init(void);
HAL_StatusTypeDef ServoSoftPwm_Start(void);
void ServoSoftPwm_Stop(void);
void ServoSoftPwm_SetPulseUs(uint32_t index, uint16_t pulse_us);
void ServoSoftPwm_OnTimerElapsed(void);

#endif