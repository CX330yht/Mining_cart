#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "tb6612.h"

typedef enum {
    MOTOR_MODE_OPEN_LOOP = 0,
    MOTOR_MODE_SPEED = 1
} MotorMode_t;

typedef enum {
    MOTOR_STOP_NONE = 0,
    MOTOR_STOP_DISABLED = 1,
    MOTOR_STOP_TIMEOUT = 2,
    MOTOR_STOP_INVALID = 3
} MotorStopReason_t;

typedef struct {
    uint32_t ready;
    uint32_t start_status;
    uint32_t encoder_start_status[TB6612_MOTOR_COUNT];
    uint32_t update_count;
    uint32_t pwm_compare[TB6612_MOTOR_COUNT];
    float applied_duty[TB6612_MOTOR_COUNT];
    uint32_t active_mode;
    uint32_t stop_reason;
    uint32_t speed_mask;
} MotorDebug_t;

extern volatile uint32_t g_motor_enable;
extern volatile uint32_t g_motor_mode;
extern volatile float g_motor_duty[TB6612_MOTOR_COUNT];
extern volatile MotorDebug_t g_motor_debug;

extern volatile int32_t g_wheel_count[TB6612_MOTOR_COUNT];
extern volatile float g_wheel_rpm[TB6612_MOTOR_COUNT];
extern volatile float g_wheel_rpm_filtered[TB6612_MOTOR_COUNT];
extern volatile float g_target_rpm[TB6612_MOTOR_COUNT];
extern volatile float g_ramped_target_rpm[TB6612_MOTOR_COUNT];
extern volatile float g_pid_command[TB6612_MOTOR_COUNT];
extern volatile float g_speed_kp[TB6612_MOTOR_COUNT];
extern volatile float g_speed_ki[TB6612_MOTOR_COUNT];
extern volatile float g_speed_kd[TB6612_MOTOR_COUNT];

void MotorControl_Task(void);

#endif