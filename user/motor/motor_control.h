#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdint.h>
#include "tb6612.h"
enum { MOTOR_MODE_CHASSIS=2 };
enum { MOTOR_STOP_NONE=0, MOTOR_STOP_DISABLED=1, MOTOR_STOP_INVALID=3 };
typedef struct {
    uint32_t ready,start_status,encoder_start_status[4],update_count;
    uint32_t pwm_compare[4];
    float applied_duty[4];
    uint32_t active_mode,stop_reason,speed_mask;
} MotorDebug_t;
extern volatile uint32_t g_motor_enable,g_motor_mode;
extern volatile MotorDebug_t g_motor_debug;
extern volatile int32_t g_wheel_count[4];
extern volatile float g_wheel_rpm[4],g_wheel_rpm_filtered[4];
extern volatile float g_target_rpm[4],g_ramped_target_rpm[4],g_pid_command[4];
extern volatile float g_speed_kp[4],g_speed_ki[4],g_speed_kd[4];
extern volatile float g_chassis_vx_mps,g_chassis_vy_mps,g_chassis_wz_radps,g_chassis_scale;
void MotorControl_Task(void);
#endif