#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "main.h"

typedef struct {
    uint32_t ready;
    uint32_t start_status;
    uint32_t update_count;
    uint32_t raw_count;
    int32_t encoder_count;
    uint32_t invalid_edges;
    uint32_t zero_valid;
    uint32_t fault_latched;
    uint32_t stop_reason;
    uint32_t pwm_compare;
    float angle_deg;
    float rpm;
    float target_rpm;
    float applied_duty;
} ArmDebug_t;

enum {
    ARM_STOP_NONE = 0, ARM_STOP_DISABLED, ARM_STOP_CONFIG,
    ARM_STOP_NO_ZERO, ARM_STOP_LIMIT, ARM_STOP_FAULT,
    ARM_STOP_TIMEOUT, ARM_STOP_STALL
};

enum { ARM_MODE_OPEN = 0, ARM_MODE_SPEED = 1, ARM_MODE_POSITION = 2 };

extern volatile uint32_t g_arm_mode;
extern volatile float g_arm_open_duty, g_arm_speed_target_rpm;
extern volatile uint32_t g_arm_enable;
extern volatile uint32_t g_arm_bench_mode;
extern volatile uint32_t g_arm_calibration_ok;
extern volatile uint32_t g_arm_limits_verified;
extern volatile uint32_t g_arm_zero_request;
extern volatile uint32_t g_arm_fault_clear;
extern volatile int32_t g_arm_encoder_direction;
extern volatile int32_t g_arm_motor_direction;
extern volatile float g_arm_counts_per_joint_rev;
extern volatile float g_arm_target_deg;
extern volatile float g_arm_min_deg, g_arm_max_deg;
extern volatile float g_arm_max_rpm, g_arm_max_duty;
extern volatile float g_arm_pos_kp, g_arm_speed_kp, g_arm_speed_ki;
extern volatile ArmDebug_t g_arm_debug;

void Arm_EncoderIRQ(void);
void ArmControl_Task(void);

#endif