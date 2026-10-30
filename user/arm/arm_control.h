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
    uint32_t stop_reason;
    uint32_t pwm_compare;
    float angle_deg;
    float rpm;
    float target_rpm;
    float applied_duty;
    /* Latched until next enable: readable after the short bench pulse ends. */
    uint32_t bench_elapsed_ms;
    float bench_start_angle_deg;
    float bench_travel_deg;
    float bench_peak_duty;
    uint32_t bench_peak_pwm_compare;
    float position_target_deg;
    float position_error_deg;
    uint32_t position_settle_ms;
    float bench_path_min_deg;
    float bench_path_max_deg;
    uint32_t limit_reason_mask;
} ArmDebug_t;

enum {
    ARM_STOP_NONE = 0, ARM_STOP_DISABLED = 1, ARM_STOP_CONFIG = 2,
    ARM_STOP_NO_ZERO = 3, ARM_STOP_LIMIT = 4,
    /* Value 5 is unused: TB6612 has no external nFAULT output. */
    ARM_STOP_TIMEOUT = 6, ARM_STOP_STALL = 7, ARM_STOP_BENCH_TRAVEL = 8,
    ARM_STOP_POSITION_REACHED = 9
};

enum { ARM_MODE_OPEN = 0, ARM_MODE_SPEED = 1, ARM_MODE_POSITION = 2 };

enum {
    ARM_LIMIT_DETAIL_MIN_INPUT = 1U, ARM_LIMIT_DETAIL_MAX_INPUT = 2U,
    ARM_LIMIT_DETAIL_ANGLE_WINDOW = 4U, ARM_LIMIT_DETAIL_START_MARGIN = 8U,
    ARM_LIMIT_DETAIL_TARGET_MARGIN = 16U
};

extern volatile uint32_t g_arm_mode;
extern volatile float g_arm_open_duty, g_arm_speed_target_rpm;
extern volatile uint32_t g_arm_enable;
extern volatile uint32_t g_arm_bench_mode;
extern volatile uint32_t g_arm_calibration_ok;
extern volatile uint32_t g_arm_limits_verified;
extern volatile uint32_t g_arm_zero_request;
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