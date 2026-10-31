#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H
#include "main.h"
typedef struct {
    uint32_t ready,start_status,update_count,raw_count;
    int32_t encoder_count;
    uint32_t invalid_edges,zero_valid,stop_reason,pwm_compare;
    float angle_deg,rpm,target_rpm,applied_duty;
    float position_target_deg,position_error_deg;
    uint32_t position_settle_ms;
    uint8_t at_target;
    /* Automatic first accepted SW4 calibration, once per A-board boot. */
    uint32_t auto_zero_count,auto_zero_epoch;
} ArmDebug_t;
enum { ARM_STOP_NONE=0,ARM_STOP_DISABLED=1,ARM_STOP_CONFIG=2,
       ARM_STOP_NO_ZERO=3,ARM_STOP_LIMIT=4 };
extern volatile uint32_t g_arm_enable,g_arm_zero_request;
extern volatile int32_t g_arm_encoder_direction,g_arm_motor_direction;
extern volatile float g_arm_counts_per_joint_rev,g_arm_target_deg;
extern volatile float g_arm_min_deg,g_arm_max_deg,g_arm_max_rpm,g_arm_max_duty;
extern volatile float g_arm_pos_kp,g_arm_speed_kp,g_arm_speed_ki;
extern volatile ArmDebug_t g_arm_debug;
void Arm_EncoderIRQ(void);
void ArmControl_Task(void);
#endif