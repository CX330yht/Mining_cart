#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "drv8847.h"

typedef enum {
    MOTOR_MODE_OPEN_LOOP = 0,
    MOTOR_MODE_SPEED = 1,
    MOTOR_MODE_CHASSIS = 2
} MotorMode_t;

typedef enum {
    MOTOR_STOP_NONE = 0,
    MOTOR_STOP_DISABLED = 1,
    MOTOR_STOP_TIMEOUT = 2,
    MOTOR_STOP_INVALID = 3,
    MOTOR_STOP_DRIVER_FAULT = 4
} MotorStopReason_t;

typedef struct {
    uint32_t ready;
    uint32_t start_status;
    uint32_t encoder_start_status[DRV8847_MOTOR_COUNT];
    uint32_t update_count;
    uint32_t pwm_compare[DRV8847_MOTOR_COUNT];
    float applied_duty[DRV8847_MOTOR_COUNT];
    uint32_t active_mode;
    uint32_t stop_reason;
    uint32_t speed_mask;
} MotorDebug_t;

extern volatile uint32_t g_motor_enable;
extern volatile uint32_t g_motor_fault_clear;
extern volatile uint32_t g_motor_mode;
extern volatile float g_motor_duty[DRV8847_MOTOR_COUNT];
extern volatile MotorDebug_t g_motor_debug;

extern volatile int32_t g_wheel_count[DRV8847_MOTOR_COUNT];
extern volatile float g_wheel_rpm[DRV8847_MOTOR_COUNT];
extern volatile float g_wheel_rpm_filtered[DRV8847_MOTOR_COUNT];
extern volatile float g_target_rpm[DRV8847_MOTOR_COUNT];
extern volatile float g_ramped_target_rpm[DRV8847_MOTOR_COUNT];
extern volatile float g_pid_command[DRV8847_MOTOR_COUNT];
extern volatile float g_speed_kp[DRV8847_MOTOR_COUNT];
extern volatile float g_speed_ki[DRV8847_MOTOR_COUNT];
extern volatile float g_speed_kd[DRV8847_MOTOR_COUNT];

/* Short bench command: sampled once when mode=2 is enabled. */
extern volatile float g_chassis_vx_mps;
extern volatile float g_chassis_vy_mps;
extern volatile float g_chassis_wz_radps;
extern volatile float g_chassis_scale;

typedef enum {
    CHASSIS_TEST_NONE = 0,
    CHASSIS_TEST_FORWARD = 1,
    CHASSIS_TEST_BACKWARD = 2,
    CHASSIS_TEST_LEFT = 3,
    CHASSIS_TEST_RIGHT = 4,
    CHASSIS_TEST_TURN_LEFT = 5,
    CHASSIS_TEST_TURN_RIGHT = 6,
    CHASSIS_TEST_STOP = 9
} ChassisTestCommand_t;

typedef enum {
    CHASSIS_TEST_IDLE = 0,
    CHASSIS_TEST_ACCEPTED = 1,
    CHASSIS_TEST_BUSY = 2,
    CHASSIS_TEST_WAIT_STOP = 3,
    CHASSIS_TEST_INVALID = 4,
    CHASSIS_TEST_STOPPED = 5,
    CHASSIS_TEST_DRIVER_FAULT = 6
} ChassisTestResult_t;

/* One-shot mailbox: write 1..6 to start a 5-second action, 9 to stop. */
/* A request is consumed immediately. Busy/not-stopped requests are not queued. */
extern volatile uint32_t g_chassis_test_cmd;
extern volatile uint32_t g_chassis_speed_percent;
extern volatile uint32_t g_chassis_test_result;

typedef struct {
    uint32_t request_count;
    uint32_t accepted_count;
    uint32_t last_command;
    uint32_t enable_before;
    uint32_t active_before;
    uint32_t active_mask_before;
    uint32_t elapsed_since_stop_ms;
} ChassisTestDebug_t;

extern volatile ChassisTestDebug_t g_chassis_test_debug;

void MotorControl_Task(void);

#endif