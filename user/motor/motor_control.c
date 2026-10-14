#include "motor_control.h"
#include "encoder.h"
#include "pid.h"
#include "cmsis_os.h"
#include "tim.h"
#include <math.h>

#define CONTROL_PERIOD_MS 10U
#define ENCODER_COUNTS_PER_REV 1560.0f
#define OPEN_LOOP_RUN_MS 2000U
#define SPEED_RUN_MS 5000U
#define SPEED_LIMIT_RPM 60.0f
#define SPEED_RAMP_RPM_PER_S 100.0f
#define SPEED_FILTER_TAU_S 0.05f

volatile int32_t g_wheel_count = 0;
volatile float g_wheel_rpm = 0.0f;
volatile float g_wheel_rpm_filtered = 0.0f;
volatile float g_target_rpm = 0.0f;
volatile float g_ramped_target_rpm = 0.0f;
volatile float g_pid_command = 0.0f;

/* Initial trial gains; tune on the actual motor after the first run. */
volatile float g_speed_kp = 0.003f;
volatile float g_speed_ki = 0.010f;
volatile float g_speed_kd = 0.0f;

volatile uint32_t g_motor_enable = 0U;
volatile uint32_t g_motor_mode = MOTOR_MODE_OPEN_LOOP;
volatile float g_motor_duty[TB6612_MOTOR_COUNT] = {0};
volatile MotorDebug_t g_motor_debug = {0};

static const uint32_t s_channels[TB6612_MOTOR_COUNT] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

static float RampTo(float current, float target, float max_step)
{
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}

static uint8_t ValidTarget(float target)
{
    return isfinite(target) && fabsf(target) <= SPEED_LIMIT_RPM;
}

static void StopRun(PID_t *pid, MotorStopReason_t reason)
{
    TB6612_StopAll();
    PID_Reset(pid);
    g_motor_enable = 0U;
    g_target_rpm = 0.0f;
    g_ramped_target_rpm = 0.0f;
    g_pid_command = 0.0f;
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        g_motor_duty[i] = 0.0f;
    }
    g_motor_debug.stop_reason = (uint32_t)reason;
}

void MotorControl_Task(void)
{
    Encoder_t encoder;
    PID_t speed_pid;
    HAL_StatusTypeDef status = TB6612_Init();
    g_motor_debug.start_status = (uint32_t)status;
    if (status != HAL_OK) Error_Handler();
    if (Encoder_Start(&encoder, &htim2, ENCODER_COUNTS_PER_REV, 1) != HAL_OK) {
        TB6612_StopAll();
        Error_Handler();
    }
    PID_Init(&speed_pid, 0.003f, 0.010f, 0.0f, TB6612_MAX_DUTY);
    g_motor_debug.ready = 1U;

    uint32_t last_wake = osKernelSysTick();
    uint32_t last_sample = last_wake;
    uint32_t run_start = last_wake;
    uint32_t active_mode = MOTOR_MODE_OPEN_LOOP;
    uint8_t run_active = 0U;
    int8_t run_direction = 0;

    for (;;) {
        osDelayUntil(&last_wake, CONTROL_PERIOD_MS);
        uint32_t now = osKernelSysTick();
        uint32_t elapsed_ms = now - last_sample;
        last_sample = now;
        float dt_s = (float)elapsed_ms * 0.001f;
        if (elapsed_ms != 0U) {
            Encoder_Update(&encoder, dt_s);
            g_wheel_count = encoder.total_count;
            g_wheel_rpm = encoder.speed_rpm;
            float alpha = dt_s / (SPEED_FILTER_TAU_S + dt_s);
            g_wheel_rpm_filtered += alpha * (g_wheel_rpm - g_wheel_rpm_filtered);
        }

        float applied_duty[TB6612_MOTOR_COUNT] = {0};
        if (g_motor_enable == 1U) {
            if (run_active == 0U) {
                active_mode = g_motor_mode;
                float target = g_target_rpm;
                float kp = g_speed_kp;
                float ki = g_speed_ki;
                float kd = g_speed_kd;
                uint8_t valid = active_mode == MOTOR_MODE_OPEN_LOOP;
                if (active_mode == MOTOR_MODE_SPEED) {
                    valid = ValidTarget(target) &&
                            isfinite(kp) && isfinite(ki) && isfinite(kd) &&
                            kp >= 0.0f && ki >= 0.0f && kd >= 0.0f;
                }
                if (valid == 0U) {
                    StopRun(&speed_pid, MOTOR_STOP_INVALID);
                } else {
                    if (active_mode == MOTOR_MODE_SPEED) {
                        PID_Init(&speed_pid, kp, ki, kd, TB6612_MAX_DUTY);
                    } else {
                        PID_Reset(&speed_pid);
                    }
                    run_start = now;
                    run_active = 1U;
                    run_direction = (target >= 0.0f) ? 1 : -1;
                    g_ramped_target_rpm = 0.0f;
                    g_pid_command = 0.0f;
                    g_motor_debug.active_mode = active_mode;
                    g_motor_debug.stop_reason = MOTOR_STOP_NONE;
                }
            }

            if (run_active != 0U) {
                uint32_t limit_ms = (active_mode == MOTOR_MODE_SPEED) ?
                                    SPEED_RUN_MS : OPEN_LOOP_RUN_MS;
                if ((uint32_t)(now - run_start) >= limit_ms) {
                    StopRun(&speed_pid, MOTOR_STOP_TIMEOUT);
                    run_active = 0U;
                } else if (active_mode == MOTOR_MODE_OPEN_LOOP) {
                    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                        applied_duty[i] = TB6612_SetDuty(i, g_motor_duty[i]);
                    }
                } else {
                    float target = g_target_rpm;
                    if (!ValidTarget(target)) {
                        StopRun(&speed_pid, MOTOR_STOP_INVALID);
                        run_active = 0U;
                    } else if (fabsf(target) < 0.001f ||
                               target * (float)run_direction < 0.0f) {
                        /* Reverse targets require a new enable after stopping. */
                        StopRun(&speed_pid, MOTOR_STOP_DISABLED);
                        run_active = 0U;
                    } else if (dt_s > 0.0f) {
                        g_ramped_target_rpm = RampTo(g_ramped_target_rpm, target,
                                                   SPEED_RAMP_RPM_PER_S * dt_s);
                        float command = PID_Calculate(&speed_pid,
                            g_ramped_target_rpm, g_wheel_rpm_filtered, dt_s);
                        if (!isfinite(command)) {
                            StopRun(&speed_pid, MOTOR_STOP_INVALID);
                            run_active = 0U;
                        } else {
                            /* First bench loop does not actively drive backwards to brake. */
                            if (command * (float)run_direction < 0.0f) {
                                command = 0.0f;
                                PID_Reset(&speed_pid);
                            }
                            g_pid_command = command;
                            applied_duty[0] = TB6612_SetDuty(0U, command);
                            for (uint32_t i = 1U; i < TB6612_MOTOR_COUNT; ++i) {
                                applied_duty[i] = TB6612_SetDuty(i, 0.0f);
                            }
                            /* Avoid integral growth during driver reversal waiting. */
                            if (fabsf(command) > 0.001f &&
                                fabsf(applied_duty[0]) < 0.00001f) {
                                PID_Reset(&speed_pid);
                            }
                        }
                    } else {
                        TB6612_StopAll();
                        PID_Reset(&speed_pid);
                        g_pid_command = 0.0f;
                    }
                }
            }
        } else {
            if (run_active != 0U) {
                StopRun(&speed_pid, MOTOR_STOP_DISABLED);
                run_active = 0U;
            } else {
                TB6612_StopAll();
                PID_Reset(&speed_pid);
                g_ramped_target_rpm = 0.0f;
                g_pid_command = 0.0f;
            }
        }

        for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
            g_motor_debug.applied_duty[i] = applied_duty[i];
            g_motor_debug.pwm_compare[i] =
                __HAL_TIM_GET_COMPARE(&htim1, s_channels[i]);
        }
        g_motor_debug.update_count++;
    }
}