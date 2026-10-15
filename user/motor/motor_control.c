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
#define TARGET_ZERO_RPM 0.001f

volatile int32_t g_wheel_count[TB6612_MOTOR_COUNT] = {0};
volatile float g_wheel_rpm[TB6612_MOTOR_COUNT] = {0};
volatile float g_wheel_rpm_filtered[TB6612_MOTOR_COUNT] = {0};
volatile float g_target_rpm[TB6612_MOTOR_COUNT] = {0};
volatile float g_ramped_target_rpm[TB6612_MOTOR_COUNT] = {0};
volatile float g_pid_command[TB6612_MOTOR_COUNT] = {0};

/* M1 bench baseline; starting values only for the other three motors. */
volatile float g_speed_kp[TB6612_MOTOR_COUNT] = {0.030f, 0.030f, 0.030f, 0.030f};
volatile float g_speed_ki[TB6612_MOTOR_COUNT] = {0.010f, 0.010f, 0.010f, 0.010f};
volatile float g_speed_kd[TB6612_MOTOR_COUNT] = {0};

volatile uint32_t g_motor_enable = 0U;
volatile uint32_t g_motor_mode = MOTOR_MODE_OPEN_LOOP;
volatile float g_motor_duty[TB6612_MOTOR_COUNT] = {0};
volatile MotorDebug_t g_motor_debug = {0};

static const uint32_t s_channels[TB6612_MOTOR_COUNT] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

/* Index 0/1/2/3 = M1/M2/M3/M4. */
static TIM_HandleTypeDef *const s_encoder_timers[TB6612_MOTOR_COUNT] = {
    &htim2, &htim4, &htim5, &htim8
};
static const int8_t s_encoder_direction[TB6612_MOTOR_COUNT] = {1, -1, -1, -1};

static float RampTo(float current, float target, float max_step)
{
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}

static uint8_t ReadSpeedTargets(float targets[], uint32_t *mask)
{
    uint32_t bits = 0U;
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        targets[i] = g_target_rpm[i];
        if (!isfinite(targets[i]) || fabsf(targets[i]) > SPEED_LIMIT_RPM) {
            return 0U;
        }
        if (fabsf(targets[i]) >= TARGET_ZERO_RPM) bits |= 1U << i;
    }
    *mask = bits;
    return 1U;
}

static void ClearControllers(PID_t pid[])
{
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        PID_Reset(&pid[i]);
        g_ramped_target_rpm[i] = 0.0f;
        g_pid_command[i] = 0.0f;
    }
}

static void StopRun(PID_t pid[], MotorStopReason_t reason)
{
    TB6612_StopAll();
    ClearControllers(pid);
    g_motor_enable = 0U;
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        g_target_rpm[i] = 0.0f;
        g_motor_duty[i] = 0.0f;
    }
    g_motor_debug.speed_mask = 0U;
    g_motor_debug.stop_reason = (uint32_t)reason;
}

void MotorControl_Task(void)
{
    Encoder_t encoder[TB6612_MOTOR_COUNT];
    PID_t speed_pid[TB6612_MOTOR_COUNT];
    HAL_StatusTypeDef status = TB6612_Init();
    g_motor_debug.start_status = (uint32_t)status;
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        g_motor_debug.encoder_start_status[i] = UINT32_MAX;
    }
    if (status != HAL_OK) Error_Handler();
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        status = Encoder_Start(&encoder[i], s_encoder_timers[i],
                               ENCODER_COUNTS_PER_REV, s_encoder_direction[i]);
        g_motor_debug.encoder_start_status[i] = (uint32_t)status;
        if (status != HAL_OK) {
            TB6612_StopAll();
            Error_Handler();
        }
        PID_Init(&speed_pid[i], 0.030f, 0.010f, 0.0f, TB6612_MAX_DUTY);
    }
    g_motor_debug.ready = 1U;

    uint32_t last_wake = osKernelSysTick();
    uint32_t last_sample = last_wake;
    uint32_t run_start = last_wake;
    uint32_t active_mode = MOTOR_MODE_OPEN_LOOP;
    uint32_t speed_mask = 0U;
    uint8_t run_active = 0U;
    int8_t run_direction[TB6612_MOTOR_COUNT] = {0};

    for (;;) {
        osDelayUntil(&last_wake, CONTROL_PERIOD_MS);
        uint32_t now = osKernelSysTick();
        uint32_t elapsed_ms = now - last_sample;
        last_sample = now;
        float dt_s = (float)elapsed_ms * 0.001f;
        if (elapsed_ms != 0U) {
            float alpha = dt_s / (SPEED_FILTER_TAU_S + dt_s);
            for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                Encoder_Update(&encoder[i], dt_s);
                g_wheel_count[i] = encoder[i].total_count;
                g_wheel_rpm[i] = encoder[i].speed_rpm;
                g_wheel_rpm_filtered[i] +=
                    alpha * (g_wheel_rpm[i] - g_wheel_rpm_filtered[i]);
            }
        }

        float applied_duty[TB6612_MOTOR_COUNT] = {0};
        if (g_motor_enable == 1U) {
            if (run_active == 0U) {
                active_mode = g_motor_mode;
                float targets[TB6612_MOTOR_COUNT] = {0};
                float kp[TB6612_MOTOR_COUNT];
                float ki[TB6612_MOTOR_COUNT];
                float kd[TB6612_MOTOR_COUNT];
                uint32_t requested_mask = 0U;
                uint8_t valid = active_mode == MOTOR_MODE_OPEN_LOOP;
                if (active_mode == MOTOR_MODE_SPEED) {
                    valid = ReadSpeedTargets(targets, &requested_mask);
                    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                        kp[i] = g_speed_kp[i];
                        ki[i] = g_speed_ki[i];
                        kd[i] = g_speed_kd[i];
                        if (!isfinite(kp[i]) || !isfinite(ki[i]) || !isfinite(kd[i]) ||
                            kp[i] < 0.0f || ki[i] < 0.0f || kd[i] < 0.0f) valid = 0U;
                    }
                }
                if (valid == 0U) {
                    StopRun(speed_pid, MOTOR_STOP_INVALID);
                } else if (active_mode == MOTOR_MODE_SPEED && requested_mask == 0U) {
                    StopRun(speed_pid, MOTOR_STOP_DISABLED);
                } else {
                    ClearControllers(speed_pid);
                    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                        if (active_mode == MOTOR_MODE_SPEED) {
                            PID_Init(&speed_pid[i], kp[i], ki[i], kd[i], TB6612_MAX_DUTY);
                        }
                        run_direction[i] = targets[i] >= 0.0f ? 1 : -1;
                    }
                    speed_mask = requested_mask;
                    run_start = now;
                    run_active = 1U;
                    g_motor_debug.active_mode = active_mode;
                    g_motor_debug.speed_mask = speed_mask;
                    g_motor_debug.stop_reason = MOTOR_STOP_NONE;
                }
            }

            if (run_active != 0U) {
                uint32_t limit_ms = active_mode == MOTOR_MODE_SPEED ?
                                    SPEED_RUN_MS : OPEN_LOOP_RUN_MS;
                if ((uint32_t)(now - run_start) >= limit_ms) {
                    StopRun(speed_pid, MOTOR_STOP_TIMEOUT);
                    run_active = 0U;
                } else if (active_mode == MOTOR_MODE_OPEN_LOOP) {
                    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                        applied_duty[i] = TB6612_SetDuty(i, g_motor_duty[i]);
                    }
                } else {
                    float targets[TB6612_MOTOR_COUNT];
                    uint32_t requested_mask = 0U;
                    MotorStopReason_t reason = MOTOR_STOP_NONE;
                    if (!ReadSpeedTargets(targets, &requested_mask)) {
                        reason = MOTOR_STOP_INVALID;
                    } else if (requested_mask != speed_mask) {
                        /* Changing the selected motors requires a new enable. */
                        reason = MOTOR_STOP_DISABLED;
                    } else {
                        for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                            if ((speed_mask & (1U << i)) != 0U &&
                                targets[i] * (float)run_direction[i] < 0.0f) {
                                reason = MOTOR_STOP_DISABLED;
                            }
                        }
                    }

                    if (reason != MOTOR_STOP_NONE) {
                        StopRun(speed_pid, reason);
                        run_active = 0U;
                    } else if (dt_s > 0.0f) {
                        float commands[TB6612_MOTOR_COUNT] = {0};
                        for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                            if ((speed_mask & (1U << i)) == 0U) continue;
                            g_ramped_target_rpm[i] = RampTo(g_ramped_target_rpm[i],
                                targets[i], SPEED_RAMP_RPM_PER_S * dt_s);
                            commands[i] = PID_Calculate(&speed_pid[i],
                                g_ramped_target_rpm[i], g_wheel_rpm_filtered[i], dt_s);
                            if (!isfinite(commands[i])) {
                                StopRun(speed_pid, MOTOR_STOP_INVALID);
                                run_active = 0U;
                                break;
                            }
                            /* No active reverse braking in this bench firmware. */
                            if (commands[i] * (float)run_direction[i] < 0.0f) {
                                commands[i] = 0.0f;
                                PID_Reset(&speed_pid[i]);
                            }
                        }
                        /* Apply only after all four commands have been checked. */
                        if (run_active != 0U) {
                            for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                                g_pid_command[i] = commands[i];
                                applied_duty[i] = TB6612_SetDuty(i, commands[i]);
                                if (fabsf(commands[i]) > 0.001f &&
                                    fabsf(applied_duty[i]) < 0.00001f) {
                                    PID_Reset(&speed_pid[i]);
                                }
                            }
                        }
                    } else {
                        TB6612_StopAll();
                        ClearControllers(speed_pid);
                    }
                }
            }
        } else {
            if (run_active != 0U) {
                StopRun(speed_pid, MOTOR_STOP_DISABLED);
                run_active = 0U;
            } else {
                TB6612_StopAll();
                ClearControllers(speed_pid);
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