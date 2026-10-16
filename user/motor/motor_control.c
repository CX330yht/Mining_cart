#include "motor_control.h"
#include "encoder.h"
#include "pid.h"
#include "mecanum.h"
#include <stddef.h>
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
#define CHASSIS_TEST_RUN_MS 2000U
#define CHASSIS_TEST_PAUSE_MS 500U
#define CHASSIS_TEST_STOP_RPM 5.0f
#define CHASSIS_TEST_LINEAR_MPS 0.060f
#define CHASSIS_TEST_TURN_RADPS 0.180f

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

volatile float g_chassis_vx_mps = 0.0f;
volatile float g_chassis_vy_mps = 0.0f;
volatile float g_chassis_wz_radps = 0.0f;
volatile float g_chassis_scale = 1.0f;
volatile uint32_t g_chassis_test_cmd = CHASSIS_TEST_NONE;
volatile uint32_t g_chassis_test_result = CHASSIS_TEST_IDLE;
volatile ChassisTestDebug_t g_chassis_test_debug = {0};

static const uint32_t s_channels[TB6612_MOTOR_COUNT] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

/* Index 0/1/2/3 = M1/M2/M3/M4. */
static TIM_HandleTypeDef *const s_encoder_timers[TB6612_MOTOR_COUNT] = {
    &htim2, &htim4, &htim5, &htim8
};
static const int8_t s_encoder_direction[TB6612_MOTOR_COUNT] = {1, 1, -1, 1};

static float RampTo(float current, float target, float max_step)
{
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}

static uint8_t WheelsStopped(void)
{
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        float rpm = g_wheel_rpm_filtered[i];
        if (!isfinite(rpm) || fabsf(rpm) > CHASSIS_TEST_STOP_RPM) return 0U;
    }
    return 1U;
}

static uint32_t ActivePwmMask(void)
{
    uint32_t mask = 0U;
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        if (__HAL_TIM_GET_COMPARE(&htim1, s_channels[i]) != 0U) mask |= 1U << i;
    }
    return mask;
}

static void SetChassisTestVelocity(uint32_t command)
{
    g_chassis_vx_mps = 0.0f;
    g_chassis_vy_mps = 0.0f;
    g_chassis_wz_radps = 0.0f;
    switch (command) {
    case CHASSIS_TEST_FORWARD: g_chassis_vx_mps = CHASSIS_TEST_LINEAR_MPS; break;
    case CHASSIS_TEST_BACKWARD: g_chassis_vx_mps = -CHASSIS_TEST_LINEAR_MPS; break;
    case CHASSIS_TEST_LEFT: g_chassis_vy_mps = CHASSIS_TEST_LINEAR_MPS; break;
    case CHASSIS_TEST_RIGHT: g_chassis_vy_mps = -CHASSIS_TEST_LINEAR_MPS; break;
    case CHASSIS_TEST_TURN_LEFT: g_chassis_wz_radps = CHASSIS_TEST_TURN_RADPS; break;
    case CHASSIS_TEST_TURN_RIGHT: g_chassis_wz_radps = -CHASSIS_TEST_TURN_RADPS; break;
    default: break;
    }
}

static uint8_t ReadSpeedTargets(float targets[], uint32_t *mask,
                                const float fixed_targets[])
{
    uint32_t bits = 0U;
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        targets[i] = fixed_targets != NULL ? fixed_targets[i] : g_target_rpm[i];
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
    g_chassis_test_cmd = CHASSIS_TEST_NONE;
    g_chassis_vx_mps = 0.0f;
    g_chassis_vy_mps = 0.0f;
    g_chassis_wz_radps = 0.0f;
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
    float chassis_targets[TB6612_MOTOR_COUNT] = {0};
    uint8_t test_run = 0U;
    uint32_t last_stop = last_wake - CHASSIS_TEST_PAUSE_MS;

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

        uint8_t was_active = run_active != 0U || g_motor_enable == 1U;
        uint32_t test_command = g_chassis_test_cmd;
        if (test_command != CHASSIS_TEST_NONE) {
            g_chassis_test_cmd = CHASSIS_TEST_NONE;
            uint32_t pwm_mask = ActivePwmMask();
            g_chassis_test_debug.request_count++;
            g_chassis_test_debug.last_command = test_command;
            g_chassis_test_debug.enable_before = g_motor_enable;
            g_chassis_test_debug.active_before = run_active;
            g_chassis_test_debug.pwm_mask_before = pwm_mask;
            g_chassis_test_debug.elapsed_since_stop_ms = now - last_stop;
            if (test_command == CHASSIS_TEST_STOP) {
                StopRun(speed_pid, MOTOR_STOP_DISABLED);
                run_active = 0U;
                test_run = 0U;
                last_stop = now;
                g_chassis_test_result = CHASSIS_TEST_STOPPED;
            } else if (test_command < CHASSIS_TEST_FORWARD ||
                       test_command > CHASSIS_TEST_TURN_RIGHT) {
                StopRun(speed_pid, MOTOR_STOP_INVALID);
                run_active = 0U;
                test_run = 0U;
                last_stop = now;
                g_chassis_test_result = CHASSIS_TEST_INVALID;
                       } else if (run_active != 0U ||
                          g_motor_enable != 0U ||
                          pwm_mask != 0U) {
                g_chassis_test_result = CHASSIS_TEST_BUSY;
            } else if ((uint32_t)(now - last_stop) < CHASSIS_TEST_PAUSE_MS ||
                       WheelsStopped() == 0U) {
                g_chassis_test_result = CHASSIS_TEST_WAIT_STOP;
                       } else {
                           SetChassisTestVelocity(test_command);
                           g_motor_mode = MOTOR_MODE_CHASSIS;
                           g_motor_enable = 1U;
                           test_run = 1U;
                           g_chassis_test_result = CHASSIS_TEST_ACCEPTED;
                           g_chassis_test_debug.accepted_count++;
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
                if (active_mode == MOTOR_MODE_SPEED ||
                    active_mode == MOTOR_MODE_CHASSIS) {
                    valid = 1U;
                    g_chassis_scale = 1.0f;
                    if (active_mode == MOTOR_MODE_CHASSIS) {
                        float scale = 1.0f;
                        valid = Mecanum_Calculate(g_chassis_vx_mps,
                            g_chassis_vy_mps, g_chassis_wz_radps,
                            SPEED_LIMIT_RPM, chassis_targets, &scale);
                        if (valid != 0U) g_chassis_scale = scale;
                    }
                    if (valid != 0U) {
                        valid = ReadSpeedTargets(targets, &requested_mask,
                            active_mode == MOTOR_MODE_CHASSIS ? chassis_targets : NULL);
                        if (active_mode == MOTOR_MODE_CHASSIS) {
                            for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                                g_target_rpm[i] = targets[i];
                            }
                        }
                    }
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
                } else if (active_mode != MOTOR_MODE_OPEN_LOOP && requested_mask == 0U) {
                    StopRun(speed_pid, MOTOR_STOP_DISABLED);
                } else {
                    ClearControllers(speed_pid);
                    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                        if (active_mode != MOTOR_MODE_OPEN_LOOP) {
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
                uint32_t limit_ms = active_mode != MOTOR_MODE_OPEN_LOOP ?
                                    SPEED_RUN_MS : OPEN_LOOP_RUN_MS;
                if (test_run != 0U) limit_ms = CHASSIS_TEST_RUN_MS;
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
                    if (!ReadSpeedTargets(targets, &requested_mask,
                        active_mode == MOTOR_MODE_CHASSIS ? chassis_targets : NULL)) {
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
                        if (active_mode == MOTOR_MODE_CHASSIS) {
                            /* g_target_rpm is an observation in chassis mode. */
                            for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                                g_target_rpm[i] = targets[i];
                            }
                        }
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

        if (was_active != 0U && run_active == 0U && g_motor_enable == 0U) {
            last_stop = now;
        }
        if (test_run != 0U && run_active == 0U && g_motor_enable == 0U) {
            last_stop = now;
            if (g_motor_debug.stop_reason == MOTOR_STOP_INVALID) {
                g_chassis_test_result = CHASSIS_TEST_INVALID;
            } else if (g_motor_debug.stop_reason == MOTOR_STOP_DISABLED) {
                g_chassis_test_result = CHASSIS_TEST_STOPPED;
            }
            test_run = 0U;
        }

        for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
            g_motor_debug.applied_duty[i] = applied_duty[i];
            g_motor_debug.pwm_compare[i] =
                __HAL_TIM_GET_COMPARE(&htim1, s_channels[i]);
        }
        g_motor_debug.update_count++;
    }
}
