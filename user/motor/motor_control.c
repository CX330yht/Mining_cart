#include "motor_control.h"
#include "encoder.h"
#include "pid.h"
#include "tb6612.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"

#define CONTROL_PERIOD_MS 10U
#define ENCODER_COUNTS_PER_REV 1560.0f
#define BENCH_RUN_TIME_MS 2000U

/* The original single-encoder/PID framework remains disconnected from output. */
volatile int32_t g_wheel_count = 0;
volatile float g_wheel_rpm = 0.0f;
volatile float g_target_rpm = 0.0f;
volatile float g_pid_command = 0.0f;

/* Index 0/1/2/3 = M1/M2/M3/M4. Set all duties before enabling. */
volatile uint32_t g_motor_enable = 0U;
volatile float g_motor_duty[TB6612_MOTOR_COUNT] = {0};
volatile MotorDebug_t g_motor_debug = {0};

static const uint32_t s_channels[TB6612_MOTOR_COUNT] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

void MotorControl_Task(void)
{
    Encoder_t encoder;
    PID_t speed_pid;

    HAL_StatusTypeDef status = TB6612_Init();
    g_motor_debug.start_status = (uint32_t)status;
    if (status != HAL_OK) {
        Error_Handler();
    }
    if (Encoder_Start(&encoder, &htim2, ENCODER_COUNTS_PER_REV, 1) != HAL_OK) {
        TB6612_StopAll();
        Error_Handler();
    }

    PID_Init(&speed_pid, 0.001f, 0.0f, 0.0f, 0.2f);
    g_motor_debug.ready = 1U;
    uint32_t last_wake = osKernelSysTick();
    uint32_t last_sample = last_wake;
    uint32_t run_start = last_wake;
    uint8_t run_active = 0U;

    for (;;) {
        osDelayUntil(&last_wake, CONTROL_PERIOD_MS);
        uint32_t now = osKernelSysTick();
        uint32_t elapsed_ms = now - last_sample;
        last_sample = now;
        if (elapsed_ms != 0U) {
            float dt_s = (float)elapsed_ms * 0.001f;
            Encoder_Update(&encoder, dt_s);
            g_wheel_count = encoder.total_count;
            g_wheel_rpm = encoder.speed_rpm;
            g_pid_command =
                PID_Calculate(&speed_pid, g_target_rpm, g_wheel_rpm, dt_s);
        }

        float applied_duty[TB6612_MOTOR_COUNT] = {0};
        if (g_motor_enable == 1U) {
            if (run_active == 0U) {
                run_start = now;
                run_active = 1U;
            }
            if ((uint32_t)(now - run_start) >= BENCH_RUN_TIME_MS) {
                g_motor_enable = 0U;
                run_active = 0U;
                for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                    g_motor_duty[i] = 0.0f;
                }
                TB6612_StopAll();
            } else {
                for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
                    applied_duty[i] = TB6612_SetDuty(i, g_motor_duty[i]);
                }
            }
        } else {
            run_active = 0U;
            TB6612_StopAll();
        }

        for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
            g_motor_debug.applied_duty[i] = applied_duty[i];
            g_motor_debug.pwm_compare[i] =
                __HAL_TIM_GET_COMPARE(&htim1, s_channels[i]);
        }
        g_motor_debug.update_count++;
    }
}