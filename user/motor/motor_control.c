#include "motor_control.h"
#include "encoder.h"
#include "pid.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"

#define CONTROL_PERIOD_MS 10U
#define ENCODER_COUNTS_PER_REV 1560.0f

/* 这些变量供 Ozone 观察；1560 计数/圈还需实物验证。 */
volatile int32_t g_wheel_count = 0;
volatile float g_wheel_rpm = 0.0f;
volatile float g_target_rpm = 0.0f;
volatile float g_pid_command = 0.0f;

void MotorControl_Task(void)
{
    Encoder_t encoder;
    PID_t speed_pid;

    if (Encoder_Start(&encoder, &htim2, ENCODER_COUNTS_PER_REV, 1) != HAL_OK) {
        Error_Handler();
    }

    /* 暂用 P 项观察计算结果；Ki/Kd 和实际输出要在开环验证后调整。 */
    PID_Init(&speed_pid, 0.001f, 0.0f, 0.0f, 0.2f);

    uint32_t last_wake = osKernelSysTick();
    uint32_t last_sample = last_wake;

    for (;;) {
        osDelayUntil(&last_wake, CONTROL_PERIOD_MS);

        uint32_t now = osKernelSysTick();
        uint32_t elapsed_ms = now - last_sample;
        last_sample = now;
        if (elapsed_ms == 0U) continue;

        float dt_s = (float)elapsed_ms * 0.001f;
        Encoder_Update(&encoder, dt_s);

        g_wheel_count = encoder.total_count;
        g_wheel_rpm = encoder.speed_rpm;
        g_pid_command =
            PID_Calculate(&speed_pid, g_target_rpm, g_wheel_rpm, dt_s);

        /* 当前只计算，不启动 PWM，也不把命令送往 D 板。 */
    }
}