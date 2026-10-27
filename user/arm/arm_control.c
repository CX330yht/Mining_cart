#include "arm_control.h"
#include "tim.h"
#include "cmsis_os.h"
#include "pid.h"
#include <math.h>

#define ARM_PERIOD_MS 10U
#define ARM_WAKE_MS 2U
#define ARM_REVERSE_MS 500U
#define ARM_BENCH_MS 2000U

volatile uint32_t g_arm_mode = ARM_MODE_POSITION;
volatile float g_arm_open_duty = 0.0f, g_arm_speed_target_rpm = 0.0f;
volatile uint32_t g_arm_enable = 0U;
volatile uint32_t g_arm_bench_mode = 0U;
volatile uint32_t g_arm_calibration_ok = 0U;
volatile uint32_t g_arm_limits_verified = 0U;
volatile uint32_t g_arm_zero_request = 0U;
volatile uint32_t g_arm_fault_clear = 0U;
volatile int32_t g_arm_encoder_direction = 1;
volatile int32_t g_arm_motor_direction = 1;
/* Nominal gearbox-output estimate only; set calibration_ok after measuring. */
volatile float g_arm_counts_per_joint_rev = 937.2f;
volatile float g_arm_target_deg = 0.0f;
volatile float g_arm_min_deg = -10.0f, g_arm_max_deg = 10.0f;
volatile float g_arm_max_rpm = 5.0f, g_arm_max_duty = 0.15f;
/* Unloaded commissioning seeds, not tuned coefficients for the actual arm. */
volatile float g_arm_pos_kp = 0.10f;
volatile float g_arm_speed_kp = 0.02f, g_arm_speed_ki = 0.0f;
volatile ArmDebug_t g_arm_debug;

static volatile uint32_t s_raw_count, s_invalid_edges;
static uint8_t s_previous_ab;
static uint32_t s_zero_raw;
static uint8_t s_active, s_output_running;
static uint32_t s_active_mode;
static int8_t s_last_direction;
static uint32_t s_run_start, s_wake_start, s_stop_tick;

static float Clamp(float x, float limit)
{
    return x > limit ? limit : (x < -limit ? -limit : x);
}

static uint8_t ReadAB(void)
{
    /* Both signals are on GPIOF; one IDR read gives a coherent snapshot. */
    uint32_t levels = GPIOF->IDR;
    return (uint8_t)(((levels & ARM_ENC_A_Pin) ? 2U : 0U) |
                     ((levels & ARM_ENC_B_Pin) ? 1U : 0U));
}

void Arm_EncoderIRQ(void)
{
    static const int8_t step[16] = {
        0, 1, -1, 0, -1, 0, 0, 1,
        1, 0, 0, -1, 0, -1, 1, 0
    };
    uint8_t next = ReadAB();
    uint8_t old = s_previous_ab;
    if ((old ^ next) == 3U) s_invalid_edges++;
    s_raw_count += (uint32_t)(int32_t)step[(old << 2U) | next];
    s_previous_ab = next;
}

static void Coast(uint32_t now)
{
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0U);
    HAL_GPIO_WritePin(ARM_DIR_GPIO_Port, ARM_DIR_Pin, GPIO_PIN_RESET);
    if (s_output_running) {
        s_stop_tick = now;
        s_output_running = 0U;
    }
    g_arm_debug.applied_duty = 0.0f;
    g_arm_debug.pwm_compare = 0U;
}

static void SleepDrive(uint32_t now)
{
    HAL_GPIO_WritePin(ARM_NSLEEP_GPIO_Port, ARM_NSLEEP_Pin, GPIO_PIN_RESET);
    Coast(now);
    s_last_direction = 0;
}

static void Abort(uint32_t reason, uint32_t now, PID_t *speed)
{
    SleepDrive(now);
    PID_Reset(speed);
    g_arm_enable = 0U;
    g_arm_open_duty = 0.0f;
    g_arm_speed_target_rpm = 0.0f;
    g_arm_target_deg = 0.0f;
    g_arm_debug.target_rpm = 0.0f;
    g_arm_debug.stop_reason = reason;
    s_active = 0U;
}

static float Apply(float duty, uint32_t now)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim13) + 1U;
    uint32_t magnitude = (uint32_t)(fabsf(duty) * (float)period + 0.5f);
    if (magnitude == 0U) {
        Coast(now);
        return 0.0f;
    }
    if (magnitude > period) magnitude = period;
    int8_t logical = duty >= 0.0f ? 1 : -1;
    int8_t physical = (int8_t)(logical * g_arm_motor_direction);
    if (s_last_direction && physical != s_last_direction) {
        Coast(now);
        if ((uint32_t)(now - s_stop_tick) < ARM_REVERSE_MS) return 0.0f;
    }
    HAL_GPIO_WritePin(ARM_DIR_GPIO_Port, ARM_DIR_Pin,
                      physical > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    uint32_t compare = physical > 0 ? magnitude : period - magnitude;
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, compare);
    s_output_running = 1U;
    s_last_direction = physical;
    g_arm_debug.pwm_compare = compare;
    g_arm_debug.applied_duty =
        (float)logical * (float)magnitude / (float)period;
    return g_arm_debug.applied_duty;
}

static uint8_t ConfigValid(void)
{
    return g_arm_mode <= ARM_MODE_POSITION &&
           isfinite(g_arm_open_duty) && fabsf(g_arm_open_duty) <= 0.15f &&
           isfinite(g_arm_speed_target_rpm) &&
           isfinite(g_arm_counts_per_joint_rev) &&
           g_arm_counts_per_joint_rev > 0.0f &&
           (g_arm_encoder_direction == 1 || g_arm_encoder_direction == -1) &&
           (g_arm_motor_direction == 1 || g_arm_motor_direction == -1) &&
           isfinite(g_arm_min_deg) && isfinite(g_arm_max_deg) &&
           g_arm_min_deg < g_arm_max_deg &&
           isfinite(g_arm_target_deg) &&
           (g_arm_mode != ARM_MODE_POSITION ||
            (g_arm_target_deg >= g_arm_min_deg &&
             g_arm_target_deg <= g_arm_max_deg)) &&
           isfinite(g_arm_max_rpm) && g_arm_max_rpm > 0.0f &&
           g_arm_max_rpm <= 20.0f &&
           isfinite(g_arm_max_duty) && g_arm_max_duty > 0.0f &&
           g_arm_max_duty <= 0.30f &&
           isfinite(g_arm_pos_kp) && g_arm_pos_kp > 0.0f &&
           isfinite(g_arm_speed_kp) && g_arm_speed_kp > 0.0f &&
           isfinite(g_arm_speed_ki) && g_arm_speed_ki >= 0.0f;
}

void ArmControl_Task(void)
{
    PID_t speed;
    PID_Init(&speed, 0.0f, 0.0f, 0.0f, 0.15f);
    g_arm_debug = (ArmDebug_t){0};
    s_previous_ab = ReadAB();
    uint32_t now = osKernelSysTick();
    SleepDrive(now);
    __HAL_TIM_DISABLE_OCxPRELOAD(&htim13, TIM_CHANNEL_1);
    HAL_StatusTypeDef status =
        HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    g_arm_debug.start_status = (uint32_t)status;
    g_arm_debug.ready = status == HAL_OK;
    uint32_t last_wake = now, last_sample = now;
    uint32_t previous_raw = s_raw_count, last_movement = now;
    float filtered_rpm = 0.0f;

    for (;;) {
        osDelayUntil(&last_wake, ARM_PERIOD_MS);
        now = osKernelSysTick();
        uint32_t elapsed = now - last_sample;
        last_sample = now;
        uint32_t raw = s_raw_count;
        int32_t delta = (int32_t)(raw - previous_raw);
        previous_raw = raw;
        g_arm_debug.raw_count = raw;
        g_arm_debug.encoder_count = (int32_t)raw;
        g_arm_debug.invalid_edges = s_invalid_edges;
        g_arm_debug.update_count++;

        if (g_arm_zero_request && !g_arm_enable) {
            g_arm_zero_request = 0U;
            s_zero_raw = raw;
            g_arm_debug.zero_valid = 1U;
            g_arm_target_deg = 0.0f;
            filtered_rpm = 0.0f;
            PID_Reset(&speed);
        }
        if (g_arm_fault_clear && !g_arm_enable) {
            g_arm_fault_clear = 0U;
            if (HAL_GPIO_ReadPin(ARM_NFAULT_GPIO_Port,
                                ARM_NFAULT_Pin) == GPIO_PIN_SET) {
                g_arm_debug.fault_latched = 0U;
            }
        }
        if (!ConfigValid() || !elapsed || elapsed > 100U) {
            Abort(ARM_STOP_CONFIG, now, &speed);
            continue;
        }

        float direction = (float)g_arm_encoder_direction;
        float counts = g_arm_counts_per_joint_rev;
        float dt = (float)elapsed * 0.001f;
        float rpm = (float)delta * direction * 60.0f / (counts * dt);
        filtered_rpm += dt / (0.10f + dt) * (rpm - filtered_rpm);
        float angle = (float)(int32_t)(raw - s_zero_raw) *
                      direction * 360.0f / counts;
        g_arm_debug.angle_deg = angle;
        g_arm_debug.rpm = filtered_rpm;

        if (!g_arm_enable) {
            if (s_active) Abort(ARM_STOP_DISABLED, now, &speed);
            SleepDrive(now);
            PID_Reset(&speed);
            s_active = 0U;
            g_arm_debug.target_rpm = 0.0f;
            continue;
        }
        if (!g_arm_debug.ready ||
            (g_arm_mode != ARM_MODE_OPEN && !g_arm_calibration_ok) ||
            (g_arm_mode == ARM_MODE_OPEN && !g_arm_bench_mode) ||
            (!g_arm_bench_mode && !g_arm_limits_verified) ||
            (s_active && g_arm_mode != s_active_mode)) {
            Abort(ARM_STOP_CONFIG, now, &speed);
            continue;
        }
        if (g_arm_mode == ARM_MODE_POSITION && !g_arm_debug.zero_valid) {
            Abort(ARM_STOP_NO_ZERO, now, &speed);
            continue;
        }
        if (g_arm_debug.fault_latched) {
            Abort(ARM_STOP_FAULT, now, &speed);
            continue;
        }
        if (HAL_GPIO_ReadPin(ARM_LIMIT_MIN_GPIO_Port,
                            ARM_LIMIT_MIN_Pin) == GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(ARM_LIMIT_MAX_GPIO_Port,
                            ARM_LIMIT_MAX_Pin) == GPIO_PIN_RESET ||
            (g_arm_mode == ARM_MODE_POSITION &&
             (angle < g_arm_min_deg || angle > g_arm_max_deg))) {
            Abort(ARM_STOP_LIMIT, now, &speed);
            continue;
        }
        if (!s_active) {
            s_active = 1U;
            s_active_mode = g_arm_mode;
            s_run_start = s_wake_start = last_movement = now;
            g_arm_debug.stop_reason = ARM_STOP_NONE;
            HAL_GPIO_WritePin(ARM_NSLEEP_GPIO_Port, ARM_NSLEEP_Pin,
                              GPIO_PIN_SET);
        }
        if (g_arm_bench_mode &&
            (uint32_t)(now - s_run_start) >= ARM_BENCH_MS) {
            Abort(ARM_STOP_TIMEOUT, now, &speed);
            continue;
        }
        if ((uint32_t)(now - s_wake_start) < ARM_WAKE_MS) continue;
        if (HAL_GPIO_ReadPin(ARM_NFAULT_GPIO_Port,
                            ARM_NFAULT_Pin) == GPIO_PIN_RESET) {
            g_arm_debug.fault_latched = 1U;
            Abort(ARM_STOP_FAULT, now, &speed);
            continue;
        }

        float target_rpm = g_arm_mode == ARM_MODE_POSITION ?
            g_arm_pos_kp * (g_arm_target_deg - angle) :
            g_arm_speed_target_rpm;
        target_rpm = Clamp(target_rpm, g_arm_max_rpm);
        g_arm_debug.target_rpm = target_rpm;
        speed.kp = g_arm_speed_kp;
        speed.ki = g_arm_speed_ki;
        speed.kd = 0.0f;
        speed.output_limit = g_arm_bench_mode ?
            fminf(g_arm_max_duty, 0.15f) : g_arm_max_duty;
        float duty;
        if (g_arm_mode == ARM_MODE_OPEN) {
            PID_Reset(&speed);
            duty = Clamp(g_arm_open_duty, speed.output_limit);
        } else {
            duty = PID_Calculate(&speed, target_rpm, filtered_rpm, dt);
        }
        if (!isfinite(duty)) {
            Abort(ARM_STOP_CONFIG, now, &speed);
            continue;
        }
        float actual = Apply(duty, now);
        if (actual == 0.0f && duty != 0.0f) PID_Reset(&speed);
        if (delta || fabsf(actual) < 0.10f) last_movement = now;
        if ((uint32_t)(now - last_movement) >= 500U) {
            Abort(ARM_STOP_STALL, now, &speed);
        }
    }
}