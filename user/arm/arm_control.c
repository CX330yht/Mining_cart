#include "arm_control.h"
#include "tim.h"
#include "cmsis_os.h"
#include "pid.h"
#include <math.h>

#define ARM_PERIOD_MS 10U
#define ARM_WAKE_MS 2U
#define ARM_REVERSE_MS 500U
#define ARM_BENCH_MS 200U
#define ARM_SPEED_BENCH_MS 1000U
#define ARM_POSITION_BENCH_MS 8000U
#define ARM_POSITION_MAX_RPM 2.0f
#define ARM_POSITION_PATH_MARGIN_DEG 5.0f
#define ARM_POSITION_TOL_DEG 1.0f
#define ARM_POSITION_TOL_RPM 0.20f
#define ARM_POSITION_SETTLE_MS 100U
#define ARM_BENCH_TRAVEL_DEG 10.0f
#define ARM_BENCH_MAX_DUTY 0.50f

volatile uint32_t g_arm_mode = ARM_MODE_POSITION;
volatile float g_arm_open_duty = 0.0f, g_arm_speed_target_rpm = 0.0f;
volatile uint32_t g_arm_enable = 0U;
volatile uint32_t g_arm_bench_mode = 0U;
volatile uint32_t g_arm_calibration_ok = 0U;
volatile uint32_t g_arm_limits_verified = 0U;
volatile uint32_t g_arm_zero_request = 0U;
volatile int32_t g_arm_encoder_direction = 1;
volatile int32_t g_arm_motor_direction = 1;
/* Joint commissioning scale: user checked horizontal/45/90/return manually.
   This does not authorize powered position control or verify limit switches. */
volatile float g_arm_counts_per_joint_rev = 12936.0f;
volatile float g_arm_target_deg = 0.0f;
/* Temporary observed manual range; powered travel limits are not verified. */
volatile float g_arm_min_deg = 0.0f, g_arm_max_deg = 90.0f;
volatile float g_arm_max_rpm = 5.0f, g_arm_max_duty = 0.05f;
/* Unloaded commissioning seeds, not tuned coefficients for the actual arm. */
volatile float g_arm_pos_kp = 0.40f;
volatile float g_arm_speed_kp = 0.50f, g_arm_speed_ki = 1.50f;
volatile ArmDebug_t g_arm_debug;

static volatile uint32_t s_raw_count, s_invalid_edges;
static uint8_t s_previous_ab;
static uint32_t s_zero_raw;
static uint8_t s_active, s_output_running;
static uint32_t s_active_mode;
static int8_t s_last_direction;
static uint32_t s_run_start, s_wake_start, s_stop_tick;
static float s_position_target;
static int8_t s_position_approach;
static uint8_t s_position_in_band;
static uint32_t s_position_settle_start;

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

static void BrakeChannel(uint32_t now)
{
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0U);
    /* PWM=0 and IN1=IN2=1 explicitly short-brakes while STBY is high. */
    HAL_GPIO_WritePin(ARM_IN1_GPIO_Port, ARM_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ARM_IN2_GPIO_Port, ARM_IN2_Pin, GPIO_PIN_SET);
    if (s_output_running) {
        s_stop_tick = now;
        s_output_running = 0U;
    }
    g_arm_debug.applied_duty = 0.0f;
    g_arm_debug.pwm_compare = 0U;
}

static void StandbyDrive(uint32_t now)
{
    HAL_GPIO_WritePin(ARM_STBY_GPIO_Port, ARM_STBY_Pin, GPIO_PIN_RESET);
    BrakeChannel(now);
    s_last_direction = 0;
}

static void Abort(uint32_t reason, uint32_t now, PID_t *speed)
{
    StandbyDrive(now);
    PID_Reset(speed);
    g_arm_enable = 0U;
    g_arm_open_duty = 0.0f;
    g_arm_speed_target_rpm = 0.0f;
    g_arm_target_deg = 0.0f;
    g_arm_debug.target_rpm = 0.0f;
    g_arm_debug.stop_reason = reason;
    s_active = 0U;
    s_position_in_band = 0U;
}

static float Apply(float duty, uint32_t now)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim9) + 1U;
    uint32_t magnitude = (uint32_t)(fabsf(duty) * (float)period + 0.5f);
    if (magnitude == 0U) {
        BrakeChannel(now);
        return 0.0f;
    }
    if (magnitude > period) magnitude = period;
    int8_t logical = duty >= 0.0f ? 1 : -1;
    int8_t physical = (int8_t)(logical * g_arm_motor_direction);
    if (s_last_direction && physical != s_last_direction) {
        BrakeChannel(now);
        if ((uint32_t)(now - s_stop_tick) < ARM_REVERSE_MS) return 0.0f;
    }
    HAL_GPIO_WritePin(ARM_IN1_GPIO_Port, ARM_IN1_Pin,
                      physical > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ARM_IN2_GPIO_Port, ARM_IN2_Pin,
                      physical > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
    uint32_t compare = magnitude;
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, compare);
    s_output_running = 1U;
    s_last_direction = physical;
    g_arm_debug.pwm_compare = compare;
    g_arm_debug.applied_duty =
        (float)logical * (float)magnitude / (float)period;
    return g_arm_debug.applied_duty;
}

static uint8_t ConfigValid(void)
{
    return g_arm_mode <= ARM_MODE_POSITION && g_arm_bench_mode <= 1U &&
           isfinite(g_arm_open_duty) &&
           fabsf(g_arm_open_duty) <= ARM_BENCH_MAX_DUTY &&
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
           (!(g_arm_mode == ARM_MODE_POSITION && g_arm_bench_mode == 1U) ||
            g_arm_max_rpm <= ARM_POSITION_MAX_RPM) &&
           isfinite(g_arm_max_duty) && g_arm_max_duty > 0.0f &&
           g_arm_max_duty <=
               ((g_arm_bench_mode == 1U) ?
                ARM_BENCH_MAX_DUTY : 0.30f) &&
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
    StandbyDrive(now);
    __HAL_DBGMCU_FREEZE_TIM9();
    __HAL_TIM_DISABLE_OCxPRELOAD(&htim9, TIM_CHANNEL_2);
    HAL_StatusTypeDef status =
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
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
        g_arm_debug.position_error_deg =
            g_arm_debug.position_target_deg - angle;

        if (!g_arm_enable) {
            if (s_active) Abort(ARM_STOP_DISABLED, now, &speed);
            StandbyDrive(now);
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
        if (!g_arm_debug.zero_valid) {
            Abort(ARM_STOP_NO_ZERO, now, &speed);
            continue;
        }
        uint32_t limit_mask = 0U;
        if (HAL_GPIO_ReadPin(ARM_LIMIT_MIN_GPIO_Port,
                            ARM_LIMIT_MIN_Pin) == GPIO_PIN_RESET)
            limit_mask |= ARM_LIMIT_DETAIL_MIN_INPUT;
        if (HAL_GPIO_ReadPin(ARM_LIMIT_MAX_GPIO_Port,
                            ARM_LIMIT_MAX_Pin) == GPIO_PIN_RESET)
            limit_mask |= ARM_LIMIT_DETAIL_MAX_INPUT;
        if ((g_arm_mode == ARM_MODE_POSITION || g_arm_bench_mode) &&
            (angle < g_arm_min_deg || angle > g_arm_max_deg))
            limit_mask |= ARM_LIMIT_DETAIL_ANGLE_WINDOW;
        if (limit_mask) {
            g_arm_debug.limit_reason_mask = limit_mask;
            Abort(ARM_STOP_LIMIT, now, &speed);
            continue;
        }
        if (s_active && g_arm_bench_mode &&
            g_arm_mode == ARM_MODE_POSITION &&
            fabsf(g_arm_target_deg - s_position_target) > 0.01f) {
            Abort(ARM_STOP_CONFIG, now, &speed);
            continue;
        }
        if (!s_active) {
            if (g_arm_bench_mode && g_arm_mode == ARM_MODE_POSITION &&
                (g_arm_target_deg < g_arm_min_deg + ARM_BENCH_TRAVEL_DEG ||
                 g_arm_target_deg > g_arm_max_deg - ARM_BENCH_TRAVEL_DEG)) {
                g_arm_debug.limit_reason_mask = ARM_LIMIT_DETAIL_TARGET_MARGIN;
                Abort(ARM_STOP_LIMIT, now, &speed);
                continue;
            }
            /* Leave room on both sides: motor direction is not known yet. */
            if (g_arm_bench_mode &&
                (angle < g_arm_min_deg + ARM_BENCH_TRAVEL_DEG ||
                 angle > g_arm_max_deg - ARM_BENCH_TRAVEL_DEG)) {
                g_arm_debug.limit_reason_mask = ARM_LIMIT_DETAIL_START_MARGIN;
                Abort(ARM_STOP_LIMIT, now, &speed);
                continue;
            }
            g_arm_debug.bench_elapsed_ms = 0U;
            g_arm_debug.bench_start_angle_deg = angle;
            g_arm_debug.bench_travel_deg = 0.0f;
            g_arm_debug.bench_peak_duty = 0.0f;
            g_arm_debug.bench_peak_pwm_compare = 0U;
            s_position_target = g_arm_target_deg;
            s_position_approach = s_position_target >= angle ? 1 : -1;
            s_position_in_band = 0U;
            g_arm_debug.position_target_deg = s_position_target;
            g_arm_debug.position_error_deg = s_position_target - angle;
            g_arm_debug.position_settle_ms = 0U;
            g_arm_debug.limit_reason_mask = 0U;
            g_arm_debug.bench_path_min_deg = fmaxf(g_arm_min_deg,
                fminf(angle, s_position_target) - ARM_POSITION_PATH_MARGIN_DEG);
            g_arm_debug.bench_path_max_deg = fminf(g_arm_max_deg,
                fmaxf(angle, s_position_target) + ARM_POSITION_PATH_MARGIN_DEG);
            s_active = 1U;
            s_active_mode = g_arm_mode;
            s_run_start = s_wake_start = last_movement = now;
            g_arm_debug.stop_reason = ARM_STOP_NONE;
            HAL_GPIO_WritePin(ARM_STBY_GPIO_Port, ARM_STBY_Pin,
                              GPIO_PIN_SET);
        }
        g_arm_debug.bench_elapsed_ms = (uint32_t)(now - s_run_start);
        g_arm_debug.bench_travel_deg =
            angle - g_arm_debug.bench_start_angle_deg;
        uint8_t travel_exceeded = g_arm_mode == ARM_MODE_POSITION ?
            (angle < g_arm_debug.bench_path_min_deg ||
             angle > g_arm_debug.bench_path_max_deg) :
            (fabsf(g_arm_debug.bench_travel_deg) >= ARM_BENCH_TRAVEL_DEG);
        if (g_arm_bench_mode && travel_exceeded) {
            Abort(ARM_STOP_BENCH_TRAVEL, now, &speed);
            continue;
        }
        uint32_t bench_ms = g_arm_mode == ARM_MODE_POSITION ?
                            ARM_POSITION_BENCH_MS :
                            (g_arm_mode == ARM_MODE_SPEED ?
                             ARM_SPEED_BENCH_MS : ARM_BENCH_MS);
        if (g_arm_bench_mode &&
            (uint32_t)(now - s_run_start) >= bench_ms) {
            Abort(ARM_STOP_TIMEOUT, now, &speed);
            continue;
        }
        if (g_arm_bench_mode && g_arm_mode == ARM_MODE_POSITION) {
            if (fabsf(g_arm_debug.position_error_deg) <= ARM_POSITION_TOL_DEG &&
                fabsf(filtered_rpm) <= ARM_POSITION_TOL_RPM) {
                if (!s_position_in_band) {
                    s_position_in_band = 1U;
                    s_position_settle_start = now;
                }
                g_arm_debug.position_settle_ms = now - s_position_settle_start;
                if (g_arm_debug.position_settle_ms >= ARM_POSITION_SETTLE_MS) {
                    Abort(ARM_STOP_POSITION_REACHED, now, &speed);
                    continue;
                }
            } else {
                s_position_in_band = 0U;
                g_arm_debug.position_settle_ms = 0U;
            }
        }
        if ((uint32_t)(now - s_wake_start) < ARM_WAKE_MS) continue;
        float target_rpm = g_arm_mode == ARM_MODE_POSITION ?
            g_arm_pos_kp * (g_arm_target_deg - angle) :
            g_arm_speed_target_rpm;
        if (g_arm_bench_mode && g_arm_mode == ARM_MODE_POSITION) {
            float error = s_position_target - angle;
            target_rpm = g_arm_pos_kp * error;
            /* First position bench stops on crossing; no reverse hunting. */
            if (fabsf(error) <= ARM_POSITION_TOL_DEG ||
                error * (float)s_position_approach <= 0.0f) {
                target_rpm = 0.0f;
                PID_Reset(&speed);
            }
        }
        target_rpm = Clamp(target_rpm, g_arm_max_rpm);
        g_arm_debug.target_rpm = target_rpm;
        speed.kp = g_arm_speed_kp;
        speed.ki = g_arm_speed_ki;
        speed.kd = 0.0f;
        speed.output_limit = g_arm_bench_mode ?
            fminf(g_arm_max_duty, ARM_BENCH_MAX_DUTY) : g_arm_max_duty;
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
        /* Speed bench drives only in the target direction; overspeed brakes. */
        if (g_arm_bench_mode && g_arm_mode != ARM_MODE_OPEN &&
            (target_rpm == 0.0f || duty * target_rpm < 0.0f)) {
            duty = 0.0f;
            PID_Reset(&speed);
        }
        float actual = Apply(duty, now);
        if (fabsf(actual) > fabsf(g_arm_debug.bench_peak_duty))
            g_arm_debug.bench_peak_duty = actual;
        if (g_arm_debug.pwm_compare > g_arm_debug.bench_peak_pwm_compare)
            g_arm_debug.bench_peak_pwm_compare = g_arm_debug.pwm_compare;
        if (actual == 0.0f && duty != 0.0f) PID_Reset(&speed);
        if (delta || fabsf(actual) < 0.10f) last_movement = now;
        if ((uint32_t)(now - last_movement) >= 500U) {
            Abort(ARM_STOP_STALL, now, &speed);
        }
    }
}