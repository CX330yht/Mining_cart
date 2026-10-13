#include "servo.h"
#include "tim.h"
#include "cmsis_os.h"
#include <math.h>

#define SERVO_PERIOD_MS 20U
#define SERVO_SPEED_DPS 60.0f

typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    uint16_t min_us;
    uint16_t max_us;
    uint8_t reversed;
} ServoChannel_t;

/* Calibrate endpoints and direction independently for each physical servo. */
static const ServoChannel_t s_channels[SERVO_COUNT] = {
    {&htim9, TIM_CHANNEL_1, 500U, 2500U, 0U},
    {&htim9, TIM_CHANNEL_2, 500U, 2500U, 0U},
    {&htim3, TIM_CHANNEL_3, 500U, 2500U, 0U},
    {&htim3, TIM_CHANNEL_4, 500U, 2500U, 0U}
};

static const float s_initial_deg[SERVO_COUNT] = {90.0f, 0.0f, 0.0f, 0.0f};

volatile float g_servo_target_deg[SERVO_COUNT] = {90.0f, 0.0f, 0.0f, 0.0f};
volatile uint8_t g_servo_hold = 0U;
volatile ServoDebug_t g_servo_debug;

static float Servo_LimitAngle(float angle)
{
    if (angle < 0.0f) return 0.0f;
    if (angle > 180.0f) return 180.0f;
    return angle;
}

static void Servo_Write(uint32_t index, float angle)
{
    const ServoChannel_t *config = &s_channels[index];
    float physical = config->reversed ? (180.0f - angle) : angle;
    float pulse = (float)config->min_us + physical *
                  (float)(config->max_us - config->min_us) / 180.0f;
    uint16_t pulse_us = (uint16_t)(pulse + 0.5f);

    __HAL_TIM_SET_COMPARE(config->timer, config->channel, pulse_us);
    g_servo_debug.angle_deg[index] = angle;
    g_servo_debug.pulse_us[index] = pulse_us;
}

HAL_StatusTypeDef Servo_Init(void)
{
    g_servo_debug = (ServoDebug_t){0};
    g_servo_hold = 0U;

    /* All initial compares are written before enabling any PWM output. */
    for (uint32_t i = 0U; i < SERVO_COUNT; ++i) {
        g_servo_debug.start_status[i] = HAL_ERROR;
        g_servo_target_deg[i] = s_initial_deg[i];
        Servo_Write(i, s_initial_deg[i]);
    }
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim9, 0U);
    (void)HAL_TIM_GenerateEvent(&htim3, TIM_EVENTSOURCE_UPDATE);
    (void)HAL_TIM_GenerateEvent(&htim9, TIM_EVENTSOURCE_UPDATE);

    for (uint32_t i = 0U; i < SERVO_COUNT; ++i) {
        HAL_StatusTypeDef status = HAL_TIM_PWM_Start(
            s_channels[i].timer, s_channels[i].channel);
        g_servo_debug.start_status[i] = status;
        if (status != HAL_OK) {
            for (uint32_t j = 0U; j < SERVO_COUNT; ++j) {
                (void)HAL_TIM_PWM_Stop(s_channels[j].timer,
                                       s_channels[j].channel);
            }
            return status;
        }
    }
    g_servo_debug.ready = 1U;
    return HAL_OK;
}

void Servo_Update(float dt_s)
{
    if (!g_servo_debug.ready || !isfinite(dt_s) || dt_s <= 0.0f) return;
    /* Avoid a large jump after a debug halt or a missed scheduling interval. */
    if (dt_s > 0.1f) dt_s = 0.1f;

    for (uint32_t i = 0U; i < SERVO_COUNT; ++i) {
        float current = g_servo_debug.angle_deg[i];
        float target = g_servo_target_deg[i];

        if (!isfinite(target)) {
            target = current;
            g_servo_debug.invalid_target_count++;
        }
        target = Servo_LimitAngle(target);
        if (g_servo_hold) target = current;
        g_servo_target_deg[i] = target;

        float difference = target - current;
        float step = SERVO_SPEED_DPS * dt_s;
        if (difference > step) current += step;
        else if (difference < -step) current -= step;
        else current = target;
        Servo_Write(i, Servo_LimitAngle(current));
    }
    g_servo_debug.update_count++;
}

void Servo_Task(void)
{
    if (Servo_Init() != HAL_OK) Error_Handler();
    uint32_t last_wake = osKernelSysTick();
    uint32_t last_sample = last_wake;

    for (;;) {
        osDelayUntil(&last_wake, SERVO_PERIOD_MS);
        uint32_t now = osKernelSysTick();
        uint32_t elapsed_ms = now - last_sample;
        last_sample = now;
        Servo_Update((float)elapsed_ms * 0.001f);
    }
}