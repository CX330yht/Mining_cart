#include "servo_soft_pwm.h"
#include "tim.h"

#define SERVO_FRAME_US 20000U
#define SERVO_MIN_US 500U
#define SERVO_MAX_US 2500U

volatile ServoSoftPwmDebug_t g_servo_soft_pwm_debug;
static uint16_t s_requested[2] = {1500U, 1500U};
static uint16_t s_active[2];
static uint16_t s_next_us;
static uint8_t s_pending_mask;

static uint32_t Lock(void)
{
    uint32_t previous = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return previous;
}

static void Unlock(uint32_t previous)
{
    __DMB();
    __set_PRIMASK(previous);
}

static void WritePin(uint32_t index, GPIO_PinState level)
{
    if (index == 0U)
        HAL_GPIO_WritePin(SERVO_A_PWM_GPIO_Port, SERVO_A_PWM_Pin, level);
    else
        HAL_GPIO_WritePin(SERVO_B_PWM_GPIO_Port, SERVO_B_PWM_Pin, level);
}

static void StopLocked(void)
{
    if (htim7.Instance == TIM7) {
        (void)HAL_TIM_Base_Stop_IT(&htim7);
        __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    }
    WritePin(0U, GPIO_PIN_RESET);
    WritePin(1U, GPIO_PIN_RESET);
    s_pending_mask = 0U;
    g_servo_soft_pwm_debug.running = 0U;
}

static void NextInterval(uint32_t microseconds)
{
    /* TIM7 is a 1 MHz counter, ARPE disabled. No busy waiting. */
    __HAL_TIM_SET_AUTORELOAD(&htim7, microseconds - 1U);
    __HAL_TIM_SET_COUNTER(&htim7, 0U);
}

static void BeginFrame(void)
{
    s_active[0] = s_requested[0];
    s_active[1] = s_requested[1];
    g_servo_soft_pwm_debug.active_pulse_us[0] = s_active[0];
    g_servo_soft_pwm_debug.active_pulse_us[1] = s_active[1];
    s_pending_mask = 3U;
    s_next_us = s_active[0] < s_active[1] ? s_active[0] : s_active[1];
    WritePin(0U, GPIO_PIN_SET);
    WritePin(1U, GPIO_PIN_SET);
    g_servo_soft_pwm_debug.frame_count++;
    NextInterval(s_next_us);
}

HAL_StatusTypeDef ServoSoftPwm_Init(void)
{
    uint32_t previous = Lock();
    g_servo_soft_pwm_debug = (ServoSoftPwmDebug_t){0};
    StopLocked();
    /* This project uses an 84 MHz APB1 timer clock. */
    if (htim7.Instance != TIM7 || htim7.Init.Prescaler != 83U ||
        htim7.Init.AutoReloadPreload != TIM_AUTORELOAD_PRELOAD_DISABLE) {
        Unlock(previous);
        return HAL_ERROR;
    }
    s_requested[0] = s_requested[1] = 1500U;
    g_servo_soft_pwm_debug.pulse_us[0] = 1500U;
    g_servo_soft_pwm_debug.pulse_us[1] = 1500U;
    g_servo_soft_pwm_debug.ready = 1U;
    Unlock(previous);
    return HAL_OK;
}

void ServoSoftPwm_SetPulseUs(uint32_t index, uint16_t pulse_us)
{
    if (index >= 2U) return;
    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;
    uint32_t previous = Lock();
    s_requested[index] = pulse_us;
    g_servo_soft_pwm_debug.pulse_us[index] = pulse_us;
    Unlock(previous);
}

HAL_StatusTypeDef ServoSoftPwm_Start(void)
{
    uint32_t previous = Lock();
    if (!g_servo_soft_pwm_debug.ready) {
        Unlock(previous);
        return HAL_ERROR;
    }
    if (g_servo_soft_pwm_debug.running) {
        Unlock(previous);
        return HAL_OK;
    }
    __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    BeginFrame();
    g_servo_soft_pwm_debug.running = 1U;
    HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(&htim7);
    g_servo_soft_pwm_debug.start_status = (uint32_t)status;
    if (status != HAL_OK) StopLocked();
    Unlock(previous);
    return status;
}

void ServoSoftPwm_Stop(void)
{
    uint32_t previous = Lock();
    StopLocked();
    Unlock(previous);
}

void ServoSoftPwm_OnTimerElapsed(void)
{
    /* TIM7 ISR only: integer operations, no RTOS, delays or printing. */
    if (!g_servo_soft_pwm_debug.running) return;
    g_servo_soft_pwm_debug.irq_count++;
    if (!s_pending_mask) {
        BeginFrame();
        return;
    }
    uint16_t elapsed = s_next_us;
    for (uint32_t i = 0U; i < 2U; ++i) {
        if ((s_pending_mask & (1U << i)) && s_active[i] <= elapsed) {
            WritePin(i, GPIO_PIN_RESET);
            s_pending_mask &= (uint8_t)~(1U << i);
        }
    }
    if (s_pending_mask) {
        s_next_us = (s_pending_mask & 1U) ? s_active[0] : s_active[1];
    } else {
        s_next_us = SERVO_FRAME_US;
    }
    NextInterval((uint32_t)s_next_us - elapsed);
}