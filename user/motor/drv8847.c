#include "drv8847.h"
#include "tim.h"
#include <math.h>
#include <stddef.h>

#define WAKE_DELAY_MS 2U
#define SLEEP_MIN_MS 1U
#define REVERSE_PAUSE_MS 500U

typedef struct {
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    uint32_t channel;
    uint32_t board;
} MotorPins_t;

typedef struct {
    GPIO_TypeDef *sleep_port;
    uint16_t sleep_pin;
    GPIO_TypeDef *fault_port;
    uint16_t fault_pin;
} BoardPins_t;

typedef struct {
    uint8_t running;
    int8_t last_direction;
    uint32_t stop_tick;
} MotorState_t;

/* Index 0..3 = left-front, right-front, left-rear, right-rear. */
static const MotorPins_t s_pins[DRV8847_MOTOR_COUNT] = {
    {D1_IN2_GPIO_Port, D1_IN2_Pin, TIM_CHANNEL_1, 0U},
    {D1_IN4_GPIO_Port, D1_IN4_Pin, TIM_CHANNEL_2, 0U},
    {D2_IN2_GPIO_Port, D2_IN2_Pin, TIM_CHANNEL_3, 1U},
    {D2_IN4_GPIO_Port, D2_IN4_Pin, TIM_CHANNEL_4, 1U}
};

static const BoardPins_t s_boards[DRV8847_BOARD_COUNT] = {
    {D1_NSLEEP_GPIO_Port, D1_NSLEEP_Pin,
     D1_NFAULT_GPIO_Port, D1_NFAULT_Pin},
    {D2_NSLEEP_GPIO_Port, D2_NSLEEP_Pin,
     D2_NFAULT_GPIO_Port, D2_NFAULT_Pin}
};

/* Existing mounted-wheel calibration, assuming motor M+ goes to connector pin1. */
static const int8_t s_motor_direction[DRV8847_MOTOR_COUNT] = {1, -1, 1, -1};
/* CN1 pin1/6=OUT1/OUT2; CN2 pin1/6=OUT4/OUT3. */
static const int8_t s_port_direction[DRV8847_MOTOR_COUNT] = {1, -1, 1, -1};

static MotorState_t s_state[DRV8847_MOTOR_COUNT];
static uint32_t s_wake_tick[DRV8847_BOARD_COUNT];
static uint32_t s_sleep_tick[DRV8847_BOARD_COUNT];
volatile Drv8847Debug_t g_drv8847_debug = {0};

static void SleepBoard(uint32_t board, uint32_t now)
{
    uint32_t bit = 1U << board;
    HAL_GPIO_WritePin(s_boards[board].sleep_port,
                      s_boards[board].sleep_pin, GPIO_PIN_RESET);
    if ((g_drv8847_debug.awake_mask & bit) != 0U) {
        s_sleep_tick[board] = now;
    }
    g_drv8847_debug.awake_mask &= ~bit;
}

static void CoastChannel(uint32_t index, uint32_t now)
{
    const MotorPins_t *pins = &s_pins[index];
    MotorState_t *state = &s_state[index];
    __HAL_TIM_SET_COMPARE(&htim1, pins->channel, 0U);
    HAL_GPIO_WritePin(pins->dir_port, pins->dir_pin, GPIO_PIN_RESET);
    if (state->running != 0U) {
        state->running = 0U;
        state->stop_tick = now;
    }
    g_drv8847_debug.active_mask &= ~(1U << index);
}

void DRV8847_StopAll(void)
{
    uint32_t now = HAL_GetTick();
    /* Sleep both boards before changing channel input levels. */
    for (uint32_t board = 0U; board < DRV8847_BOARD_COUNT; ++board) {
        SleepBoard(board, now);
    }
    for (uint32_t i = 0U; i < DRV8847_MOTOR_COUNT; ++i) {
        CoastChannel(i, now);
    }
}

void DRV8847_PollFaults(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t raw = 0U;
    uint32_t qualified = 0U;
    for (uint32_t board = 0U; board < DRV8847_BOARD_COUNT; ++board) {
        uint32_t bit = 1U << board;
        if (HAL_GPIO_ReadPin(s_boards[board].fault_port,
                            s_boards[board].fault_pin) == GPIO_PIN_RESET) {
            raw |= bit;
        }
        if ((g_drv8847_debug.awake_mask & bit) != 0U &&
            (uint32_t)(now - s_wake_tick[board]) >= WAKE_DELAY_MS) {
            qualified |= bit;
        }
    }
    g_drv8847_debug.raw_fault_mask = raw;
    g_drv8847_debug.fault_mask |= raw & qualified;
    if (g_drv8847_debug.fault_mask != 0U) {
        DRV8847_StopAll();
    }
}

void DRV8847_ClearFaults(void)
{
    /* The control task calls this only while disabled and stopped. */
    DRV8847_StopAll();
    g_drv8847_debug.fault_mask = 0U;
}

HAL_StatusTypeDef DRV8847_Init(void)
{
    uint32_t now = HAL_GetTick();
    g_drv8847_debug.ready = 0U;
    g_drv8847_debug.awake_mask = 0U;
    g_drv8847_debug.active_mask = 0U;
    g_drv8847_debug.raw_fault_mask = 0U;
    g_drv8847_debug.fault_mask = 0U;
    for (uint32_t board = 0U; board < DRV8847_BOARD_COUNT; ++board) {
        s_wake_tick[board] = now;
        s_sleep_tick[board] = now;
    }
    DRV8847_StopAll();
    for (uint32_t i = 0U; i < DRV8847_MOTOR_COUNT; ++i) {
        s_state[i].running = 0U;
        s_state[i].last_direction = 0;
        s_state[i].stop_tick = now;
        /* Immediate CCR writes allow an input pair to enter coast promptly. */
        __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, s_pins[i].channel);
        __HAL_TIM_SET_COMPARE(&htim1, s_pins[i].channel, 0U);
    }
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    HAL_StatusTypeDef status =
        HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    if (status != HAL_OK) return status;
    for (uint32_t i = 0U; i < DRV8847_MOTOR_COUNT; ++i) {
        status = HAL_TIM_PWM_Start(&htim1, s_pins[i].channel);
        if (status != HAL_OK) {
            DRV8847_StopAll();
            return status;
        }
    }
    g_drv8847_debug.ready = 1U;
    DRV8847_PollFaults();
    return HAL_OK;
}

HAL_StatusTypeDef DRV8847_Apply(
    const float duty[DRV8847_MOTOR_COUNT],
    float applied[DRV8847_MOTOR_COUNT])
{
    if (applied != NULL) {
        for (uint32_t i = 0U; i < DRV8847_MOTOR_COUNT; ++i) applied[i] = 0.0f;
    }
    if (duty == NULL || applied == NULL || g_drv8847_debug.ready == 0U) {
        DRV8847_StopAll();
        return HAL_ERROR;
    }

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    uint32_t count[DRV8847_MOTOR_COUNT] = {0};
    int8_t direction[DRV8847_MOTOR_COUNT] = {0};
    int8_t logical_direction[DRV8847_MOTOR_COUNT] = {0};
    /* Validate the whole frame before changing any output. */
    for (uint32_t i = 0U; i < DRV8847_MOTOR_COUNT; ++i) {
        float value = duty[i];
        if (!isfinite(value)) {
            DRV8847_StopAll();
            return HAL_ERROR;
        }
        if (value > DRV8847_MAX_DUTY) value = DRV8847_MAX_DUTY;
        if (value < -DRV8847_MAX_DUTY) value = -DRV8847_MAX_DUTY;
        logical_direction[i] = value >= 0.0f ? 1 : -1;
        direction[i] = logical_direction[i] * s_motor_direction[i] * s_port_direction[i];
        count[i] = (uint32_t)(fabsf(value) * (float)period + 0.5f);
        if (count[i] > period) count[i] = period;
    }

    DRV8847_PollFaults();
    if (g_drv8847_debug.fault_mask != 0U) return HAL_ERROR;
    uint32_t now = HAL_GetTick();
    uint32_t requested_boards = 0U;
    for (uint32_t i = 0U; i < DRV8847_MOTOR_COUNT; ++i) {
        MotorState_t *state = &s_state[i];
        if (count[i] == 0U) {
            CoastChannel(i, now);
            continue;
        }
        if (state->last_direction != 0 && direction[i] != state->last_direction) {
            CoastChannel(i, now);
            if ((uint32_t)(now - state->stop_tick) < REVERSE_PAUSE_MS) {
                count[i] = 0U;
                continue;
            }
        }
        requested_boards |= 1U << s_pins[i].board;
    }

    for (uint32_t board = 0U; board < DRV8847_BOARD_COUNT; ++board) {
        uint32_t bit = 1U << board;
        if ((requested_boards & bit) == 0U) {
            SleepBoard(board, now);
        } else if ((g_drv8847_debug.awake_mask & bit) == 0U) {
            CoastChannel(board * 2U, now);
            CoastChannel(board * 2U + 1U, now);
            if ((uint32_t)(now - s_sleep_tick[board]) < SLEEP_MIN_MS) continue;
            s_wake_tick[board] = now;
            HAL_GPIO_WritePin(s_boards[board].sleep_port,
                              s_boards[board].sleep_pin, GPIO_PIN_SET);
            g_drv8847_debug.awake_mask |= bit;
        }
    }

    for (uint32_t i = 0U; i < DRV8847_MOTOR_COUNT; ++i) {
        const MotorPins_t *pins = &s_pins[i];
        uint32_t bit = 1U << pins->board;
        if (count[i] == 0U || (g_drv8847_debug.awake_mask & bit) == 0U ||
            (uint32_t)(now - s_wake_tick[pins->board]) < WAKE_DELAY_MS) continue;
        if (s_state[i].running == 0U) {
            HAL_GPIO_WritePin(pins->dir_port, pins->dir_pin,
                              direction[i] > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
        /* Reverse is driven while PWM is low: use the complement. */
        uint32_t compare = direction[i] > 0 ? count[i] : period - count[i];
        __HAL_TIM_SET_COMPARE(&htim1, pins->channel, compare);
        s_state[i].running = 1U;
        s_state[i].last_direction = direction[i];
        g_drv8847_debug.active_mask |= 1U << i;
        applied[i] = ((float)count[i] / (float)period) * (float)logical_direction[i];
    }
    return HAL_OK;
}