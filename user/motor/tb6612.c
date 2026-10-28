#include "tb6612.h"
#include "tim.h"
#include <math.h>
#include <stddef.h>

#define REVERSE_PAUSE_MS 500U
/* Conservative software settling interval, not a DRV8847 wake-time claim. */
#define ENABLE_SETTLE_MS 2U

typedef struct {
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    uint32_t channel;
    uint32_t board;
} MotorPins_t;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} BoardPins_t;

typedef struct {
    uint8_t running;
    int8_t last_direction;
    uint32_t stop_tick;
} MotorState_t;

/* M1 LF=left A; M2 RF=right A; M3 LR=left B; M4 RR=right B. */
static const MotorPins_t s_pins[TB6612_MOTOR_COUNT] = {
    {TB1_AIN1_GPIO_Port, TB1_AIN1_Pin,
     TB1_AIN2_GPIO_Port, TB1_AIN2_Pin, TIM_CHANNEL_1, 0U},
    {TB2_AIN1_GPIO_Port, TB2_AIN1_Pin,
     TB2_AIN2_GPIO_Port, TB2_AIN2_Pin, TIM_CHANNEL_3, 1U},
    {TB1_BIN1_GPIO_Port, TB1_BIN1_Pin,
     TB1_BIN2_GPIO_Port, TB1_BIN2_Pin, TIM_CHANNEL_2, 0U},
    {TB2_BIN1_GPIO_Port, TB2_BIN1_Pin,
     TB2_BIN2_GPIO_Port, TB2_BIN2_Pin, TIM_CHANNEL_4, 1U}
};
static const BoardPins_t s_boards[TB6612_BOARD_COUNT] = {
    {TB1_STBY_GPIO_Port, TB1_STBY_Pin},
    {TB2_STBY_GPIO_Port, TB2_STBY_Pin}
};
/* Starting calibration only: verify after reconnecting each motor. */
static const int8_t s_motor_direction[TB6612_MOTOR_COUNT] = {1, -1, 1, -1};
static MotorState_t s_state[TB6612_MOTOR_COUNT];
static uint32_t s_enable_tick[TB6612_BOARD_COUNT];
volatile Tb6612Debug_t g_tb6612_debug = {0};

static void StandbyBoard(uint32_t board)
{
    HAL_GPIO_WritePin(s_boards[board].port, s_boards[board].pin,
                      GPIO_PIN_RESET);
    g_tb6612_debug.enabled_board_mask &= ~(1U << board);
}

static void StopChannel(uint32_t index, uint32_t now)
{
    const MotorPins_t *pins = &s_pins[index];
    __HAL_TIM_SET_COMPARE(&htim1, pins->channel, 0U);
    /* Explicit short brake if its companion keeps the module enabled. */
    HAL_GPIO_WritePin(pins->in1_port, pins->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(pins->in2_port, pins->in2_pin, GPIO_PIN_SET);
    if (s_state[index].running) {
        s_state[index].running = 0U;
        s_state[index].stop_tick = now;
    }
    g_tb6612_debug.active_mask &= ~(1U << index);
}

void TB6612_StopAll(void)
{
    uint32_t now = HAL_GetTick();
    /* STBY low makes both outputs high impedance before changing inputs. */
    for (uint32_t board = 0U; board < TB6612_BOARD_COUNT; ++board)
        StandbyBoard(board);
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i)
        StopChannel(i, now);
}

HAL_StatusTypeDef TB6612_Init(void)
{
    g_tb6612_debug = (Tb6612Debug_t){0};
    TB6612_StopAll();
    uint32_t now = HAL_GetTick();
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        s_state[i] = (MotorState_t){0U, 0, now};
        __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, s_pins[i].channel);
        __HAL_TIM_SET_COMPARE(&htim1, s_pins[i].channel, 0U);
    }
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    HAL_StatusTypeDef status =
        HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    if (status != HAL_OK) return status;
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        status = HAL_TIM_PWM_Start(&htim1, s_pins[i].channel);
        if (status != HAL_OK) {
            TB6612_StopAll();
            return status;
        }
    }
    g_tb6612_debug.ready = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef TB6612_Apply(
    const float duty[TB6612_MOTOR_COUNT],
    float applied[TB6612_MOTOR_COUNT])
{
    if (applied != NULL)
        for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) applied[i] = 0.0f;
    if (duty == NULL || applied == NULL || !g_tb6612_debug.ready) {
        TB6612_StopAll();
        return HAL_ERROR;
    }
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    uint32_t compare[TB6612_MOTOR_COUNT] = {0};
    int8_t logical[TB6612_MOTOR_COUNT] = {0};
    int8_t physical[TB6612_MOTOR_COUNT] = {0};
    /* Reject a bad frame before applying any of its outputs. */
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        if (!isfinite(duty[i])) {
            TB6612_StopAll();
            return HAL_ERROR;
        }
        float magnitude = fminf(fabsf(duty[i]), TB6612_MAX_DUTY);
        compare[i] = (uint32_t)(magnitude * (float)period + 0.5f);
        if (compare[i] > period) compare[i] = period;
        logical[i] = duty[i] >= 0.0f ? 1 : -1;
        physical[i] = logical[i] * s_motor_direction[i];
    }
    uint32_t now = HAL_GetTick(), requested = 0U;
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        if (!compare[i]) {
            StopChannel(i, now);
            continue;
        }
        if (s_state[i].last_direction &&
            physical[i] != s_state[i].last_direction) {
            StopChannel(i, now);
            if ((uint32_t)(now - s_state[i].stop_tick) < REVERSE_PAUSE_MS) {
                compare[i] = 0U;
                continue;
            }
        }
        requested |= 1U << s_pins[i].board;
    }
    for (uint32_t board = 0U; board < TB6612_BOARD_COUNT; ++board) {
        uint32_t bit = 1U << board;
        if (!(requested & bit)) {
            StandbyBoard(board);
        } else if (!(g_tb6612_debug.enabled_board_mask & bit)) {
            /* Group members are M1/M3 or M2/M4, not adjacent indices. */
            for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i)
                if (s_pins[i].board == board) StopChannel(i, now);
            s_enable_tick[board] = now;
            HAL_GPIO_WritePin(s_boards[board].port, s_boards[board].pin,
                              GPIO_PIN_SET);
            g_tb6612_debug.enabled_board_mask |= bit;
        }
    }
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        const MotorPins_t *pins = &s_pins[i];
        if (!compare[i] ||
            (uint32_t)(now - s_enable_tick[pins->board]) < ENABLE_SETTLE_MS)
            continue;
        if (!s_state[i].running) {
            HAL_GPIO_WritePin(pins->in1_port, pins->in1_pin,
                              physical[i] > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(pins->in2_port, pins->in2_pin,
                              physical[i] > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
        /* TB has a separate PWM pin: both directions use magnitude, never 1-d. */
        __HAL_TIM_SET_COMPARE(&htim1, pins->channel, compare[i]);
        s_state[i].running = 1U;
        s_state[i].last_direction = physical[i];
        g_tb6612_debug.active_mask |= 1U << i;
        applied[i] = logical[i] * (float)compare[i] / (float)period;
    }
    return HAL_OK;
}