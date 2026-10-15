#include "tb6612.h"
#include "tim.h"
#include <math.h>

#define REVERSE_PAUSE_MS 500U
/* Positive duty makes the mounted wheel roll forward. */
static const int8_t s_motor_direction[TB6612_MOTOR_COUNT] = {1, -1, 1, -1};

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

/* Index 0/1/2/3 = left-front/right-front/left-rear/right-rear. */
static const MotorPins_t s_pins[TB6612_MOTOR_COUNT] = {
    {TB_AIN1_GPIO_Port, TB_AIN1_Pin,
     TB_AIN2_GPIO_Port, TB_AIN2_Pin, TIM_CHANNEL_1, 0U},
    {TB_BIN1_GPIO_Port, TB_BIN1_Pin,
     TB_BIN2_GPIO_Port, TB_BIN2_Pin, TIM_CHANNEL_2, 0U},
    {TB2_AIN1_GPIO_Port, TB2_AIN1_Pin,
     TB2_AIN2_GPIO_Port, TB2_AIN2_Pin, TIM_CHANNEL_3, 1U},
    {TB2_BIN1_GPIO_Port, TB2_BIN1_Pin,
     TB2_BIN2_GPIO_Port, TB2_BIN2_Pin, TIM_CHANNEL_4, 1U}
};

static const BoardPins_t s_boards[2] = {
    {TB_STBY_GPIO_Port, TB_STBY_Pin},
    {TB2_STBY_GPIO_Port, TB2_STBY_Pin}
};

static MotorState_t s_state[TB6612_MOTOR_COUNT];
static uint8_t s_ready = 0U;

static void UpdateStandby(uint32_t board)
{
    uint32_t first = board * 2U;
    uint8_t awake = s_state[first].running || s_state[first + 1U].running;
    HAL_GPIO_WritePin(s_boards[board].port, s_boards[board].pin,
                      awake ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void StopChannel(uint32_t index)
{
    const MotorPins_t *pins = &s_pins[index];
    MotorState_t *state = &s_state[index];

    __HAL_TIM_SET_COMPARE(&htim1, pins->channel, 0U);
    /* IN1=IN2=H: short brake if the other motor keeps this board awake. */
    HAL_GPIO_WritePin(pins->in1_port, pins->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(pins->in2_port, pins->in2_pin, GPIO_PIN_SET);
    if (state->running != 0U) {
        state->stop_tick = HAL_GetTick();
        state->running = 0U;
    }
}

void TB6612_StopAll(void)
{
    /* Shut down both boards before changing any channel state. */
    for (uint32_t board = 0U; board < 2U; ++board) {
        HAL_GPIO_WritePin(s_boards[board].port, s_boards[board].pin,
                          GPIO_PIN_RESET);
    }
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        StopChannel(i);
    }
}

HAL_StatusTypeDef TB6612_Init(void)
{
    s_ready = 0U;
    TB6612_StopAll();
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        s_state[i].running = 0U;
        s_state[i].last_direction = 0;
        s_state[i].stop_tick = HAL_GetTick();
    }

    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    HAL_StatusTypeDef status =
        HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE);
    if (status != HAL_OK) {
        return status;
    }
    for (uint32_t i = 0U; i < TB6612_MOTOR_COUNT; ++i) {
        status = HAL_TIM_PWM_Start(&htim1, s_pins[i].channel);
        if (status != HAL_OK) {
            TB6612_StopAll();
            return status;
        }
    }
    s_ready = 1U;
    return HAL_OK;
}

float TB6612_SetDuty(uint32_t motor_index, float duty)
{
    if (motor_index >= TB6612_MOTOR_COUNT) {
        return 0.0f;
    }
    if (s_ready == 0U) {
        TB6612_StopAll();
        return 0.0f;
    }

    const MotorPins_t *pins = &s_pins[motor_index];
    MotorState_t *state = &s_state[motor_index];
    if (!isfinite(duty)) {
        duty = 0.0f;
    }
    if (duty > TB6612_MAX_DUTY) duty = TB6612_MAX_DUTY;
    if (duty < -TB6612_MAX_DUTY) duty = -TB6612_MAX_DUTY;

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    float magnitude = (duty >= 0.0f) ? duty : -duty;
    uint32_t compare = (uint32_t)(magnitude * (float)period + 0.5f);
    if (compare == 0U) {
        StopChannel(motor_index);
        UpdateStandby(pins->board);
        return 0.0f;
    }

    int8_t logical_direction = (duty > 0.0f) ? 1 : -1;
    int8_t direction = logical_direction * s_motor_direction[motor_index];
    if ((state->last_direction != 0) &&
        (direction != state->last_direction)) {
        StopChannel(motor_index);
        UpdateStandby(pins->board);
        if ((uint32_t)(HAL_GetTick() - state->stop_tick) < REVERSE_PAUSE_MS) {
            return 0.0f;
        }
    }
    if (state->running == 0U) {
        /* This channel's PWM has been zero during the stop interval. */
        HAL_GPIO_WritePin(pins->in1_port, pins->in1_pin,
                          (direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pins->in2_port, pins->in2_pin,
                          (direction > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        state->last_direction = direction;
        state->running = 1U;
    }
    __HAL_TIM_SET_COMPARE(&htim1, pins->channel, compare);
    UpdateStandby(pins->board);
    return ((float)compare / (float)period) * (float)logical_direction;
}