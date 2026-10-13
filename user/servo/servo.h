#ifndef SERVO_H
#define SERVO_H

#include "main.h"

enum {
    SERVO_A = 0,
    SERVO_B,
    SERVO_C1,
    SERVO_C2,
    SERVO_COUNT
};

typedef struct {
    float angle_deg[SERVO_COUNT]; /* Commanded output, not measured position. */
    uint16_t pulse_us[SERVO_COUNT];
    HAL_StatusTypeDef start_status[SERVO_COUNT];
    uint32_t update_count;
    uint32_t invalid_target_count;
    uint8_t ready;
} ServoDebug_t;

/* Bench controls for Ozone. Only ServoTask writes timer compare registers. */
extern volatile float g_servo_target_deg[SERVO_COUNT];
extern volatile uint8_t g_servo_hold;
extern volatile ServoDebug_t g_servo_debug;

HAL_StatusTypeDef Servo_Init(void);
void Servo_Update(float dt_s);
void Servo_Task(void);

#endif