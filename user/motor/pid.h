#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float last_measure;
    float output_limit;
    uint8_t has_last;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_limit);
void PID_Reset(PID_t *pid);
float PID_Calculate(PID_t *pid, float target, float measure, float dt_s);

#endif