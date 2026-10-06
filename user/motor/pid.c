#include "pid.h"
#include <stddef.h>

static float Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL) return;
    pid->integral = 0.0f;
    pid->last_measure = 0.0f;
    pid->has_last = 0;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float output_limit)
{
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    PID_Reset(pid);
}

float PID_Calculate(PID_t *pid, float target, float measure, float dt_s)
{
    if (pid == NULL || dt_s <= 0.0f || pid->output_limit <= 0.0f) {
        return 0.0f;
    }

    float error = target - measure;
    float proportional = pid->kp * error;
    float derivative = 0.0f;

    if (pid->has_last) {
        derivative = -pid->kd * (measure - pid->last_measure) / dt_s;
    }
    pid->last_measure = measure;
    pid->has_last = 1;

    float next_integral =
        Clamp(pid->integral + pid->ki * error * dt_s, pid->output_limit);
    float proposed = proportional + next_integral + derivative;

    /* 输出已饱和且误差还想继续增大输出时，暂停积分。 */
    if ((proposed > pid->output_limit && error > 0.0f) ||
        (proposed < -pid->output_limit && error < 0.0f)) {
        next_integral = pid->integral;
        }

    pid->integral = next_integral;
    return Clamp(proportional + pid->integral + derivative,
                 pid->output_limit);
}