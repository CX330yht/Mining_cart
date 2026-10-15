#include "mecanum.h"
#include <math.h>
#include <stddef.h>

#define WHEEL_RADIUS_M 0.030f
#define HALF_LENGTH_PLUS_HALF_WIDTH_M 0.32725f
#define PI_F 3.14159265358979323846f

uint8_t Mecanum_Calculate(float vx, float vy, float wz,
                         float max_rpm, float wheel_rpm[4], float *scale)
{
    if (scale != NULL) *scale = 0.0f;
    if (wheel_rpm == NULL) return 0U;
    for (uint32_t i = 0U; i < MECANUM_WHEEL_COUNT; ++i) {
        wheel_rpm[i] = 0.0f;
    }
    if (!isfinite(vx) || !isfinite(vy) || !isfinite(wz) ||
        !isfinite(max_rpm) || max_rpm <= 0.0f) {
        return 0U;
        }

    float turn = HALF_LENGTH_PLUS_HALF_WIDTH_M * wz;
    float factor = 60.0f / (2.0f * PI_F * WHEEL_RADIUS_M);
    float raw[MECANUM_WHEEL_COUNT] = {
        (vx - vy - turn) * factor,
        (vx + vy + turn) * factor,
        (vx + vy - turn) * factor,
        (vx - vy + turn) * factor
    };
    float peak = 0.0f;
    for (uint32_t i = 0U; i < MECANUM_WHEEL_COUNT; ++i) {
        if (!isfinite(raw[i])) return 0U;
        if (fabsf(raw[i]) > peak) peak = fabsf(raw[i]);
    }

    float ratio = peak > max_rpm ? max_rpm / peak : 1.0f;
    for (uint32_t i = 0U; i < MECANUM_WHEEL_COUNT; ++i) {
        float value = raw[i] * ratio;
        /* Only remove floating-point rounding beyond the common limit. */
        if (value > max_rpm) value = max_rpm;
        if (value < -max_rpm) value = -max_rpm;
        wheel_rpm[i] = value;
    }
    if (scale != NULL) *scale = ratio;
    return 1U;
}