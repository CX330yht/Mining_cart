#ifndef MECANUM_H
#define MECANUM_H

#include <stdint.h>

#define MECANUM_WHEEL_COUNT 4U

/* vx: forward m/s; vy: left m/s; wz: counterclockwise rad/s. */
/* Wheel order: left-front, right-front, left-rear, right-rear. */
/* Returns 1 on success. Invalid inputs clear outputs and return 0. */
uint8_t Mecanum_Calculate(float vx, float vy, float wz,
                         float max_rpm, float wheel_rpm[4], float *scale);

#endif