#ifndef SPEED_PLANNER_H
#define SPEED_PLANNER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#define MAX_ACCEL_MM_S2  2000.0f  // 最大加速度 mm/s^2
float Velocity_Smoother(float target_v, float current_v, float max_accel);

#endif
