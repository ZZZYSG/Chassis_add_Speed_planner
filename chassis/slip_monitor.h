#ifndef SPEED_PLANNER_H
#define SPEED_PLANNER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"


float Velocity_Smoother(float target_v, float current_v, float max_accel);

#endif
