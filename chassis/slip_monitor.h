#ifndef SPEED_PLANNER_H
#define SPEED_PLANNER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "odometer.h"

#define A_LINE               300.0f  // 线加速度 mm/s²
#define W_LINE               180.0f   // 角加速度 °/s²
#define A_MAX                500.0f // 最大加速度 mm/s^2
#define V_MAX                300.0f // 最大速度 mm/s
#define J                    2000.0 // 固定加加速度 mm/s^3
#define DT                   0.001  // 控制周期


float Velocity_Smoother_Asym(float target_v, float current_v, float max_accel);  // 只限加速，减速/反向直接跟随
float S_Speed_Planner(float target_pos, float current_pos, float max_speed, float max_accel);

#endif
