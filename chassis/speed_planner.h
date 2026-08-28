#ifndef SPEED_PLANNER_H
#define SPEED_PLANNER_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct {
    float total_dist;    // 总路程 (mm)
    float total_time;    // 总时间 (ms)
    float start_time;    // 起始时刻 (ms)
    uint8_t active;      // 是否运行中
} SCurvePlanner_t;

void    SCurve_Init(SCurvePlanner_t *p, float dist_mm, float time_ms);
float   SCurve_GetVel(SCurvePlanner_t *p, uint32_t tick_ms);
uint8_t SCurve_IsActive(SCurvePlanner_t *p);
void    SCurve_Stop(SCurvePlanner_t *p);
#endif
