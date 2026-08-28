#include "speed_planner.h"

void SCurve_Init(SCurvePlanner_t *p, float dist_mm, float time_ms)
{
    p->total_dist = dist_mm;
    p->total_time = time_ms;
    p->start_time = (float)HAL_GetTick();
    p->active     = 1;
}

float SCurve_GetVel(SCurvePlanner_t *p, uint32_t tick_ms)
{
    if (!p->active) return 0.0f;

    float t = (float)tick_ms - p->start_time;
    if (t <= 0.0f) return 0.0f;
    if (t >= p->total_time) {
        p->active = 0;
        return 0.0f;
    }

    float tau = t / p->total_time;
    /* 五次多项式: v(t) = S/T * (30*tau^2 - 60*tau^3 + 30*tau^4) */
    float vel = (p->total_dist / p->total_time)
              * (30.0f*tau*tau - 60.0f*tau*tau*tau + 30.0f*tau*tau*tau*tau);
    return vel;
}

uint8_t SCurve_IsActive(SCurvePlanner_t *p) { return p->active; }
void    SCurve_Stop(SCurvePlanner_t *p)     { p->active = 0; }
