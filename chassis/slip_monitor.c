#include "slip_monitor.h"

// 非对称梯形限速器：仅"加速"(同向且 |目标| >= |当前|)时限斜率，
// 减速与反向直接跟随指令值——用于位置环输出末端，位置 PID 要减速时立即生效，
// 避免对称限速器在减速段拖慢指令导致过冲振荡。
// 注意：反向时一步跳到指令值(跨零点不限速)，急停能力由轮速环保证
float Velocity_Smoother_Asym(float target_v, float current_v, float max_accel) {
    float t_abs, c_abs, step;
    int   same_sign;

    t_abs = target_v  < 0.0f ? -target_v  : target_v;
    c_abs = current_v < 0.0f ? -current_v : current_v;
    same_sign = ((target_v < 0.0f) == (current_v < 0.0f));  // 含同时为 0

    if (same_sign && (t_abs >= c_abs)) {          // 加速或保持：限斜率
        step = target_v - current_v;
        if (step >  max_accel) step =  max_accel;
        if (step < -max_accel) step = -max_accel;
        return current_v + step;
    }
    return target_v;                              // 减速或反向：直接用指令
}

// S型速度规划器(但是暂时不用)： 跟据目标位移与当前位移数据，计算出当前速度
// 核心函数：位移 d -> 允许速度
float f(float d) {
    if (d <= 0) return 0.0;
    float term = (A_MAX * A_MAX * A_MAX * A_MAX)/(4.0 * J * J) + 2.0 * A_MAX * d;
    return - (A_MAX * A_MAX)/(2.0 * J) + sqrt(term);
}

float S_Speed_Planner(float target_pos, float current_pos, float max_speed, float max_accel){

// 每个控制周期
float s = odometer.body_odom_x;          // 已走位移，mm
float d_rem = target_pos - s;  // 剩余位移，mm
if (d_rem < 0) d_rem = 0;

// 两个约束取最小
float v_from_start = f(s);
float v_to_end     = f(d_rem);
float v_target = fminf(V_MAX, fminf(v_from_start, v_to_end));

// 限制加速度和jerk（简单版）
static float v_current = 0;
float dv_max = A_MAX * DT;  // DT为控制周期，秒
if (v_target > v_current + dv_max) v_target = v_current + dv_max;
if (v_target < v_current - dv_max) v_target = v_current - dv_max;
v_current = v_target;

return v_target;
}
