#include "slip_monitor.h"

// 简单的加速度限制滤波器
float Velocity_Smoother(float target_v, float current_v, float max_accel) {
    if (target_v > current_v + max_accel) {
        return current_v + max_accel; // 限制加速率
    } 
    else if (target_v < current_v - max_accel) {
        return current_v - max_accel; // 限制减速率
    } 
    else {
        return target_v; // 到达目标速度
    }
}
