#ifndef __DAMIAO_H
#define __DAMIAO_H

#include "main.h"

// 达妙电机控制相关定义
typedef struct {
    float pos;
    float vel;
    float torque;
    float temp;
} Damiao_Feedback_t;

// 函数声明
void Damiao_Enable(CAN_HandleTypeDef *hcan, uint16_t motor_id);
void Damiao_Disable(CAN_HandleTypeDef *hcan, uint16_t motor_id);
void Damiao_Control_MIT(CAN_HandleTypeDef *hcan, uint16_t motor_id, float kp, float kd, float pos, float vel, float torq);
void Damiao_Unpack_Feedback(uint8_t *rx_data, Damiao_Feedback_t *feedback);
void Damiao_Control_Speed(CAN_HandleTypeDef *hcan, uint16_t motor_id, float speed);
#endif // __DAMIAO_H
