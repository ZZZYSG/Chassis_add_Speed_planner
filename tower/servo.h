#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h" // 如果你不是F4系列，请修改为对应的头文件，如 stm32f1xx_hal.h

// 舵机结构体定义
typedef struct {
    TIM_HandleTypeDef *htim;      // 绑定的定时器句柄 (如 &htim1)
    uint32_t channel;             // 绑定的通道 (如 TIM_CHANNEL_1)

    float    current_pwm;         // 内部记录当前的 PWM 比较值 (270度建议用float防止误差累积)
    int16_t  remaining_steps;     // 剩余需要转动的步数 (度数)
    int8_t   step_dir;            // 转动方向：1 为增加 PWM，-1 为减小 PWM
    uint32_t step_delay;          // 步进延时 (控制转动速度，ms)
    uint32_t last_tick;           // 上次执行步进的时间戳
    
    float    pwm_per_degree;      // 每 1 步(度)对应的 PWM 变化量
} Servo_TypeDef;

// --- 外部可调用的函数声明 ---

// 1. 角度转 PWM 比较值 (针对270度舵机)
uint16_t Angle_To_PWM(uint16_t angle);

// 2. 初始化舵机 (设定初始角度)
void Servo_Init(Servo_TypeDef *servo, TIM_HandleTypeDef *htim, uint32_t channel, uint16_t init_angle);

// 3. 设定相对运动目标 (转动多少度，速度是多少)
void Servo_MoveRelative(Servo_TypeDef *servo, int16_t degrees, uint32_t delay_ms);

// 4. 核心更新函数 (必须放在 while(1) 中不断轮询)
void Servo_Update(Servo_TypeDef *servo);

// 5. 判断舵机是否正在转动 (返回 1 表示在转，0 表示已停止)
uint8_t Servo_IsMoving(Servo_TypeDef *servo);

#endif
