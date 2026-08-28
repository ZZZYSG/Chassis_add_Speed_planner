#include "servo.h"

// 1. 角度转 PWM 比较值 (针对 0~270 度)
uint16_t Angle_To_PWM(uint16_t angle) 
{
    if (angle > 270) {
        angle = 270; // 限制最大角度为 270 度
    }
    // 0度对应500, 270度对应2500
    return (uint16_t)(500.0f + ((float)angle / 270.0f) * 2000.0f);
}

// 2. 初始化舵机 (设定初始角度)
void Servo_Init(Servo_TypeDef *servo, TIM_HandleTypeDef *htim, uint32_t channel, uint16_t init_angle) 
{
    servo->htim = htim;
    servo->channel = channel;
    
    // 初始化当前 PWM 为初始角度对应的值
    servo->current_pwm = (float)Angle_To_PWM(init_angle);
    
    servo->remaining_steps = 0;
    servo->step_dir = 1;
    servo->step_delay = 0;
    servo->last_tick = HAL_GetTick();
    
    // 计算每 1 度对应的 PWM 增量：(2500 - 500) / 270 ≈ 7.4074...
    servo->pwm_per_degree = 2000.0f / 270.0f; 

    // 设置硬件定时器 (将 float 转回整数)
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, (uint16_t)servo->current_pwm);
    HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

// 3. 设定相对运动目标 (转动多少度，速度是多少)
void Servo_MoveRelative(Servo_TypeDef *servo, int16_t degrees, uint32_t delay_ms) 
{
    if (degrees == 0) return;

    if (degrees > 0) {
        servo->step_dir = 1;               // 正向转动
        servo->remaining_steps = degrees;  // 设定需要走的总步数(度)
    } else {
        servo->step_dir = -1;              // 反向转动
        servo->remaining_steps = -degrees; // 取绝对值
    }
    
    // 记录用户设置的步进延时，数值越大转得越慢
    servo->step_delay = delay_ms;
}

// 4. 核心更新函数 (必须放在 while(1) 中不断轮询)
void Servo_Update(Servo_TypeDef *servo) 
{
    if (servo->remaining_steps > 0) 
    {
        uint32_t current_tick = HAL_GetTick();
        
        // 当经过的时间达到了设定的延时，才走 1 度
        if (current_tick - servo->last_tick >= servo->step_delay) 
        {
            servo->last_tick = current_tick;

            // 浮点数加减，保证 270 度舵机的小步长精度
            if (servo->step_dir == 1) {
                servo->current_pwm += servo->pwm_per_degree;
            } else {
                servo->current_pwm -= servo->pwm_per_degree;
            }

            // 输出到硬件寄存器时，强制转换回整数
            __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, (uint16_t)servo->current_pwm);

            // 步数减一，直到归零停止
            servo->remaining_steps--;
        }
    }
}


// 5. 判断舵机是否正在转动 (返回 1 表示在转，0 表示已停止)
uint8_t Servo_IsMoving(Servo_TypeDef *servo) 
{
    return (servo->remaining_steps > 0) ? 1 : 0;
}
