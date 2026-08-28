#include "damiao.h"

// 浮点数限制函数
static float limit_value(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// 浮点数压缩函数 (float -> uint16_t)
static uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    uint32_t val = (uint32_t)((x - offset) * ((float)((1 << bits) - 1) / span));
    return (uint16_t)val;
}

// 通用发送函数
static void Damiao_Send_Cmd(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = id;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);
}

// 使能电机
void Damiao_Enable(CAN_HandleTypeDef *hcan, uint16_t motor_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    Damiao_Send_Cmd(hcan, motor_id, data);
}

// 失能电机
void Damiao_Disable(CAN_HandleTypeDef *hcan, uint16_t motor_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    Damiao_Send_Cmd(hcan, motor_id, data);
}

// MIT 模式控制
void Damiao_Control_MIT(CAN_HandleTypeDef *hcan, uint16_t motor_id, float kp, float kd, float pos, float vel, float torq) {
    // 达妙标准范围定义
    pos = limit_value(pos, -12.5f, 12.5f);
    vel = limit_value(vel, -45.0f, 45.0f);
    kp  = limit_value(kp, 0.0f, 500.0f);
    kd  = limit_value(kd, 0.0f, 5.0f);
    torq = limit_value(torq, -18.0f, 18.0f);

    uint16_t p_int = float_to_uint(pos, -12.5f, 12.5f, 16);
    uint16_t v_int = float_to_uint(vel, -45.0f, 45.0f, 12);
    uint16_t kp_int = float_to_uint(kp, 0.0f, 500.0f, 12);
    uint16_t kd_int = float_to_uint(kd, 0.0f, 5.0f, 12);
    uint16_t t_int = float_to_uint(torq, -18.0f, 18.0f, 12);

    uint8_t data[8];
    data[0] = (p_int >> 8) & 0xFF;
    data[1] = p_int & 0xFF;
    data[2] = (v_int >> 4) & 0xFF;
    data[3] = ((v_int & 0x0F) << 4) | (kp_int >> 8);
    data[4] = kp_int & 0xFF;
    data[5] = (kd_int >> 4) & 0xFF;
    data[6] = ((kd_int & 0x0F) << 4) | (t_int >> 8);
    data[7] = t_int & 0xFF;

    Damiao_Send_Cmd(hcan, motor_id, data);
}

// 解包反馈数据 (此为标准实现，请根据手册微调)
void Damiao_Unpack_Feedback(uint8_t *rx_data, Damiao_Feedback_t *feedback) {
    uint16_t p_int = (rx_data[1] << 8) | rx_data[2];
    uint16_t v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    uint16_t t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];

    feedback->pos = (float)p_int * 25.0f / 65535.0f - 12.5f;
    feedback->vel = (float)v_int * 90.0f / 4095.0f - 45.0f;
    feedback->torque = (float)t_int * 36.0f / 4095.0f - 18.0f;
    feedback->temp = (float)rx_data[6];
}
// 速度模式控制函数（单位：rad/s 或 rpm，根据官方手册微调）
void Damiao_Control_Speed(CAN_HandleTypeDef *hcan, uint16_t motor_id, float speed)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t data[8];

    TxHeader.StdId = motor_id;
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    // 将浮点数速度转换为 4 字节发送（根据达妙速度模式协议）
    // 部分型号速度模式协议：将 float 转为 4 个字节直接塞进 data 里
    *(float*)(&data[0]) = speed; 
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0; // 后面补零或根据协议填控制字节

    HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);
}
