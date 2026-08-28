#include "jy60.h"


static uint8_t jy60_packet[11];
static uint8_t jy60_index = 0;

uint8_t jy60_rx_byte;          // <--- 加上这一行（去掉前面的 static，因为要在 main.c 外部调用）

// 把之前写在中断里的解析逻辑封装到这里


// 在 jy60.c 中
float jy60_yaw = 0.0f;          // 原始实时角度
float final_yaw = 0.0f;         // 校准后的角度（给底盘用的）
float yaw_offset = 0.0f;        // 偏移量
uint8_t is_calibrated = 0;      // 校准标志位

void JY60_ProcessData(uint8_t rx_byte) 
{
    // 寻找包头 0x55 (必须先对齐包头)
    if (jy60_index == 0)
    {
        if (rx_byte == 0x55)
        {
            jy60_packet[0] = rx_byte;
            jy60_index = 1;
        }
        return; // 还没找到包头，退出
    }
    
    // 接收后续字节
    jy60_packet[jy60_index++] = rx_byte;

    // 当收满 11 个字节时进行解析
    if (jy60_index >= 11)
    {
        jy60_index = 0; // 重置索引，准备接收下一帧

        // 验证功能字是否为 0x53 (角度包)
        if (jy60_packet[1] == 0x53)
        {
            // 只有收到完整且正确的包，才更新角度
            int16_t yaw_raw = (int16_t)(((uint16_t)jy60_packet[7] << 8) | jy60_packet[6]);
            jy60_yaw = (float)yaw_raw / 32768.0f * 180.0f;

            // 校准逻辑
            if (!is_calibrated) 
							{
                yaw_offset = jy60_yaw; 
                is_calibrated = 1;
              }

            final_yaw = jy60_yaw - yaw_offset;
            
            // 归一化
            if (final_yaw > 180.0f)  final_yaw -= 360.0f;
            if (final_yaw < -180.0f) final_yaw += 360.0f;
        }
    }
}


void jy60_uart_rx_callback(uint8_t byte)
{
    JY60_ProcessData(byte);
    HAL_UART_Receive_IT(&huart6, &jy60_rx_byte, 1);
}
