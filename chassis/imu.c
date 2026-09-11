#include "imu.h"


static uint8_t jyimu_rx_byte;       //  DMA 接收的目标缓冲（每收 1 字节触发一次回调）
static uint8_t jyimu_frame[11];     //  组帧缓冲（状态机逐字节装填，与 DMA 目标分离）
static uint8_t jyimu_index = 0;     //  数据帧的索引


static int16_t yaw_raw;             //  角度的实际数据
float jyimu_yaw = 0.0f;             //  世界系下的角度
float final_yaw = 0.0f;             //  场地系下的角度（校准后，全局唯一定义）

float yaw_offset = 0.0f;            //  偏移量
uint8_t is_calibrated = 0;          //  校准标志位


extern UART_HandleTypeDef huart6;

/* ========== 私有函数 ========== */
static void JYIMU_ParseByte(uint8_t rx_byte){
	

    if (jyimu_index == 0){
        if (rx_byte == 0x55)
        {
            jyimu_frame[0] = rx_byte;
            jyimu_index = 1;
        }
        return; 
    }
		
		jyimu_frame[jyimu_index++] = rx_byte;
		if (jyimu_index >= 11){
				jyimu_index = 0;
				if (jyimu_frame[1] == 0x53){
					yaw_raw = (int16_t)(((uint16_t)jyimu_frame[7] << 8) | jyimu_frame[6]);
					jyimu_yaw = (float)yaw_raw / 32768.0f * 180.0f;
		
					
		if(!is_calibrated){
					yaw_offset = jyimu_yaw;
          is_calibrated = 1;
		}
		final_yaw = jyimu_yaw - yaw_offset;
		if (final_yaw > 180.0f)  final_yaw -= 360.0f;
        if (final_yaw < -180.0f) final_yaw += 360.0f;
		}
		
		}
} 

/* ========== 对外接口 ========== */
void JYIMU_Init(void)
{
   HAL_UART_Receive_DMA(&huart6, &jyimu_rx_byte, 1);   // DMA 目标：单字节缓冲
}

/* USART6 每收到 1 字节，由 main.c 的 HAL_UART_RxCpltCallback 转发到这里 */
void JYIMU_Uart6_RxCpltCallback(void)
{
    JYIMU_ParseByte(jyimu_rx_byte);                    // 1. 喂给状态机解析
    HAL_UART_Receive_DMA(&huart6, &jyimu_rx_byte, 1);  // 2. 重新武装 DMA，准备收下一字节
}


float JYIMU_GetYaw(void)
{
    return jyimu_yaw;
}
