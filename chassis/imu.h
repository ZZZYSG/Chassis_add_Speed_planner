#ifndef IMU_H
#define IMU_H

#include "stm32f4xx_hal.h"

extern float final_yaw;                     //  场地系下的角度（校准后，在 imu.c 中定义）

void JYIMU_Init(void);                      //  初始化：启动 USART6 的 DMA 逐字节接收
void JYIMU_Uart6_RxCpltCallback(void);      //  USART6 收到 1 字节：解析 + 重新开启 DMA 接收
float JYIMU_GetYaw(void);                   //  获取当前世界系角度

#endif
