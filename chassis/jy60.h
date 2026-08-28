#ifndef __JY60_H
#define __JY60_H

#include "main.h"
#include "usart.h"
// 声明外部可调用的变量或函数
extern float final_yaw ;  
extern float jy60_yaw; 
void JY60_ProcessData(uint8_t byte); // 状态机解析函数
extern uint8_t jy60_rx_byte;
void jy60_uart_rx_callback(uint8_t byte);

#endif
