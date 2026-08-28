#ifndef __VISION_DATA_H
#define __VISION_DATA_H

#include "stm32f4xx_hal.h"
#include "tjc_usart_hmi.h"
#include "stdint.h"
// 对外暴露
extern uint16_t vision_data[4];
extern uint8_t vision_frame_ready;
extern uint8_t vision_rx_byte;
void vision_frame_parse(uint8_t ucData);
void Vision_To_TJC_Update(void);



typedef struct {
    int16_t dx;        // X轴偏差
    int16_t dy;        // Y轴偏差
    int16_t color_id;  // 颜色ID
    int16_t status;    // 状态
} CamVisionData_t;

extern CamVisionData_t g_cam_vision;

#endif
