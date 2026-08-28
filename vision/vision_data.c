#include "vision_data.h"
// 全局变量定义
uint16_t vision_data[4];
uint8_t vision_frame_ready = 0;
uint8_t vision_rx_byte = 0;


void vision_frame_parse(uint8_t ucData)
{
    static uint8_t frame[10];
    static uint8_t count = 0;
    uint8_t i;

    if(count >= sizeof(frame))
    {
        count = 0;
    }

    if (count == 0)
    {
        if (ucData == 0x51)
            frame[count++] = ucData;
        return;
    }

    if (ucData == 0x51)
    {
        count = 1;
        frame[0] = ucData;
        return;
    }

    frame[count++] = ucData; 
    if (count < sizeof(frame))
        return;

    if (frame[9] == 0x52)
    {
        for (i = 0; i < 4; i++)
        {
            vision_data[i] = ((uint16_t)frame[2 + i * 2] << 8) | frame[1 + i * 2];
        }
        vision_frame_ready = 1;
    }
    count = 0;
}

void Vision_To_TJC_Update(void)
{
     //检查视觉数据是否准备就绪
   if (vision_frame_ready)
    {
        vision_frame_ready = 0; // 清除标志位，准备接收下一帧

        // 依次将四组数据发送到串口屏的 n0, n1, n2, n3 控件
        tjc_send_val("n0", "val",vision_data[0]);
        tjc_send_val("n1", "val",vision_data[1]);
        tjc_send_val("n2", "val",vision_data[2]);
        tjc_send_val("n3", "val",vision_data[3]);
    }
}
