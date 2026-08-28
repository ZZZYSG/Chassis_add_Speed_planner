/**
  ****************************(C) COPYRIGHT 2019 DJI****************************

  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
	
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

#define CHASSIS_CAN   hcan2      //CAN串口2(底盘电机接在CAN2)
//#define GIMBAL_CAN    hcan1      //CAN串口1

/* CAN send and receive ID */ 
// CAN 发送和接收结构体 ID
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
    CAN_YAW_MOTOR_ID = 0x205,
   
} can_msg_id_e;

/* CAN send and receive msg */ 
// CAN 发送的电流和接收的转速/机械角
typedef struct
{   
    uint16_t ecd;          
    uint16_t  last_ecd;    /* 上一周期机械角(原始值) */
    int16_t  speed_rpm;
    int16_t  given_current;
	
	
} motor_measure_t;

/* 底盘电机反馈数据数组(由 main.c 中的 CAN 接收中断回调填充) */
extern motor_measure_t motor_chassis[5];

// 电机反馈数据解析宏
#define get_motor_measure(ptr, data)                                 \
{                                                                    \
    (ptr)->last_ecd = (ptr)->ecd;                                    \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);             \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);       \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);   \
}

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif

