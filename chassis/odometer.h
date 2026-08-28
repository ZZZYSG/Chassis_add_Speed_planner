#ifndef ODOMETER_H
#define ODOMETER_H

#include "CAN_receive.h"
#include "imu.h"
#include "math.h"

typedef struct
{
    float x;            // 机器人在世界系x方向的位置
    float y;            // 机器人在世界系y方向的位置
    float body_odom_x;  // 机器人在车体系x方向的位置
    float body_odom_y;  // 机器人在车体系y方向的位置
    float theta;        // 机器人朝向角度，单位为弧度
} Odometer;

extern Odometer odometer;
extern const motor_measure_t *motor_data[4];

/**
 * @brief  里程计初始化 (清零)
 */
void Odometer_Init(void);

/**
 * @brief  段起点清零（本段车体系累计位移归零）
 */
void Odometer_ResetSegment(void);

/**
 * @brief  里程与坐标实时更新函数
 */
void Odometer_Update(void);



#endif
