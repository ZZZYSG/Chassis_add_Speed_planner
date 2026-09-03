#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "A_star.h"          /* NODES、astar()、init_edges() */
#include "M3508.h"           /* Chassis_Speed_Control、DiPan_Motor_Stop */
#include "imu.h"
#include "odometer.h"        /* odometer.x/y: 里程计场地系位置(mm) */


/* ==================== 导航层参数 ==================== */
#define MAX_WAYPOINTS   16        /* 航点数上限(必须 >= NODES) */
#define ARRIVE_RADIUS   25.0f     /* 到达判定半径 mm */


//节点的信息
typedef struct {
    uint8_t  idx;                 /* 地图节点编号(0~NODES-1) */
    float    x_mm, y_mm;          /* 世界系坐标 mm */
} Waypoint_t;

//到达一次节点，更新一次状态
typedef struct {

    Waypoint_t wp[MAX_WAYPOINTS]; /* 节点信息 */
    uint8_t    count;             /* 航点总数 */
    uint8_t    cur;               /* 当前航点下标(目标 = count -1)用于判断是否结束 */
    uint8_t    running;           
    float      seg_tx, seg_ty;    /* 本段固定目标(段起点车体系, mm) */
    float      correct_yaw;

} Navigation_t;

/* ==================== 对外接口 ==================== */
void    Nav_Init(void);                          
uint8_t Nav_Start(uint8_t start, uint8_t goal);   /* 内部调 A*,成功返回 1 */
void    Nav_Update(void);                         /* 1ms 周期调用,内部直接读 final_yaw */
void    Nav_Stop(void);
uint8_t Nav_State(void);


#endif