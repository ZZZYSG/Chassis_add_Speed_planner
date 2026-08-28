#include "navigation.h"

#define PI  3.1415926f

/* 导航层全局状态(仅本文件可见),所有函数共享这一个实例 */
extern float final_yaw;
extern float pos_hold_yaw;
static Navigation_t nav;


/* ==================== 3×3 拓扑节点 -> 世界坐标 (mm) ====================
 * 表编号 1~9 = 代码索引 0~8(按行序,row-major):
 *   编号 |  Y   |  X   | (X, Y)
 *    1   | 275  | 350  | (350,  275)
 *    2   | 275  | 1200 | (1200, 275)
 *    3   | 275  | 2080 | (2080, 275)
 *    4   | 1200 | 350  | (350, 1200)
 *    5   | 1200 | 1200 | (1200,1200)
 *    6   | 1200 | 2080 | (2080,1200)
 *    7   | 2050 | 350  | (350, 2050)
 *    8   | 2050 | 1200 | (1200,2050)
 *    9   | 2050 | 2080 | (2080,2050)
 * 格距不相等(X: 850/880, Y: 925/850),所以查表,不用统一格长
 */
static const float NODE_X[NODES] = {
    350.0f, 1200.0f, 2080.0f,     /* 第 0 行 */
    350.0f, 1200.0f, 2080.0f,     /* 第 1 行 */
    350.0f, 1200.0f, 2080.0f      /* 第 2 行 */
};
static const float NODE_Y[NODES] = {
    275.0f,  275.0f,  275.0f,
    1200.0f, 1200.0f, 1200.0f,
    2050.0f, 2050.0f, 2050.0f
};

/* 换段: 把本段场地系误差转成段起点车体系相对位移, 并清段里程,航线角锁定在0，90，180(-180)，根据实际情况确认 */
static void Nav_LoadSegment(void)
{
    float ex = nav.wp[nav.cur + 1].x_mm - odometer.x;
    float ey = nav.wp[nav.cur + 1].y_mm - odometer.y;

    float yaw_rad = final_yaw * PI / 180.0f;
    float cos_y = cosf(yaw_rad);
    float sin_y = sinf(yaw_rad);

    nav.seg_tx =  ex * cos_y + ey * sin_y;   /* R(-yaw)·(ex,ey) */
    nav.seg_ty = -ex * sin_y + ey * cos_y;

    if(-45 < final_yaw && final_yaw < 45) 
    { nav.correct_yaw = 0;  } 
    else if(45 < final_yaw && final_yaw < 135)
    { nav.correct_yaw = 90; }
    else if(final_yaw < -135 || final_yaw > 135)
    { nav.correct_yaw = 180; }
    else if(final_yaw > -135 && final_yaw < -45)
    { nav.correct_yaw = -90; }

    pos_hold_yaw = nav.correct_yaw;

    //速度规划器单位方向向量
    switch((int)(nav.correct_yaw))
    {
    case 0:    nav.seg_dir_x = 1.0f, nav.seg_dir_y = 0.0f;    break;
    case 90:   nav.seg_dir_x = 0.0f, nav.seg_dir_y = 1.0f;    break;
    case 180:  nav.seg_dir_x =-1.0f, nav.seg_dir_y = 0.0f;    break;
    case -90:  nav.seg_dir_x = 0.0f, nav.seg_dir_y = -1.0f;   break;
    }
    //当前段的长度
    nav.seg_len = fabs(nav.seg_tx) + fabs(nav.seg_ty);
    //估算时间
    float time_ms = nav.seg_len / PLAN_V_MAX * 1500.0f;
    if (time_ms < 500.0f) time_ms = 500.0f; 
    SCurve_Init(&nav.sc, nav.seg_len, time_ms);

    Odometer_ResetSegment();   /* body_odom_x/y 归零, 与 seg_tx/ty 配套 */
}

/* ==================== 对外接口 ==================== */
void Nav_Init(void)
{
    memset(&nav, 0, sizeof(nav));
}


uint8_t Nav_Start(uint8_t start, uint8_t goal)
{
    uint8_t path[MAX_WAYPOINTS];                    //存储算出的路径
    
    if (start >= NODES || goal >= NODES) return 0;  //不存在的点
    if (start == goal) return 0;                    //已经在原位

    /* 内部调 A*,path[] 是从起点到终点的节点序列 */
    uint8_t len = astar(start, goal, path, MAX_WAYPOINTS);
    if (len < 2) { nav.running = 0; return 0; }   /* 没路可走 */

    //提取路径信息
    for (uint8_t i = 0; i < len && i < MAX_WAYPOINTS; i++) {
        nav.wp[i].idx  = path[i];
        nav.wp[i].x_mm = NODE_X[path[i]];
        nav.wp[i].y_mm = NODE_Y[path[i]];
    }
    nav.count   = len;
    nav.cur     = 0;
    /* 前置条件: 调用前 Odometer_Init 已清零,且车实际停在起点节点的世界坐标上 */
    Nav_LoadSegment();
    nav.running = 1;
    return 1;
}




/* 每个控制周期调用一次,内部直接读里程计 odometer 和 IMU 的 final_yaw */
void Nav_Update(void)
{
    if (!nav.running) return;         //还没有导航信息
    if (nav.cur >= nav.count - 1) { Nav_Stop(); return; }   //导航完了

    /* 1. 当前位置 -> 目标航点 的场地系误差(闭环核心) */
    float ex = nav.wp[nav.cur + 1].x_mm - odometer.x;
    float ey = nav.wp[nav.cur + 1].y_mm - odometer.y;
    float dist = sqrtf(ex * ex + ey * ey);

    /* 2. 到达判定: 进入半径换下一段; 末段进圈即停车 */
    if (dist < ARRIVE_RADIUS) {
        nav.cur++;
        if (nav.cur >= nav.count - 1) { Nav_Stop(); return; }  /* 到终点,停车 */
        Nav_LoadSegment();      /* 换段: 更新固定目标 + 清段里程 */
    }

     /* <<< 新增：S 型速度前馈 >>> */
    float v_plan = SCurve_GetVel(&nav.sc, HAL_GetTick());   // 标量速度 mm/s
    float v_x_ff = v_plan * nav.seg_dir_x;                  // 分解到车体系 x
    float v_y_ff = v_plan * nav.seg_dir_y;                  // 分解到车体系 y

     /* 3. 每周期用同一个段目标调位置环(measure = odometer.body_odom_x/y) */
    Chassis_Position_Control(nav.seg_tx, nav.seg_ty, v_x_ff, v_y_ff);   
}

void Nav_Stop(void)
{
    nav.running = 0;
    Chassis_Motor_Stop();
}

uint8_t Nav_IsDone(void)    { return !nav.running; }
uint8_t Nav_IsRunning(void) { return nav.running; }

