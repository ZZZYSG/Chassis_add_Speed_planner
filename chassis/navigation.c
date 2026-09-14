#include "navigation.h"

#define PI  3.1415926f

/* 导航层全局状态(仅本文件可见),所有函数共享这一个实例 */
extern float pos_hold_yaw;
extern uint8_t route_step_mark;
extern uint8_t task_step_mark;
Navigation_t nav;
/* ==================== 3×3 拓扑节点 -> 世界坐标 (mm) ====================
 * 表编号 1~9 = 代码索引 0~8(排序规则: 先比 Y 小, 再比 X 小):
 *   编号 |  X   |  Y  | (X, Y)
 *    1   | 275  | 350  | (275,  350)
 *    2   | 1200 | 350  | (1200, 350)
 *    3   | 2050 | 350  | (2050, 350)
 *    4   | 275  | 1200 | (275, 1200)
 *    5   | 1200 | 1200 | (1200,1200)
 *    6   | 2050 | 1200 | (2050,1200)
 *    7   | 275  | 2080 | (275, 2080)
 *    8   | 1200 | 2080 | (1200,2080)
 *    9   | 2050 | 2080 | (2050,2080)
 * 格距不相等(X: 850/880, Y: 925/850),所以查表,不用统一格长
 */
static const float NODE_X[NODES] = {
    275.0f, 1200.0f, 2050.0f,     /* 第 0 行 */
    275.0f, 1200.0f, 2050.0f,     /* 第 1 行 */
    275.0f, 1200.0f, 2050.0f      /* 第 2 行 */
};
static const float NODE_Y[NODES] = {
    350.0f,  350.0f,  350.0f,     /* 第 0 行 */
    1200.0f, 1200.0f, 1200.0f,    /* 第 1 行 */
    2080.0f, 2080.0f, 2080.0f     /* 第 2 行 */
};


/* 换段: 把本段场地系误差转成段起点车体系相对位移, 并清段里程,航线角锁定在0，90，180(-180)，根据实际情况确认 */
static void Nav_LoadSegment(void)
{
    float ex = nav.wp[nav.cur + 1].x_mm - odometer.x;
    float ey = nav.wp[nav.cur + 1].y_mm - odometer.y;

    float yaw_rad = final_yaw * PI / 180.0f;
    float cos_y = cosf(yaw_rad);
    float sin_y = sinf(yaw_rad);

    /* R(-yaw)·(ex,ey), yaw 正 = 右转(顺时针), 与 odometer.c 约定一致 */
    nav.seg_tx =  ex * cos_y - ey * sin_y;
    nav.seg_ty =  ex * sin_y + ey * cos_y;

    /* 标定旋转方向 */
    int8_t dy = (int8_t)(nav.wp[nav.cur+1].idx / 3) - (int8_t)(nav.wp[nav.cur].idx / 3);
    int8_t dx = (int8_t)(nav.wp[nav.cur+1].idx % 3) - (int8_t)(nav.wp[nav.cur].idx % 3);
    if      (dy > 0)  nav.correct_yaw = 0.0f;    /* 朝 +Y */
    else if (dy < 0)  nav.correct_yaw = 180.0f;  /* 朝 -Y */
    else if (dx > 0)  nav.correct_yaw = 90.0f;   /* 朝 +X (符号实车验证) */
    else              nav.correct_yaw = -90.0f;  /* 朝 -X */
    nav.turning = 1;

    /* 航向目标换算到展开坐标系(与底层 yaw 环的测量 unwrap_yaw 对齐) */
    Chassis_UpdateUnwrap();                    // 先刷新当前展开角
    float dyaw = nav.correct_yaw - final_yaw;  // 目标与当前的绝对角度差
    //边界保护
    if (dyaw > 180) dyaw -= 360;
    if (dyaw < -180) dyaw += 360;
    pos_hold_yaw = unwrap_yaw + dyaw;          // 换算到展开角度差

    /* 换段时清 yaw 环积分，避免上段残留扭矩干扰新航向 */
    pos_motor_pid[2].iout = 0.0f;

    Odometer_ResetSegment();   /* body_odom_x/y 归零, 与 seg_tx/ty 配套 */
}


void Nav_Init(void)
{
    memset(&nav, 0, sizeof(nav));
}

/* 根据出发航向判断从哪个启停区出发:
 * ≈INIT_YAW1(0°)    -> 从节点 0 进场
 * ≈INIT_YAW2(180°)  -> 从节点 6 进场 */
uint8_t Nav_DetectStartNode(void)
{
    float yaw = JYIMU_GetYaw();
    float d1 = yaw - INIT_YAW1;
    float d2 = yaw - INIT_YAW2;

    d1 += (d1 > 180.0f) ? -360.0f : (d1 < -180.0f) ? 360.0f : 0.0f;
    d2 += (d2 > 180.0f) ? -360.0f : (d2 < -180.0f) ? 360.0f : 0.0f;

    if (fabsf(d1) <= YAW_MATCH_TOL) return 6;
    if (fabsf(d2) <= YAW_MATCH_TOL) return 0;
    return 0xFF;
}

uint8_t Nav_Start_From_Origin(uint8_t first_node)
{
    static uint8_t odo_inited = 0;
    if (first_node >= NODES) return 0;
    else if (first_node == 0xFF) return 0;
    else if (first_node == 0 && !odo_inited)
    {
        odometer.x = 125.0f;  odometer.y = 102.0f; 
        odo_inited = 1;
    }
    else if (first_node == 6 && !odo_inited)
    {
        odometer.x = 125.0f; odometer.y = 2298.0f; 
        odo_inited = 1;
    }

    if(first_node == 0)
    Chassis_Position_Control(NODE_X[first_node]-odometer.x, NODE_Y[first_node] - odometer.y, pos_hold_yaw);
    else if(first_node == 6)
    Chassis_Position_Control(odometer.x - NODE_X[first_node], odometer.y - NODE_Y[first_node], pos_hold_yaw);

    float ox = NODE_X[first_node] - odometer.x;
    float oy = NODE_Y[first_node] - odometer.y;
    if (sqrtf(ox * ox + oy * oy) < 10.0f) 
    {
        Chassis_Motor_Stop();
        return 1;
    }
    else
    {
        return 0;
    }
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
    for (volatile uint8_t i = 0; i < len && i < MAX_WAYPOINTS; i++) {
        nav.wp[i].idx  = path[i];
        nav.wp[i].x_mm = NODE_X[path[i]];
        nav.wp[i].y_mm = NODE_Y[path[i]];
    }
    nav.count   = len;
    nav.cur     = 0;
    /* 把里程计场地系位置对齐到起点节点的世界坐标 */
    odometer.x = NODE_X[start];
    odometer.y = NODE_Y[start];

    /* 计算换段时,车体在x,y方向移动的位移,以及锁定的航向 */
    Nav_LoadSegment();
    nav.running = 1;
    return 1;
}




/* 每个控制周期调用一次,内部直接读里程计 odometer 和 IMU 的 yaw_final */
void Nav_Update(void)
{
    if (!nav.running) return;   //还没有导航信息
                            
    if (nav.cur >= nav.count - 1) { Nav_Stop(); return; }   //导航完了
        float ex = nav.wp[nav.cur + 1].x_mm - odometer.x;
        float ey = nav.wp[nav.cur + 1].y_mm - odometer.y;
    //平移段
    if (!nav.turning){
        /* 1. 当前位置 -> 目标航点 的场地系误差(闭环核心) */
        float dist = sqrtf(ex * ex + ey * ey);

        /* 2. 到达判定: 进入半径换下一段; 末段进圈即停车 */
        if (dist < ARRIVE_RADIUS) {
            nav.cur++;
            if (nav.cur >= nav.count - 1) { Nav_Stop(); return; }  /* 到终点,停车 */
            Nav_LoadSegment();                   /* 换段: 更新固定目标 + 清段里程 */
            return;   /* 换段拍直接结束: 下一拍 ex/ey 才是新段误差,
                         否则本拍用旧航点误差(≈0)重算会把新段目标覆盖成 0 */
        }

    /* 3. 每周期用同一个段目标调位置环(measure = odometer.body_odom_x/y),
          航向锁定 pos_hold_yaw(已换算到展开系),内部含 IMU 纠偏 */
    Chassis_Position_Control(nav.seg_tx, nav.seg_ty, pos_hold_yaw);  
    }

    /* 转向段: 每拍直接驱动 yaw 环, 原地转, 不发送平动 */
    if (nav.turning) {

        if (fabsf(unwrap_yaw - pos_hold_yaw) < 1.0f) {     /* 转到位(3°容差) */
            nav.turning = 0;
            /* 用转向后的新航向, 把场地系误差换算成车体系目标(yaw 正 = 右转) */
            float yr = final_yaw * PI / 180.0f;
            nav.seg_tx =  ex * cosf(yr) - ey * sinf(yr);
            nav.seg_ty =  ex * sinf(yr) + ey * cosf(yr);
            Odometer_ResetSegment();   /* 清掉转向期间滑移混入的里程, 让平移段从0计 */
            pos_motor_pid[0].iout = 0.0f;   /* 清平动积分 */
            pos_motor_pid[1].iout = 0.0f;
           }
    Chassis_Position_Control(odometer.body_odom_x, odometer.body_odom_y, pos_hold_yaw);
    }

 
}

void Nav_Stop(void)
{
    nav.running = 0;
    Chassis_Motor_Stop();
}

uint8_t Nav_State(void)   { return nav.running; }
