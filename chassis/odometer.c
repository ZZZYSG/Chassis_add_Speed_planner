#include "odometer.h"


#ifndef PI
#define PI  3.1415926f
#endif

/* M3508 + 19:1 减速箱: 编码器在电机轴上, 8192 计数 = 电机 1 圈 = 车轮 1/19 圈
 * 每计数对应车轮位移(mm) = 轮周长 / 8192 / 减速比 */
#define WHEEL_RADIUS_MM     37.5f
#define WHEEL_CIRCUMFERENCE (2.0f * PI * WHEEL_RADIUS_MM)                          /* ≈ 235.62 mm */
#define REDUCTION_RATIO     19.0f                                                  /* 与 M3508.h 的 REDUCE_RADIO 一致 */
#define MM_PER_ECD          (WHEEL_CIRCUMFERENCE / 8192.0f / REDUCTION_RATIO)      /* ≈ 0.00153 mm/计数 */

Odometer odometer;

/* 里程计侧的原始角度快照 */
static uint16_t wheel_last_ecd[4];
static uint8_t  odom_first_run = 1;



    /* 清零时车必须停在已知的场地坐标点上(即 A* 起点的世界坐标),
     * 车头朝向与场地系 +X 的关系要和 final_yaw 的零点定义一致 */
void Odometer_Init(void)
{
    odometer.x = 0.0f;
    odometer.y = 0.0f;
    odometer.theta = 0.0f;
    odom_first_run = 1;   /* 下次 Update 时重新记录角度参考点 */

}
void Odometer_ResetSegment(void)
{
    odometer.body_odom_x = 0.0f;
    odometer.body_odom_y = 0.0f;
}
   /* 调用周期建议 ≤ 50ms(全速下半圈约 64ms, 跨圈修正才有效) */
void Odometer_Update(void)
{
    int   i;
    float d[4];
    for (i = 0; i < 4; i++)
    {
        if (odom_first_run || motor_data[i] == NULL) 
        {
            //首次执行，数据指针有效
            if (motor_data[i] != NULL)
            {
                wheel_last_ecd[i] = motor_data[i]->ecd;   /* 首次只记参考点 */
            }
            //未执行，数据指针无效
            d[i] = 0.0f;
        }
        else
        {
            int16_t delta = (int16_t)(motor_data[i]->ecd - wheel_last_ecd[i]);
            // 4096是指单个测量周期中最大转过角度，通过采样频率估算
            if (delta >  4096) delta -= 8192;   /* 8191->0 反向跨圈 */
            if (delta < -4096) delta += 8192;   /* 0->8191 正向跨圈 */
            wheel_last_ecd[i] = motor_data[i]->ecd;
            d[i] = (float)delta * MM_PER_ECD;   /* 本次调用区间的位移 */
        }
    }
    odom_first_run = 0;

    /* 2. 正解算(车体系位移增量, 单位 mm)
         轮序与 Chassis_Speed_Control 的逆解算对应:
           dx = (-m0 - m1 + m2 + m3) / 4
           dy = (-m0 + m1 - m2 + m3) / 4*/
    float body_dx = ( d[0] + d[1] - d[2] - d[3]) * 0.25f;
    float body_dy = (-d[0] + d[1] - d[2] + d[3]) * 0.25f;
    odometer.body_odom_x += body_dx;
    odometer.body_odom_y += body_dy;

    /* 3. 场地系航向角(度 -> 弧度) 
    odometer.theta = final_yaw * PI / 180.0f;

     4. 车体系 -> 场地系旋转并累加位置 
    float cos_t = cosf(odometer.theta);
    float sin_t = sinf(odometer.theta);
    odometer.x += body_dx * cos_t - body_dy * sin_t;
    odometer.y += body_dx * sin_t + body_dy * cos_t; */
}
