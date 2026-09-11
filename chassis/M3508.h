#ifndef __M3508_H
#define __M3508_H

/* ==================== 头文件包含 ==================== */
#include "stm32f4xx_hal.h"
#include "pid.h"            // 需要包含 pid.h         以识别 PID_TypeDef
#include "CAN_receive.h"    // 需要包含 CAN_receive.h 以识别 motor_measure_t
#include "bsp_can.h"        // 需要包含 bsp_can.h     以使用 CAN 功能
#include "imu.h"            // 需要包含 imu.h         以用于读取场地系角度
#include "odometer.h"       // 需要包含 odometer.h    以识别 Odometer 类型


// ============ 机械参数(两组底盘)定义 ============
#define WHEEL_RADIUS_MM      37.5f    // 轮子半径 (mm)  37.5 
#define PI                   3.1415926f
#define a                    125      //半轴距 125  107
#define b                    102      //半轮距 102  107
#define REDUCE_RADIO         19       //减速比
// 轮子周长 (mm)
#define WHEEL_CIRCUMFERENCE  (2.0f * PI * WHEEL_RADIUS_MM)  // ≈ 235.5 mm

// 线速度(mm/s) ↔ 轮转速(rpm) 互转
// M3508 输出轴直连轮子
#define MM_S_TO_RPM(v)    ((v) * 60.0f / WHEEL_CIRCUMFERENCE * REDUCE_RADIO)


/* ==================== 外部变量声明 ==================== */
extern float final_yaw;                       // 场地系角度（度，-180~180，由 imu 更新）
extern const motor_measure_t *motor_data[4];  // 4 个底盘电机反馈（只读）
extern PID_TypeDef drive_motor_pid[4];        // 轮速度环 PID
extern PID_TypeDef pos_motor_pid[3];          // 位置环 PID (x / y / yaw)
extern Odometer odometer;
extern float unwrap_yaw;                      // yaw 角度展开后的连续值（调试观察）
extern float tgt_unwrap;                      // 展开坐标系中的 yaw 目标（调试观察）


/* ==================== 函数声明 ==================== */

/* ---------- 初始化 ---------- */
void Chassis_Motor_Init(void);

/* ---------- 实际用的函数 ---------- */
void Chassis_UpdateUnwrap(void);                                            // 每拍调用：更新 yaw 展开角(展开+锁存)
void Chassis_Position_Control(float body_x, float body_y, float hold_yaw);  // 车体以自身坐标系向(x,y)移动,车的航向角为hold_yaw度,内部集成IMU纠偏
void Chassis_Move_yaw(float yaw);                                           // 车体系yaw方向旋转到固定角度
void Chassis_Speed_Control(float v_x, float v_y, float w);                  // 车体中心以(vx,vy)平移
void Chassis_Speed_yaw(float dps);                                          // 绕z轴旋转，参数单位：度/秒
void Chassis_Motor_Stop(void);                                              // 停止
void Chassis_ResetYawHold(void);                                            // <<< 新增：换段/新目标时重置 IMU 锁存

/* ---------- 调试时调用进行测试 ---------- */
void Chassis_Move_x(float x);   // 车体系x方向移动固定距离
void Chassis_Move_y(float y);   // 车体系y方向移动固定距离
void Chassis_Speed_x(float v);  // 车体系x方向以v速移动
void Chassis_Speed_y(float v);  // 车体系y方向以v速移动

#endif
