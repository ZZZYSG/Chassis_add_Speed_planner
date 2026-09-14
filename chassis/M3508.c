/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : M3508.c
  * @brief          : M3508 Motor Driver Implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "M3508.h"

//#define RPM_TO_MM_S(rpm)  ((rpm) * (WHEEL_CIRCUMFERENCE * REDUCE_RADIO) / 60.0f)  统一由转速计算误差

/* ==================== 配置PID初始化参数 ==================== */
/* 字段顺序: {kp, ki, kd, max_output, integral_limit, deadband} */
const PID_Param_t drive_param[4] = {
    {4, 0.5, 0, 16384, 5000, 3},   // 轮0
    {4, 0.5, 0, 16384, 5000, 3},   // 轮1
    {4, 0.5, 0, 16384, 5000, 3},   // 轮2
    {4, 0.5, 0, 16384, 5000, 3},   // 轮3
};
const PID_Param_t pos_param[3] = {
    {3, 0.001, 0, 300, 20, 1.2},   // x:   首次测试限幅 500 mm/s（2000 起步太猛，调稳后再加大）
    {3, 0.001, 0, 300, 20, 1.2},   // y:   同上
    {1.25, 0.025, 0, 75, 0.25, 0},   // yaw: 输出单位度/s，建议后续降到 60~120
};


// ================== 全局变量定义 ==================
// 1. 电机数据指针数组 (指向只读数据)
const motor_measure_t *motor_data[4]; 

// 2. 底盘4个电机的PID结构体
PID_TypeDef drive_motor_pid[4]; //每个轮子的速度环PID
PID_TypeDef pos_motor_pid[3];   //底盘位置环PID

// 3. 纠偏基准角度
float pos_hold_yaw = 0;         //初始化为0

// 4. yaw 角度展开状态（全局：旋转到位与直行纠偏共用，外部可 extern 观察）
float unwrap_yaw   = 0.0f;   // 展开后的连续角度
float tgt_unwrap   = 0.0f;   // 展开坐标系中的目标(锁存)
static float last_yaw   = 0.0f;   // 上一拍的原始角度
static uint8_t unwrap_ok = 0;     // 展开基准是否已建立
static float lock_cmd   = 0.0f;   // 目标对应的指令值

// 5. 梯形速度规划器的相关变量
static float smooth_vx = 0, smooth_vy = 0, smooth_w = 0;   /* 文件级,方便 Stop 清零 */
const float dt    = 0.001f;          /* TIM4 1kHz */
const float a_max = A_LINE * dt;    /* 2.0 mm/s/拍 */
const float w_max = W_LINE * dt;     /* 180°/s²,0.18°/s/拍 */

/*================== 私有接口函数 =====================*/

/* 每拍调用一次：把 -180~180 的陀螺仪角度展开成连续累计量 */
void Chassis_UpdateUnwrap(void)
{
    float delta;

    if (!unwrap_ok)
    {
        unwrap_yaw = final_yaw;
        last_yaw   = final_yaw;
        unwrap_ok  = 1;
    }
    else
    {
        delta = final_yaw - last_yaw;
        if (delta >  180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        unwrap_yaw += delta;
        last_yaw    = final_yaw;
    }
}


/*=================== 公共接口函数 =====================*/
/**
  * @brief 初始化底盘电机及PID参数
  */
void Chassis_Motor_Init(void)
{
    volatile int i;

    // 1. 获取电机反馈数据的指针
    motor_data[0] = get_chassis_motor_measure_point(0); 
    motor_data[1] = get_chassis_motor_measure_point(1); 
    motor_data[2] = get_chassis_motor_measure_point(2); 
    motor_data[3] = get_chassis_motor_measure_point(3); 

    // 2. 初始化PID参数（参数统一在 M3508.h 的 drive_param / pos_param 表中维护）
    for (i = 0; i < 4; i++)
    {
        pid_init(&drive_motor_pid[i]);

        drive_motor_pid[i].f_param_init(
            &drive_motor_pid[i],
            (uint16_t)drive_param[i].max_output,
            (uint16_t)drive_param[i].integral_limit,
            drive_param[i].kp,
            drive_param[i].ki,
            drive_param[i].kd,
            drive_param[i].deadband,
            0                // 初始目标值
        );
    }
    for (i = 0; i < 3; i++)
    {
        pid_init(&pos_motor_pid[i]);

        pos_motor_pid[i].f_param_init(
            &pos_motor_pid[i],
            (uint16_t)pos_param[i].max_output,
            (uint16_t)pos_param[i].integral_limit,
            pos_param[i].kp,
            pos_param[i].ki,
            pos_param[i].kd,
            pos_param[i].deadband,
            0                // 初始目标值
        );
    }
}



/**
  * @brief 底盘位置控制函数 
  车体以自身坐标系，向(x,y)平移,配合S型速度规划前馈，内置imu前馈以纠偏
 
  */
void Chassis_Position_Control(float body_x, float body_y, float hold_yaw)
{
   

    /* 1. 每拍先更新里程计与展开角（位置环和纠偏的反馈来源，
          里程计不更新的话 body_odom_x/y 恒为 0，位置环无法收敛） */
    Odometer_Update();
    Chassis_UpdateUnwrap();

    /* 2. 位置环：x/y 跟踪里程计，yaw 环跟踪锁存航向 */
    pos_motor_pid[0].target = body_x;
    pos_motor_pid[1].target = body_y;
    pos_motor_pid[2].target = hold_yaw;
    pos_motor_pid[0].f_cal_pid(&pos_motor_pid[0], odometer.body_odom_x);
    pos_motor_pid[1].f_cal_pid(&pos_motor_pid[1], odometer.body_odom_y);
    pos_motor_pid[2].f_cal_pid(&pos_motor_pid[2], unwrap_yaw);

    float v_x = pos_motor_pid[0].output;
    float v_y = pos_motor_pid[1].output;
    float w_correct = -pos_motor_pid[2].output;   // 负号与 Chassis_Move_yaw 调好的方向一致

    /* 3. 平动速度 + 纠偏旋转合成后一次下发 */
    Chassis_Speed_Control(v_x, v_y, w_correct);
    
}


//在车体系yaw方向旋转到固定角度(-180~180)
void Chassis_Move_yaw(float yaw)
{
    float delta;

    /* 1. 每拍先更新展开角 */
    Chassis_UpdateUnwrap();

    /* 2. 仅在新指令到来时，按最短路径在展开系里选定一次目标，之后锁存不动 */
    if (lock_cmd != yaw)
    {
        delta = yaw - unwrap_yaw;
        if (delta >  180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        tgt_unwrap = unwrap_yaw + delta;   // 如 +180 或 -180，方向唯一
        lock_cmd   = yaw;
        pos_motor_pid[2].iout = 0.0f;      // 新目标清积分
    }

    /* 3. 连续误差直接给 PID，全程无边界翻转 */
    pos_motor_pid[2].target = tgt_unwrap;
    pos_motor_pid[2].f_cal_pid(&pos_motor_pid[2], unwrap_yaw);
    Chassis_Speed_yaw(-pos_motor_pid[2].output);   // 取负号：与已调好的方向一致
    
}


/**
  * @brief 底盘速度统一出口：平移(vx,vy) 叠加
  * @param w 绕z轴角速度，rad/s（逆时针为正/顺时针为正按你原定义）
   范围大概在0-1700mm/s，不要填大
  */
void Chassis_Speed_Control(float v_x, float v_y, float w_dps)
{
    /* 注意参数顺序:Velocity_Smoother_Asym(目标, 当前, 每拍最大增量) */
    smooth_vx = Velocity_Smoother_Asym(v_x, smooth_vx, a_max);
    smooth_vy = Velocity_Smoother_Asym(v_y, smooth_vy, a_max);
    smooth_w  = Velocity_Smoother_Asym(w_dps, smooth_w, w_max);

 
    float rot = MM_S_TO_RPM((smooth_w * PI / 180.0f) * (a + b));  
    // 简单的全向移动解算示例---左为x正向，前为y正向，逆时针旋转为正方向
    drive_motor_pid[0].target = MM_S_TO_RPM( smooth_vx - smooth_vy ) + rot; // 左前轮
    drive_motor_pid[1].target = MM_S_TO_RPM( smooth_vx + smooth_vy ) + rot; // 右前轮
    drive_motor_pid[2].target = MM_S_TO_RPM(-smooth_vx - smooth_vy ) + rot; // 左后轮
    drive_motor_pid[3].target = MM_S_TO_RPM(-smooth_vx + smooth_vy ) + rot; // 右后轮

    // 计算PID输出并发送
     drive_motor_pid[0].f_cal_pid(&drive_motor_pid[0], motor_data[0]->speed_rpm);
     drive_motor_pid[1].f_cal_pid(&drive_motor_pid[1], motor_data[1]->speed_rpm);
     drive_motor_pid[2].f_cal_pid(&drive_motor_pid[2], motor_data[2]->speed_rpm);
     drive_motor_pid[3].f_cal_pid(&drive_motor_pid[3], motor_data[3]->speed_rpm);
        
    
    CAN_cmd_chassis(
        drive_motor_pid[0].output , 
        drive_motor_pid[1].output , 
        drive_motor_pid[2].output , 
        drive_motor_pid[3].output 
    );
}

//往逆时针方向以dps(度每秒)转动 低转速推荐:5-10,高转速推荐10-20
void Chassis_Speed_yaw(float dps){

    float w = dps * PI / 180.0f;

    drive_motor_pid[0].target = MM_S_TO_RPM( w * (a + b) ); // 左前轮
    drive_motor_pid[1].target = MM_S_TO_RPM( w * (a + b) ); // 右前轮
    drive_motor_pid[2].target = MM_S_TO_RPM( w * (a + b) ); // 左后轮
    drive_motor_pid[3].target = MM_S_TO_RPM( w * (a + b) ); // 右后轮
	
	drive_motor_pid[0].f_cal_pid(&drive_motor_pid[0], motor_data[0]->speed_rpm);
    drive_motor_pid[1].f_cal_pid(&drive_motor_pid[1], motor_data[1]->speed_rpm);
    drive_motor_pid[2].f_cal_pid(&drive_motor_pid[2], motor_data[2]->speed_rpm);
    drive_motor_pid[3].f_cal_pid(&drive_motor_pid[3], motor_data[3]->speed_rpm);
	
		 CAN_cmd_chassis(
        drive_motor_pid[0].output , 
        drive_motor_pid[1].output , 
        drive_motor_pid[2].output , 
        drive_motor_pid[3].output 
    );

}

//小车停止
void Chassis_Motor_Stop(void)    
{       
    volatile int i;
    for (i = 0; i < 4; i++) {
        drive_motor_pid[i].target = 0.0f;
        drive_motor_pid[i].iout   = 0.0f;
        drive_motor_pid[i].output = 0.0f;
        drive_motor_pid[i].pout   = 0.0f;
    }
    CAN_cmd_chassis(0, 0, 0, 0);   /* 直接发 0 电流,不做 PID 计算 */      
}

/* 锁存当前航向为直行纠偏基准(换段/开始直行时调用) */
void Chassis_ResetYawHold(void)
{
    Chassis_UpdateUnwrap();
    pos_hold_yaw = unwrap_yaw;
    pos_motor_pid[2].iout = 0.0f;
}



/*=================== 调试时调用进行测试 =====================*/

//在车体系x方向移动固定距离  
//注：使用前应调用Odometer_ResetSegment()函数将里程计临时清0
void Chassis_Move_x(float s)
{
	Chassis_UpdateUnwrap();
	Chassis_Position_Control( s, 0, unwrap_yaw);   // 直行并保持当前航向
}


//在车体系y方向移动固定距离
//注：使用前应调用Odometer_ResetSegment()函数将里程计临时清0
void Chassis_Move_y(float s)
{
	Chassis_UpdateUnwrap();
	Chassis_Position_Control( 0, s, unwrap_yaw);
}


//往x方向(>0为正方向，<0为负方向)以v速移动
void Chassis_Speed_x(float v)  
{
    Chassis_Speed_Control(v, 0.0f, 0.0f);
}   


//往y方向(>0为正方向，<0为负方向)以v速移动
void Chassis_Speed_y(float v)
{
    Chassis_Speed_Control(0.0f, v, 0.0f);              
}
