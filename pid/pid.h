#ifndef PID_H
#define PID_H

#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#include "math.h"

#define NULL 0
///////////////////////////////////////////////
typedef struct {
    float kp, ki, kd;
    float max_output, integral_limit, deadband;
} PID_Param_t;

typedef struct _PID_TypeDef
{
  //PID配置参数
	float target;				    //目标值
	float kp,ki,kd;					//pid参数
    float MaxOutput;				//输出限幅
	float IntegralLimit;		    //积分限幅
	float DeadBand;			        //死区（绝对值）
	
  //PID运行状态
	float   measure;					//测量值
	float   err;							//误差
	float   last_err;      		//上次误差
	float   pout;     
    float   iout;      
    float   dout;
	float   output;						//本次输出
	float   last_output;			//上次输出	
	
	void (*f_param_init)(
				   struct _PID_TypeDef *pid,  //PID各个参数配置
				   uint16_t  maxOutput,
				   uint16_t integralLimit,
				   float kp,
				   float ki,
				   float kd,
				   float    deadband,  
				   int16_t  tainitrget
		);
				   
   void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid三个参数修改
   float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);   //输入速度实测值，输出控制电流值
}PID_TypeDef;

void pid_init(PID_TypeDef* pid);
	

///////////////////////////////////////////////


#endif
