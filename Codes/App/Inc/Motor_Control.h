/*
 *    文件名称：	Motor_Control.h
 *    作者：		光电卓越
 *    版本：		V1.0
 *    说明：		电机控制头文件
 *    创建日期：	2018-02-13
 *    修改日期：
 *    修改者：
 *    修改目的:
 */


#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_


/*宏定义*/

#define Motor_FTM FTM1
#define Motor_Hz 20000
#define CONTROL_HIGH_SPEED  500//控制时给电机的最大占空比70%
#define CONTROL_LOW_SPEED   100//控制时给电机的最小占空比10%


/*外部函数声明*/

extern void Motor_Init(void);//电机初始化
extern void Stopcar(void);//电机转速置零即控制停车
extern void Speed_Control(void);//电机速度控制


#endif

