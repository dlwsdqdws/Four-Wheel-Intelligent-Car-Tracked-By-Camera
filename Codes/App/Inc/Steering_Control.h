/*
 *    文件名称：	Steering_Control.h
 *    作者：		光电卓越
 *    版本：		V1.0
 *    说明：		舵机控制头文件
 *    创建日期：	2018-02-13
 *    修改日期：
 *    修改者：
 *    修改目的:
 */


#ifndef _STEERING_CONTROL_H_
#define _STEERING_CONTROL_H_


/*宏定义*/

#define S3010_FTM    FTM0
#define S3010_CH	FTM_CH0//舵机初始化PTC1
#define S3010_HZ    100//工作频率200Hz，对应周期5ms 
#define CONTROL_MID  1340//舵机中值对应占空比输入 
#define CONTROL_LEFT_MAX  940//允许给舵机的左极限对应占空比输入 
#define CONTROL_RIGHT_MAX 1740//允许给舵机的右极限对应占空比输入


/*外部函数声明*/

extern void Steering_Init(void);//舵机初始化
extern void Steering_Control(void);//舵机打角控制


#endif

