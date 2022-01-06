/*
 *    文件名称：	Motor_Control.c
 *    作者：		光电卓越
 *    版本：		V1.0
 *    说明：		电机控制相关函数实现文件
 *    创建日期：	2018-02-13
 *    修改日期：    2018-07-27
 *    修改者：
 *    修改目的:
 */


#include "include.h"
#include "Picture_Deal.h"
#include "Motor_Control.h"


/*内部函数声明*/

uint32 PID_Cal_Speed(uint32 Speed);


/*外部函数实现*/

/*
 *    函数名称：    Motor_Init();
 *    函数功能：	初始化电机
 *    入口参数： 	无
 *    出口参数： 	无
 *    作者： 		光电卓越
 *    创建日期：	2018-02-13
 *    版本：		V1.0
 */
void Motor_Init(void)
{
    ftm_pwm_init(Motor_FTM, FTM_CH0, Motor_Hz, 0);
    ftm_pwm_init(Motor_FTM, FTM_CH1, Motor_Hz, 0);
}

/*
 *    函数名称：    Stopcar();
 *    函数功能：	电机转速置零
 *    入口参数： 	无
 *    出口参数： 	无
 *    作者： 		光电卓越
 *    创建日期：	2018-02-13
 *    版本：		V1.0
 */
void Stopcar(void)
{
    ftm_pwm_duty(Motor_FTM, FTM_CH0, 0);
    ftm_pwm_duty(Motor_FTM, FTM_CH1, 0);
}

/*
 *    函数名称：    Speed_Control();
 *    函数功能：	电机转速控制
 *    入口参数： 	无
 *    出口参数： 	无
 *    作者： 		光电卓越
 *    创建日期：	2018-02-13
 *    版本：		V1.0
 *    修改者：
 *    修改记录：
 */
void Speed_Control(void)
{
    uint32 u32Control;

    u32Control=PID_Cal_Speed(g_u32Speed);

    if(u32Control<CONTROL_LOW_SPEED)
    {
        u32Control=CONTROL_LOW_SPEED;
    }
    if(u32Control>CONTROL_HIGH_SPEED)
    {
        u32Control=CONTROL_HIGH_SPEED;
    }
    
    ftm_pwm_duty(Motor_FTM, FTM_CH0, u32Control);
    ftm_pwm_duty(Motor_FTM, FTM_CH1, 0);
}


/*内部函数实现*/

/*
 *    函数名称：    PID_Cal_Speed();
 *    函数功能：	PID计算输出电机所需占空比
 *    入口参数： 	根据图像计算得到的速度
 *    出口参数： 	经PID计算后的输出输出速度
 *    作者： 		光电卓越
 *    创建日期：	2018-02-13
 *    版本：		V1.0
 *    修改者：
 *    修改记录：
 */
uint32 PID_Cal_Speed(uint32 Speed)
{
    return Speed;
}

