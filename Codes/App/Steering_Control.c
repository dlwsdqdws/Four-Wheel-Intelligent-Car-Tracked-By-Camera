/*
 *    文件名称：	Steering_Control.c
 *    作者：		光电卓越
 *    版本：		V1.0
 *    说明：		舵机控制相关函数实现文件
 *    创建日期：	2018-02-13
 *    修改日期：
 *    修改者：
 *    修改目的:
 */


#include "include.h"
#include "math.h"
#include "Picture_Deal.h"
#include "Steering_Control.h"


/*内部函数声明*/

float PID_Cal_Offset(float Offset);


/*外部函数实现*/

/*
 *    函数名称：    Steering_Init();
 *    函数功能：	舵机初始化
 *    入口参数： 	无
 *    出口参数： 	无
 *    作者： 		光电卓越
 *    创建日期：	2018-02-13
 *    版本：		V1.0
 *    修改者：
 *    修改记录：
 */
void Steering_Init(void)
{
    ftm_pwm_init(S3010_FTM, S3010_CH,S3010_HZ, CONTROL_MID);
}

/*
 *    函数名称：    Steering_Control();
 *    函数功能：	舵机打角控制
 *    入口参数： 	无
 *    出口参数： 	无
 *    作者： 		光电卓越
 *    创建日期：	2018-02-13
 *    版本：		V1.0
 *    修改者：
 *    修改记录：
 */
void Steering_Control(void)
{
    float control;

    control = CONTROL_MID + PID_Cal_Offset(g_f_Offset);

    if(control<CONTROL_LEFT_MAX)
    {
        control=CONTROL_LEFT_MAX;
    }
    if(control>CONTROL_RIGHT_MAX)
    {
        control=CONTROL_RIGHT_MAX;
    }
    
    ftm_pwm_duty(S3010_FTM, S3010_CH, (uint32)control);
}


/*内部函数实现*/

/*
 *    函数名称：    PID_Cal_Offset();
 *    函数功能：	PID计算输出偏差
 *    入口参数： 	根据图像计算得到的偏差Offset
 *    出口参数： 	经PID计算后的输出偏差
 *    作者： 		光电卓越
 *    创建日期：	2018-02-13
 *    版本：		V1.0
 *    修改者：
 *    修改记录：
 */
float PID_Cal_Offset(float Offset)
{
    static float s_f_LastOffset=0.0f;

    float f_T=0.1f;
    float f_Output=0.0f;

    f_Output = g_f_Kp*Offset + (g_f_Td/f_T)*(Offset-s_f_LastOffset);

    s_f_LastOffset=Offset;

    return f_Output;
}

