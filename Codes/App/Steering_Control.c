/*
 *    �ļ����ƣ�	Steering_Control.c
 *    ���ߣ�		���׿Խ
 *    �汾��		V1.0
 *    ˵����		���������غ���ʵ���ļ�
 *    �������ڣ�	2018-02-13
 *    �޸����ڣ�
 *    �޸��ߣ�
 *    �޸�Ŀ��:
 */


#include "include.h"
#include "math.h"
#include "Picture_Deal.h"
#include "Steering_Control.h"


/*�ڲ���������*/

float PID_Cal_Offset(float Offset);


/*�ⲿ����ʵ��*/

/*
 *    �������ƣ�    Steering_Init();
 *    �������ܣ�	�����ʼ��
 *    ��ڲ����� 	��
 *    ���ڲ����� 	��
 *    ���ߣ� 		���׿Խ
 *    �������ڣ�	2018-02-13
 *    �汾��		V1.0
 *    �޸��ߣ�
 *    �޸ļ�¼��
 */
void Steering_Init(void)
{
    ftm_pwm_init(S3010_FTM, S3010_CH,S3010_HZ, CONTROL_MID);
}

/*
 *    �������ƣ�    Steering_Control();
 *    �������ܣ�	�����ǿ���
 *    ��ڲ����� 	��
 *    ���ڲ����� 	��
 *    ���ߣ� 		���׿Խ
 *    �������ڣ�	2018-02-13
 *    �汾��		V1.0
 *    �޸��ߣ�
 *    �޸ļ�¼��
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


/*�ڲ�����ʵ��*/

/*
 *    �������ƣ�    PID_Cal_Offset();
 *    �������ܣ�	PID�������ƫ��
 *    ��ڲ����� 	����ͼ�����õ���ƫ��Offset
 *    ���ڲ����� 	��PID���������ƫ��
 *    ���ߣ� 		���׿Խ
 *    �������ڣ�	2018-02-13
 *    �汾��		V1.0
 *    �޸��ߣ�
 *    �޸ļ�¼��
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

