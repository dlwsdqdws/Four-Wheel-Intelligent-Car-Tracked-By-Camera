/*
 *    �ļ����ƣ�	Motor_Control.c
 *    ���ߣ�		���׿Խ
 *    �汾��		V1.0
 *    ˵����		���������غ���ʵ���ļ�
 *    �������ڣ�	2018-02-13
 *    �޸����ڣ�    2018-07-27
 *    �޸��ߣ�
 *    �޸�Ŀ��:
 */


#include "include.h"
#include "Picture_Deal.h"
#include "Motor_Control.h"


/*�ڲ���������*/

uint32 PID_Cal_Speed(uint32 Speed);


/*�ⲿ����ʵ��*/

/*
 *    �������ƣ�    Motor_Init();
 *    �������ܣ�	��ʼ�����
 *    ��ڲ����� 	��
 *    ���ڲ����� 	��
 *    ���ߣ� 		���׿Խ
 *    �������ڣ�	2018-02-13
 *    �汾��		V1.0
 */
void Motor_Init(void)
{
    ftm_pwm_init(Motor_FTM, FTM_CH0, Motor_Hz, 0);
    ftm_pwm_init(Motor_FTM, FTM_CH1, Motor_Hz, 0);
}

/*
 *    �������ƣ�    Stopcar();
 *    �������ܣ�	���ת������
 *    ��ڲ����� 	��
 *    ���ڲ����� 	��
 *    ���ߣ� 		���׿Խ
 *    �������ڣ�	2018-02-13
 *    �汾��		V1.0
 */
void Stopcar(void)
{
    ftm_pwm_duty(Motor_FTM, FTM_CH0, 0);
    ftm_pwm_duty(Motor_FTM, FTM_CH1, 0);
}

/*
 *    �������ƣ�    Speed_Control();
 *    �������ܣ�	���ת�ٿ���
 *    ��ڲ����� 	��
 *    ���ڲ����� 	��
 *    ���ߣ� 		���׿Խ
 *    �������ڣ�	2018-02-13
 *    �汾��		V1.0
 *    �޸��ߣ�
 *    �޸ļ�¼��
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


/*�ڲ�����ʵ��*/

/*
 *    �������ƣ�    PID_Cal_Speed();
 *    �������ܣ�	PID��������������ռ�ձ�
 *    ��ڲ����� 	����ͼ�����õ����ٶ�
 *    ���ڲ����� 	��PID�������������ٶ�
 *    ���ߣ� 		���׿Խ
 *    �������ڣ�	2018-02-13
 *    �汾��		V1.0
 *    �޸��ߣ�
 *    �޸ļ�¼��
 */
uint32 PID_Cal_Speed(uint32 Speed)
{
    return Speed;
}

