/*
 *    �ļ����ƣ�	Motor_Control.h
 *    ���ߣ�		���׿Խ
 *    �汾��		V1.0
 *    ˵����		�������ͷ�ļ�
 *    �������ڣ�	2018-02-13
 *    �޸����ڣ�
 *    �޸��ߣ�
 *    �޸�Ŀ��:
 */


#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_


/*�궨��*/

#define Motor_FTM FTM1
#define Motor_Hz 20000
#define CONTROL_HIGH_SPEED  500//����ʱ����������ռ�ձ�70%
#define CONTROL_LOW_SPEED   100//����ʱ���������Сռ�ձ�10%


/*�ⲿ��������*/

extern void Motor_Init(void);//�����ʼ��
extern void Stopcar(void);//���ת�����㼴����ͣ��
extern void Speed_Control(void);//����ٶȿ���


#endif

