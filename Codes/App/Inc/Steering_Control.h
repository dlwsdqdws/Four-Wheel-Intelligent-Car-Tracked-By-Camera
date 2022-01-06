/*
 *    �ļ����ƣ�	Steering_Control.h
 *    ���ߣ�		���׿Խ
 *    �汾��		V1.0
 *    ˵����		�������ͷ�ļ�
 *    �������ڣ�	2018-02-13
 *    �޸����ڣ�
 *    �޸��ߣ�
 *    �޸�Ŀ��:
 */


#ifndef _STEERING_CONTROL_H_
#define _STEERING_CONTROL_H_


/*�궨��*/

#define S3010_FTM    FTM0
#define S3010_CH	FTM_CH0//�����ʼ��PTC1
#define S3010_HZ    100//����Ƶ��200Hz����Ӧ����5ms 
#define CONTROL_MID  1340//�����ֵ��Ӧռ�ձ����� 
#define CONTROL_LEFT_MAX  940//�������������޶�Ӧռ�ձ����� 
#define CONTROL_RIGHT_MAX 1740//�����������Ҽ��޶�Ӧռ�ձ�����


/*�ⲿ��������*/

extern void Steering_Init(void);//�����ʼ��
extern void Steering_Control(void);//�����ǿ���


#endif

