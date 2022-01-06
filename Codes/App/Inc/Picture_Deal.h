

/*
 *	  �ļ����ƣ�	Picture_Deal.h
 *	  ���ߣ�		���׿Խ
 *	  �汾��		V1.0
 *	  ˵����		ͼ����ͷ�ļ�
 *	  �������ڣ�	2018-02-13
 *	  �޸����ڣ�	2018-04-04
 *	  �޸��ߣ�		Ф����
 *	  �޸�Ŀ��:		��ӱ����ͺ궨������
 */
#ifndef _PICTURE_DEAL_H_
#define _PICTURE_DEAL_H_
#include "stdbool.h" //�������������궨���ͷ�ļ�

/*�궨��*/
#define IMG_BLACK 0              //ͼ���Ӧ��Ϊ��ʱ��ֵ
#define IMG_WHITE 255            //ͼ���Ӧ��Ϊ��ʱ��ֵ
#define FARLINE 10               //ʹ�õ���Զ��
#define LINE_MAX (CAMERA_H - 1)  //�����±����ֵ
#define INDEX_MAX (CAMERA_W - 1) //�����±����ֵ
#define CENTER_POINT 39             

#define TURNDECIDE 20     
#define STRAIGHTDECIDE 12 

/*�ⲿ��������*/
extern float g_f_Offset;  //����ͼ������ƫ��
extern float g_f_Kp;      //���PID���Ʊ���ϵ��
extern float g_f_Td;      //PID����΢��ϵ��
extern uint32 g_u32Speed; //����ͼ�������ٶ�
extern bool g_b_OLEDSHOW;

//�洢ͼ��������Ϣ�Ķ�ά���飬һ���ֽڶ�Ӧ1������
extern uint8 Img[CAMERA_H][CAMERA_W];

//�洢ͼ��������Ϣ��һά���飬һ���ֽڶ�Ӧ8������
extern uint8 ImgBuffer[CAMERA_SIZE];

/*�ⲿ��������*/
extern void Picture_Deal(void); //ͼ��ɼ��ʹ���
extern void OLED_Drawimg();     //дӥ������ͷ����
extern void Stopcar(void);      //���ת�����㼴����ͣ��

#endif
