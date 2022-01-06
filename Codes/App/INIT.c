/*
 *    �ļ����ƣ�	INIT.c
 *    ���ߣ�		���׿Խ
 *    �汾��		V1.0
 *    ˵����		��ʼ��ʵ���ļ�����ʼ������ģ��
 *    �������ڣ�	2018-02-13
 *    �޸����ڣ�
 *    �޸��ߣ�
 *    �޸�Ŀ��:
 */


#include "INIT.h"


/*�ڲ���������*/

void PORTA_IRQHandler(void);
void DMA0_IRQHandler(void);


/*�ⲿ����ʵ��*/

/*
 *    �������ƣ�    Init_All();
 *    �������ܣ�	��ʼ������ģ��
 *    ��ڲ����� 	��
 *    ���ڲ����� 	��
 *    ���ߣ� 		���׿Խ
 *    �������ڣ�	2018-02-13
 *    �汾��		V1.0
 *    �޸��ߣ�
 *    �޸ļ�¼��
 */
void Init_All(void)
{
    DisableInterrupts;//�����ж�
    
    Steering_Init();//�����ʼ��

    Motor_Init();//�����ʼ��

    camera_init(ImgBuffer);//����ͷ��ʼ��
    
   // key_init(KEY_MAX);
    
   OLED_Init();//OLED��ʼ��
    
    
    //�����жϷ��������ж�������
    set_vector_handler(PORTA_VECTORn, PORTA_IRQHandler);
    set_vector_handler(DMA0_VECTORn, DMA0_IRQHandler);

    //�趨�ж����ȼ���ȷ������ͷ�ɼ������ȼ����
    NVIC_SetPriority(PORTA_IRQn, 0);
    NVIC_SetPriority(DMA0_IRQn, 1);

    EnableInterrupts;//�����ж�
}


/*�ڲ�����ʵ��*/

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler(void)
{
    camera_dma();
}


/*!
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler(void)
{
    uint8  n;    //���ź�
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if CAMERA_USE_HREF                                      //��ʹ�����ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif
}

