/*
 *    �ļ����ƣ�	main.c
 *    ���ߣ�		���׿Խ
 *    �汾��		V1.0
 *    ˵����		������ʵ���ļ�
 *    �������ڣ�	2018-02-13
 *    �޸����ڣ�
 *    �޸��ߣ�
 *    �޸�Ŀ��:
 */


#include "main.h"


/*
 *    �������ƣ�    main();
 *    �������ܣ�	������
 *    ��ڲ����� 	��
 *    ���ڲ����� 	��
 *    ���ߣ� 		���׿Խ
 *    �������ڣ�	2018-02-13
 *    �汾��		V1.0
 *    �޸��ߣ�
 *    �޸ļ�¼��
 */
void main(void)
{
    Init_All();//��ʼ����ģ��

    while(1)
    {
        Picture_Deal();//ͼ��Ĳɼ�����
        
        Steering_Control();//�����ǿ���
         
        Speed_Control();//����ٶȿ���,�ٶ�Ϊ0
    }
}
