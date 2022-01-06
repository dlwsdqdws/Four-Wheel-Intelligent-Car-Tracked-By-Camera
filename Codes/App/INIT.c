/*
 *    文件名称：	INIT.c
 *    作者：		光电卓越
 *    版本：		V1.0
 *    说明：		初始化实现文件，初始化各个模块
 *    创建日期：	2018-02-13
 *    修改日期：
 *    修改者：
 *    修改目的:
 */


#include "INIT.h"


/*内部函数声明*/

void PORTA_IRQHandler(void);
void DMA0_IRQHandler(void);


/*外部函数实现*/

/*
 *    函数名称：    Init_All();
 *    函数功能：	初始化各个模块
 *    入口参数： 	无
 *    出口参数： 	无
 *    作者： 		光电卓越
 *    创建日期：	2018-02-13
 *    版本：		V1.0
 *    修改者：
 *    修改记录：
 */
void Init_All(void)
{
    DisableInterrupts;//关总中断
    
    Steering_Init();//舵机初始化

    Motor_Init();//电机初始化

    camera_init(ImgBuffer);//摄像头初始化
    
   // key_init(KEY_MAX);
    
   OLED_Init();//OLED初始化
    
    
    //配置中断服务函数到中断向量表
    set_vector_handler(PORTA_VECTORn, PORTA_IRQHandler);
    set_vector_handler(DMA0_VECTORn, DMA0_IRQHandler);

    //设定中断优先级，确保摄像头采集的优先级最高
    NVIC_SetPriority(PORTA_IRQn, 0);
    NVIC_SetPriority(DMA0_IRQn, 1);

    EnableInterrupts;//开总中断
}


/*内部函数实现*/

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler(void)
{
    camera_dma();
}


/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler(void)
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if CAMERA_USE_HREF                                      //不使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}

