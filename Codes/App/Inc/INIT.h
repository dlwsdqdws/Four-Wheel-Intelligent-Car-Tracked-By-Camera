/*
 *    文件名称：	INIT.h
 *    作者：		光电卓越
 *    版本：		V1.0
 *    说明：		初始化头文件
 *    创建日期：	2018-02-13
 *    修改日期：
 *    修改者：
 *    修改目的:
 */


#ifndef _INIT_H_
#define _INIT_H_


#include "include.h"
#include "Motor_Control.h"
#include "Steering_Control.h"
#include "Picture_Deal.h"
#include "OLED.h"
#include "stdbool.h"//包含布尔变量宏定义的头文件 


/*外部函数声明*/

extern void Init_All(void);//初始化各模块，在主函数中最先调用


#endif

