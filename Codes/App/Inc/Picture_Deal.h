

/*
 *	  文件名称：	Picture_Deal.h
 *	  作者：		光电卓越
 *	  版本：		V1.0
 *	  说明：		图像处理头文件
 *	  创建日期：	2018-02-13
 *	  修改日期：	2018-04-04
 *	  修改者：		肖无棱
 *	  修改目的:		添加变量和宏定义声明
 */
#ifndef _PICTURE_DEAL_H_
#define _PICTURE_DEAL_H_
#include "stdbool.h" //包含布尔变量宏定义的头文件

/*宏定义*/
#define IMG_BLACK 0              //图像对应点为黑时的值
#define IMG_WHITE 255            //图像对应点为白时的值
#define FARLINE 10               //使用的最远行
#define LINE_MAX (CAMERA_H - 1)  //行数下标最大值
#define INDEX_MAX (CAMERA_W - 1) //列数下标最大值
#define CENTER_POINT 39             

#define TURNDECIDE 20     
#define STRAIGHTDECIDE 12 

/*外部变量声明*/
extern float g_f_Offset;  //根据图像计算的偏差
extern float g_f_Kp;      //舵机PID控制比例系数
extern float g_f_Td;      //PID控制微分系数
extern uint32 g_u32Speed; //根据图像计算的速度
extern bool g_b_OLEDSHOW;

//存储图像像素信息的二维数组，一个字节对应1个像素
extern uint8 Img[CAMERA_H][CAMERA_W];

//存储图像像素信息的一维数组，一个字节对应8个像素
extern uint8 ImgBuffer[CAMERA_SIZE];

/*外部函数声明*/
extern void Picture_Deal(void); //图像采集和处理
extern void OLED_Drawimg();     //写鹰眼摄像头函数
extern void Stopcar(void);      //电机转速置零即控制停车

#endif
