/*
 *	  文件名称：	Picture_Deal.c
 *	  作者： 	    光电卓越肖佑锋
 *	  版本：		V4.0
 *	  说明：		图像处理相关函数实现文件
 *	  创建日期：	2018-02-13
 *	  修改日期：	2018-07-28
 *	  修改者：		肖佑锋
 *	  修改目的:		完善
 */
#include "include.h"
#include "stdbool.h"
#include "Motor_Control.h"
#include "Steering_Control.h"
#include "Picture_Deal.h"

/*宏定义*/
//#define DEBUGING//被定义时调试语句就参加编译
//自定义输出信息宏，用于调试模式下输出变量值和变量名称
//使用时注意变量的类型
//#define PRINT(x) printf(#x" =%d\n",x)
#define IMGSHOW 0       
#define OLEDSHOW 1       
#define OLEDBORDERSHOW 0 
#define STOPCARADD 1     
#define TRACKWIDTH_INI 0 

/*外部变量定义*/
//根据图像计算的偏差
float g_f_Offset = 0.0f;

//舵机PID控制比例系数
float g_f_Kp = 0.0f;

//PID控制微分系数
float g_f_Td = 0.0f;

//根据图像计算的速度
uint32 g_u32Speed = 0;

//存储图像像素信息的二维数组，一个字节对应1个像素
uint8 Img[CAMERA_H][CAMERA_W];

//存储图像像素信息的一维数组，一个字节对应8个像素
uint8 ImgBuffer[CAMERA_SIZE];

/*内部变量定义*/
//全局变量，使用前注意初始化
uint8 Width[CAMERA_H] =
    {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 10, 12, 13, 14,
        15, 16, 17, 18, 19, 20, 22, 22, 24, 25,
        26, 28, 28, 30, 30, 32, 32, 34, 35, 36,
        37, 38, 39, 40, 41, 43, 43, 45, 45, 47,
        47, 49, 49, 51, 51, 53, 53, 55, 55, 57};

uint8 ActualWidth[CAMERA_H] =
    {
        0};

uint8 LeftBorder[CAMERA_H] =
    {
        0};

//左边界位置记录数组
uint8 RightBorder[CAMERA_H] =
    {
        INDEX_MAX};

//右边界位置记录数组
uint8 MidBorder[CAMERA_H] =
    {
        0};

uint8 RowBorder[CAMERA_W] =
    {
        0};

uint8 g_u8Farline = FARLINE;     
uint8 g_u8RowMinX = CENTER_POINT;

bool g_b_CrossFlag = false;      //用于标记十字赛道
bool g_b_RoundAboutFlag = false; //用于标记进入环岛
bool g_b_RoundDirection = false; //标记环岛的转向

//定义枚举类型变量
enum BorderFindFlag
{
    MISS = 0, //边界丢失
    FIND = 1, //边界找到
    MEND = 2  //边界是补上的
} e_LeftBFF[CAMERA_H],
    e_RightBFF[CAMERA_H];

enum BorderState
{
    AllFind,   //左右边界都找到
    LeftMiss,  //左边界缺失
    RightMiss, //右边界缺失
    AllMiss    //左右边界都缺失
} e_BorderState[CAMERA_H];

enum linestate {straightline,straightocurve,curveline};
enum carstate {turnleft,gostraight,turnright};
enum linestate lastLineState=straightline;
enum linestate currentLineState=straightline;
enum carstate lastCarState=gostraight;
enum carstate currentCarState=gostraight;

struct SomeValueSave
{
    uint8 m_u8DownLeftPointCopy;  
    uint8 m_u8DownRightPointCopy; 
    uint8 m_u8UpLeftPointCopy;   
    uint8 m_u8UpRightPointCopy;   
} st_SomeValueSave =
    {
        11, 68, 32, 47};

struct ForkPosition
{
    uint8 u8Row;     
    uint8 u8Line;     
    uint8 u8ForkType; 
    bool b_FindFork;  
} st_ForkPosition[5] =
    {
        //元素0占位标记是否找到任一类型点
        {
            0, 0, 0, false},
        {
            0, 0, 0, false},

        {
            0, 0, 0, false},

        {
            0, 0, 0, false},

        {
            0, 0, 0, false}};

/*内部函数声明*/
void Picture_Get(void);  
void Img_Filter(void);   
void Img_Rec(void);   
void Border_Get(void); 
void Send_Image(void);   
void Border_Show(void);  
void Img_Reduce(uint8 *p_OriginalArray, uint8 *p_GetArray,
                uint32 u32Length); 
void GlobalVar_INI(void);         

//边界提取的几种方式
void FirstLine_GetBorder(void);                               
uint8 LeftBorder_Get(uint8 u8ReferCenter, uint8 u8DealLine);  
uint8 RightBorder_Get(uint8 u8ReferCenter, uint8 u8DealLine); 
uint8 RowBorder_Get(uint8 u8Row);                             

void TrackType_Decide(void);              
void LineStateDecide(void);               
void BreakPoint_Find(void);              
uint8 Blackpoint_Get(uint8 u8FindBorder); 
void Border_Mend(int8 s8UpX, int8 s8UpY, int8 s8DownX, int8 s8DownY,
                 bool b_Direction); 

//几种特殊赛道类型的处理
void Cross_Deal(void);                
void XieCross_Deal(uint8 u8DealType); 
void Roundabout_Deal(void);           
void Slope_Deal(void);               
void Obstacle_Deal(void);             
bool Stopcar_Test(void);              

void Offset_Cal(); 

/*外部函数实现*/

void Picture_Deal(void)
{
    Picture_Get(); //获取图像
    Img_Filter();  //图像滤波
    Img_Rec();  //图像识别及处理

#if STOPCARADD

    if (Stopcar_Test())
    {
        g_u32Speed = 0;
        Stopcar();
        Steering_Init();

        while (1)
        {
            ;
        }
    }

#endif

#if IMGSHOW //边界上位机显示

    Border_Show(); //图像上位机显示，只显示边界加最远行
    Send_Image();  //图像上位机显示，原图像加边界加最远行
#endif

#if OLEDSHOW //原图像加边界加最远行液晶显示

    for (uint8 i = LINE_MAX; i >= g_u8Farline; i--)
    {
        Img[i][LeftBorder[i]] = IMG_BLACK;
        Img[i][RightBorder[i]] = IMG_BLACK;
        Img[i][MidBorder[i]] = IMG_BLACK; 
        //Img[i][g_u8RowMinX] = IMG_BLACK; 
    }

    for (uint8 i = LeftBorder[g_u8Farline]; i <= RightBorder[g_u8Farline]; i++)
    {
        Img[g_u8Farline][i] = IMG_BLACK;
    }

    OLED_Drawimg();
#endif

#if OLEDBORDERSHOW //图像边界加中线显示

    uint8 ImgBorder[CAMERA_SIZE] =
        {
            0};

    //给边界对应的像素点赋值
    for (uint8 i = LINE_MAX; i > g_u8Farline; i--)
    {
        ImgBorder[(LeftBorder[i] + i * CAMERA_W) / 8] += 0x01 << (7 - LeftBorder[i] % 8);
        ImgBorder[(RightBorder[i] + i * CAMERA_W) / 8] += 0x01 << (7 - RightBorder[i] % 8);
        ImgBorder[(MidBorder[i] + i * CAMERA_W) / 8] += 0x01 << (7 - MidBorder[i] % 8);
    }

    img_extract(Img, ImgBorder, CAMERA_SIZE);

    for (uint8 i = 0; i <= INDEX_MAX; i++)
    {
        Img[g_u8Farline][i] = IMG_BLACK;
    }

    OLED_Drawimg();
#endif
}

/*内部函数实现*/

void GlobalVar_INI(void)
{
    g_u8Farline = FARLINE;      
    g_u8RowMinX = CENTER_POINT; 
    g_f_Offset = 0.0f;          //偏差初始化
    g_f_Kp = 0.0f;              //P值初始化
    g_f_Td = 0.0f;              //D值初始化
    g_u32Speed = 0;             //速度初始化

    st_ForkPosition[0].b_FindFork = false; 
    st_ForkPosition[1].b_FindFork = false; 
    st_ForkPosition[2].b_FindFork = false; 
    st_ForkPosition[3].b_FindFork = false; 
    st_ForkPosition[4].b_FindFork = false; 
    st_ForkPosition[0].u8ForkType = 0;     
    st_ForkPosition[1].u8ForkType = 0;     
    st_ForkPosition[2].u8ForkType = 0;     
    st_ForkPosition[3].u8ForkType = 0;     
    st_ForkPosition[4].u8ForkType = 0;
}

void Picture_Get(void)
{
    camera_get_img(); 
    img_extract(Img, ImgBuffer,
                CAMERA_SIZE); 
}

void Img_Filter(void)
{
    #define NUM_FILTER 5 
    for (uint8 i = LINE_MAX; i > 1; i--)
    {
        uint8 j = 0;

        if (Img[i][j] == IMG_WHITE)
        {
            for (uint8 k = j + 1; k <= j + NUM_FILTER; k++)
            {
                if (Img[i][k] == IMG_BLACK)
                {
                    Img[i][j] = IMG_BLACK;
                    break;
                }
            }
        }

        for (j = 1; j < INDEX_MAX - NUM_FILTER; j++)
        {
            if (Img[i][j] == IMG_WHITE && Img[i][j - 1] == IMG_BLACK)
            {
                for (uint8 k = j + 1; k <= j + NUM_FILTER; k++)
                {
                    if (Img[i][k] == IMG_BLACK)
                    {
                        Img[i][j] = IMG_BLACK;
                        break;
                    }
                }
            }
        }

        j = INDEX_MAX;
        if (Img[i][j] == IMG_WHITE)
        {
            for (uint8 k = j - 1; k >= j - NUM_FILTER; k--)
            {
                if (Img[i][k] == IMG_BLACK)
                {
                    Img[i][j] = IMG_BLACK;
                    break;
                }
            }
        }

        for (j = INDEX_MAX - 1; j >= INDEX_MAX - NUM_FILTER; j--)
        {
            if (Img[i][j] == IMG_WHITE && Img[i][j + 1] == IMG_BLACK)
            {
                for (uint8 k = j - 1; k >= j - NUM_FILTER; k--)
                {
                    if (Img[i][k] == IMG_BLACK)
                    {
                        Img[i][j] = IMG_BLACK;
                        break;
                    }
                }
            }
        }
    }

    #define BLACK_FILTER 2
    for(uint8 i = LINE_MAX; i > 10; i--)
    {
        for (uint8 j = 1; j < INDEX_MAX - BLACK_FILTER; j++)
        {
            if (Img[i][j] == IMG_BLACK && Img[i][j - 1] == IMG_WHITE)
            {
                for (uint8 k = j + 1; k <= j + BLACK_FILTER; k++)
                {
                    if (Img[i][k] == IMG_WHITE)
                    {
                        Img[i][j] = IMG_WHITE;
                        break;
                    }
                }
            }
        }

        for (uint8 j = INDEX_MAX - 1; j >= INDEX_MAX - BLACK_FILTER; j--)
        {
            if (Img[i][j] == IMG_BLACK && Img[i][j + 1] == IMG_WHITE)
            {
                for (uint8 k = j - 1; k >= j - BLACK_FILTER; k--)
                {
                    if (Img[i][k] == IMG_WHITE)
                    {
                        Img[i][j] = IMG_WHITE;
                        break;
                    }
                }
            }
        }
    }
}

void Img_Rec(void)
{
    GlobalVar_INI();

    Border_Get();

    TrackType_Decide();

    Border_Get();

    Offset_Cal();
}

void Border_Get(void)
{
    uint8 u8CenterL;     
    uint8 u8CenterR;     
    uint8 u8LeftRowMinX; 
    uint8 u8LeftRowMinY;
    uint8 u8RightRowMinX; 
    uint8 u8RightRowMinY;
    uint8 u8MidRowMinX; 
    uint8 u8MidRowMinY;

    FirstLine_GetBorder();

    u8CenterL = (LeftBorder[LINE_MAX] + RightBorder[LINE_MAX]) / 2;
    u8CenterR = u8CenterL + 1;

    u8LeftRowMinX = u8CenterL;
    u8LeftRowMinY = RowBorder_Get(u8CenterL);
    RowBorder[u8LeftRowMinX] = u8LeftRowMinY;

    for (int8 i = u8CenterL - 1; i >= LeftBorder[LINE_MAX]; i--)
    {
        if (u8LeftRowMinY > RowBorder_Get(i))
        {
            u8LeftRowMinX = i;
            u8LeftRowMinY = RowBorder_Get(i);
        }
        RowBorder[i] = RowBorder_Get(i);
    }

    u8RightRowMinX = u8CenterR;
    u8RightRowMinY = RowBorder_Get(u8CenterR);
    RowBorder[u8RightRowMinX] = u8RightRowMinY;

    for (int8 i = u8CenterR + 1; i <= RightBorder[LINE_MAX]; i++)
    {
        if (u8RightRowMinY > RowBorder_Get(i))
        {
            u8RightRowMinX = i;
            u8RightRowMinY = RowBorder_Get(i);
        }
        RowBorder[i] = RowBorder_Get(i);
    }

    if (u8RightRowMinY > u8LeftRowMinY)
    {
        u8MidRowMinX = u8LeftRowMinX;
        u8MidRowMinY = u8LeftRowMinY;
    }
    else if (u8RightRowMinY < u8LeftRowMinY)
    {
        u8MidRowMinX = u8RightRowMinX;
        u8MidRowMinY = u8RightRowMinY;
    }
    else
    {
        if (u8CenterL - u8LeftRowMinX > u8RightRowMinX - u8CenterR)
        {
            u8MidRowMinX = u8RightRowMinX;
            u8MidRowMinY = u8RightRowMinY;
        }

        else if (u8CenterL - u8LeftRowMinX < u8RightRowMinX - u8CenterR)
        {
            u8MidRowMinX = u8LeftRowMinX;
            u8MidRowMinY = u8LeftRowMinY;
        }
        else
        {
            u8MidRowMinX = (uint8)((u8CenterL + u8CenterR) / 2.0 + 0.5);
            u8MidRowMinY = (uint8)((RowBorder_Get(u8CenterL) + RowBorder_Get(u8CenterR)) / 2.0 + 0.5);
        }
    }

    if (u8MidRowMinY <= FARLINE)
        u8MidRowMinY = FARLINE;

    g_u8RowMinX = u8MidRowMinX;
    g_u8Farline = u8MidRowMinY;

    if (u8MidRowMinX == 0)
    {
        for (uint8 i = LINE_MAX - 1; i >= g_u8Farline; i--)
        {
            LeftBorder[i] = 0;
            e_LeftBFF[i] = MISS;
            RightBorder[i] = INDEX_MAX;
            e_RightBFF[i] = MISS;

            for (int8 j = 1; j <= INDEX_MAX; j++)
            {
                if (Img[i][j] == IMG_BLACK && Img[i][j - 1] == IMG_WHITE)
                {
                    RightBorder[i] = j - 1;
                    e_RightBFF[i] = FIND;
                    e_BorderState[i] = LeftMiss;
                    break;
                }
            }

            if (e_RightBFF[i] != FIND)
            {
                RightBorder[i] = INDEX_MAX;
                e_RightBFF[i] = MISS;
                e_BorderState[i] = AllMiss;
            }

            //计算中线
            MidBorder[i] = (uint8)((LeftBorder[i] + RightBorder[i]) / 2.0 + 0.5);

            ActualWidth[i] = RightBorder[i] - LeftBorder[i];
        }
    }

    else if (u8MidRowMinX == INDEX_MAX)
    {
        for (uint8 i = LINE_MAX - 1; i >= g_u8Farline; i--)
        {
            LeftBorder[i] = 0;
            e_LeftBFF[i] = MISS;
            RightBorder[i] = INDEX_MAX;
            e_RightBFF[i] = MISS;

            for (int8 j = INDEX_MAX - 1; j >= 0; j--)
            {
                if (Img[i][j] == IMG_BLACK && Img[i][j + 1] == IMG_WHITE)
                {
                    LeftBorder[i] = j + 1;
                    e_LeftBFF[i] = FIND;
                    e_BorderState[i] = RightMiss;
                    break;
                }
            }

            if (e_LeftBFF[i] != FIND)
            {
                LeftBorder[i] = 0;
                e_LeftBFF[i] = MISS;
                e_BorderState[i] = AllMiss;
            }

            //计算中线
            MidBorder[i] = (uint8)((LeftBorder[i] + RightBorder[i]) / 2.0 + 0.5);

            ActualWidth[i] = RightBorder[i] - LeftBorder[i];
        }
    }

    else
    {
        for (uint8 i = LINE_MAX - 1; i >= g_u8Farline; i--)
        {
            LeftBorder[i] = 0;
            e_LeftBFF[i] = MISS;
            RightBorder[i] = INDEX_MAX;
            e_RightBFF[i] = MISS;

            for (int8 j = u8MidRowMinX; j >= 0; j--)
            {
                if (Img[i][j] == IMG_BLACK && Img[i][j + 1] == IMG_WHITE)
                {
                    LeftBorder[i] = j + 1;
                    e_LeftBFF[i] = FIND;
                    break;
                }
            }

            if (e_LeftBFF[i] != FIND)
            {
                LeftBorder[i] = 0;
                e_LeftBFF[i] = MISS;
            }

            for (int8 j = u8MidRowMinX; j <= INDEX_MAX; j++)
            {
                if (Img[i][j] == IMG_BLACK && Img[i][j - 1] == IMG_WHITE)
                {
                    RightBorder[i] = j - 1;
                    e_RightBFF[i] = FIND;
                    break;
                }
            }

            if (e_RightBFF[i] != FIND)
            {
                RightBorder[i] = INDEX_MAX;
                e_RightBFF[i] = MISS;
            }

            if (e_LeftBFF[i] == FIND && e_RightBFF[i] == FIND)
                e_BorderState[i] = AllFind;
            else if (e_LeftBFF[i] == FIND && e_RightBFF[i] == MISS)
                e_BorderState[i] = RightMiss;
            else if (e_LeftBFF[i] == MISS && e_RightBFF[i] == FIND)
                e_BorderState[i] = LeftMiss;
            else
                e_BorderState[i] = AllMiss;

            //计算中线
            MidBorder[i] = (uint8)((LeftBorder[i] + RightBorder[i]) / 2.0 + 0.5);

            ActualWidth[i] = RightBorder[i] - LeftBorder[i];
        }
    }
}

void FirstLine_GetBorder(void)
{
#define LEFTPOINT 19
#define RIGHTPOINT 59
    uint8 u8Left, u8Right; 
    uint8 u8DealLine = LINE_MAX;
    bool b_FindLeftFlag = false;//左边界找到标志
    bool b_FindRightFlag = false;//右边界找到标志

    LeftBorder[u8DealLine] = 0;
    RightBorder[u8DealLine] = INDEX_MAX; 

    if (Img[u8DealLine][CENTER_POINT] == IMG_WHITE && Img[u8DealLine][CENTER_POINT + 1] == IMG_WHITE) 
    {
        for (u8Left = CENTER_POINT, u8Right = CENTER_POINT + 1; u8Left > 2;
             u8Left--, u8Right++)
        {
            if (!b_FindRightFlag && Img[u8DealLine][u8Right + 1] == IMG_BLACK &&
                Img[u8DealLine][u8Right + 2] == IMG_BLACK)
            {
                b_FindRightFlag = true;
                RightBorder[u8DealLine] = u8Right;
            }

            if (!b_FindLeftFlag && Img[u8DealLine][u8Left - 1] == IMG_BLACK &&
                Img[u8DealLine][u8Left - 2] == IMG_BLACK)
            {
                b_FindLeftFlag = true;
                LeftBorder[u8DealLine] = u8Left;
            }

            if (b_FindLeftFlag && b_FindRightFlag)
            {
                if (RightBorder[u8DealLine] - LeftBorder[u8DealLine] > 15)
                    break;
                else if (Img[u8DealLine][LEFTPOINT] == IMG_WHITE &&
                         Img[u8DealLine][LEFTPOINT + 1] == IMG_WHITE)
                {
                    b_FindLeftFlag = false;
                    b_FindRightFlag = false;
                    goto LPISWHITE;
                }
                else if (Img[u8DealLine][RIGHTPOINT] == IMG_WHITE &&
                         Img[u8DealLine][RIGHTPOINT + 1] == IMG_WHITE)
                {
                    b_FindLeftFlag = false;
                    b_FindRightFlag = false;
                    goto RPISWHITE;
                }
                else
                {
                    b_FindLeftFlag = false;
                    b_FindRightFlag = false;
                    goto BORDERFIND;
                }
            }
        }
    }
    else if (Img[u8DealLine][LEFTPOINT] == IMG_WHITE && Img[u8DealLine][LEFTPOINT + 1] == IMG_WHITE) 
    {
    LPISWHITE:
        for (u8Left = LEFTPOINT, u8Right = LEFTPOINT + 1; u8Right < INDEX_MAX - 2;
             u8Right++)
        {
            if (!b_FindLeftFlag && u8Left > 2 && Img[u8DealLine][u8Left - 1] == IMG_BLACK &&
                Img[u8DealLine][u8Left - 2] == IMG_BLACK)
            {
                b_FindLeftFlag = true;
                LeftBorder[u8DealLine] = u8Left;
                u8Left--;
            }

            if (!b_FindRightFlag && Img[u8DealLine][u8Right + 1] == IMG_BLACK &&
                Img[u8DealLine][u8Right + 2] == IMG_BLACK)
            {
                b_FindRightFlag = true;
                RightBorder[u8DealLine] = u8Right;
            }

            if (b_FindLeftFlag && b_FindRightFlag)
            {
                if (RightBorder[u8DealLine] - LeftBorder[u8DealLine] > 15)
                    break;
                else if (Img[u8DealLine][RIGHTPOINT] == IMG_WHITE &&
                         Img[u8DealLine][RIGHTPOINT + 1] == IMG_WHITE)
                {
                    b_FindLeftFlag = false;
                    b_FindRightFlag = false;
                    goto RPISWHITE;
                }
                else
                {
                    b_FindLeftFlag = false;
                    b_FindRightFlag = false;
                    goto BORDERFIND;
                }
            }
        }
    }
    else if (Img[u8DealLine][RIGHTPOINT] == IMG_WHITE && Img[u8DealLine][RIGHTPOINT + 1] == IMG_WHITE) 
    {
    RPISWHITE:
        for (u8Left = RIGHTPOINT, u8Right = RIGHTPOINT + 1; u8Left > 2; u8Left--)
        {
            if (!b_FindLeftFlag && Img[u8DealLine][u8Left - 1] == IMG_BLACK &&
                Img[u8DealLine][u8Left - 2] == IMG_BLACK)
            {
                b_FindLeftFlag = true;
                LeftBorder[u8DealLine] = u8Left;
            }

            if (!b_FindRightFlag && u8Right < INDEX_MAX - 2 && Img[u8DealLine][u8Right + 1] == IMG_BLACK &&
                Img[u8DealLine][u8Right + 2] == IMG_BLACK)
            {
                b_FindRightFlag = true;
                RightBorder[u8DealLine] = u8Right;
                u8Right++;
            }

            if (b_FindLeftFlag && b_FindRightFlag)
            {
                if (RightBorder[u8DealLine] - LeftBorder[u8DealLine] > 15)
                    break;
                else
                {
                    b_FindLeftFlag = false;
                    b_FindRightFlag = false;
                    goto BORDERFIND;
                }
            }
        }
    }
    else 
    {
    BORDERFIND:
        for (uint8 i = 0; i < INDEX_MAX - 3; i++) //右边界
        {
            if (Img[u8DealLine][i] == IMG_WHITE && Img[u8DealLine][i + 1] == IMG_WHITE &&
                Img[u8DealLine][i + 2] == IMG_BLACK && Img[u8DealLine][i + 3] == IMG_BLACK)
            {
                b_FindRightFlag = true;
                RightBorder[u8DealLine] = i + 1;
            }
        }

        for (uint8 i = INDEX_MAX; i > 3; i--) //左边界
        {
            if (Img[u8DealLine][i] == IMG_WHITE && Img[u8DealLine][i - 1] == IMG_WHITE &&
                Img[u8DealLine][i - 2] == IMG_BLACK && Img[u8DealLine][i - 3] == IMG_BLACK)
            {
                b_FindLeftFlag = true;
                LeftBorder[u8DealLine] = i - 1;
            }
        }
    }

    if (b_FindLeftFlag)
        e_LeftBFF[u8DealLine] = FIND;
    else
        e_LeftBFF[u8DealLine] = MISS;

    if (b_FindRightFlag)
        e_RightBFF[u8DealLine] = FIND;
    else
        e_RightBFF[u8DealLine] = MISS;

    if (e_LeftBFF[u8DealLine] == FIND && e_RightBFF[u8DealLine] == FIND)
        e_BorderState[u8DealLine] = AllFind;
    else if (e_LeftBFF[u8DealLine] == FIND && e_RightBFF[u8DealLine] == MISS)
        e_BorderState[u8DealLine] = RightMiss;
    else if (e_LeftBFF[u8DealLine] == MISS && e_RightBFF[u8DealLine] == FIND)
        e_BorderState[u8DealLine] = LeftMiss;
    else
        e_BorderState[u8DealLine] = AllMiss;

    //计算中线
    MidBorder[u8DealLine] = (uint8)((LeftBorder[u8DealLine] + RightBorder[u8DealLine]) / 2.0 + 0.5);

    ActualWidth[u8DealLine] = RightBorder[u8DealLine] - LeftBorder[u8DealLine];
}

uint8 LeftBorder_Get(uint8 u8ReferCenter, uint8 u8DealLine)
{
    for (uint8 i = u8ReferCenter - 1; i > 0; i--)
    {
        if (Img[u8DealLine][i] == IMG_BLACK && Img[u8DealLine][i + 1] == IMG_WHITE &&
            Img[u8DealLine][i + 2] == IMG_WHITE)
        {
            return (i + 1);
        }
    }

    return 0;
}

uint8 RightBorder_Get(uint8 u8ReferCenter, uint8 u8DealLine)
{
    for (uint8 i = u8ReferCenter + 1; i < INDEX_MAX; i++)
    {
        if (Img[u8DealLine][i] == IMG_BLACK && Img[u8DealLine][i - 1] == IMG_WHITE &&
            Img[u8DealLine][i - 2] == IMG_WHITE)
        {
            return (i - 1);
        }
    }

    return (INDEX_MAX);
}

void XieCross_Deal(uint8 u8DealType)
{
    if (u8DealType == 1)
    {
        uint8 u8DownPointX, u8DownPointY, u8UpPointX, u8UpPointY;
        bool b_DownFind = false; 
        bool b_UpFind = false;   

        for (uint8 i = RightBorder[LINE_MAX]; i > 10; i--)
        {
            if (!b_DownFind && RowBorder[i] - RowBorder[i - 1] > 7)
            {
                u8DownPointX = i;
                u8DownPointY = RowBorder[i];
                b_DownFind = true;
                i -= 2;
            }

            if (b_DownFind && RowBorder[i] > RowBorder[i - 1] && RowBorder[i - 1] > RowBorder[i - 4])
            {
                u8UpPointX = i;
                u8UpPointY = RowBorder[i];
                b_UpFind = true;
            }

            if (b_DownFind && b_UpFind)
            {
                break;
            }
        }

        if (!b_DownFind)
        {
            u8DownPointX = RightBorder[LINE_MAX];
            u8DownPointY = LINE_MAX;
        }

        if (!b_UpFind)
        {
            u8UpPointX = RightBorder[g_u8Farline + 7];
            u8UpPointY = g_u8Farline + 7;
        }

        //补右边界
        Border_Mend(u8UpPointX, u8UpPointY, u8DownPointX, u8DownPointY, true);
    }
    else if (u8DealType == 2)
    {
        uint8 u8DownPointX, u8DownPointY, u8UpPointX, u8UpPointY;
        bool b_DownFind = false; 
        bool b_UpFind = false;   

        for (uint8 i = LeftBorder[LINE_MAX]; i < 70; i++)
        {
            if (!b_DownFind && RowBorder[i] - RowBorder[i + 1] > 7)
            {
                u8DownPointX = i;
                u8DownPointY = RowBorder[i];
                b_DownFind = true;
                i += 2;
            }

            if (b_DownFind && RowBorder[i] > RowBorder[i + 1] && RowBorder[i + 1] > RowBorder[i + 4])
            {
                u8UpPointX = i;
                u8UpPointY = RowBorder[i];
                b_UpFind = true;
            }

            if (b_DownFind && b_UpFind)
            {
                break;
            }
        }

        if (!b_DownFind)
        {
            u8DownPointX = LeftBorder[LINE_MAX];
            u8DownPointY = LINE_MAX;
        }

        if (!b_UpFind)
        {
            u8UpPointX = LeftBorder[g_u8Farline + 7];
            u8UpPointY = g_u8Farline + 7;
        }

        //补左边界
        Border_Mend(u8UpPointX, u8UpPointY, u8DownPointX, u8DownPointY, false);
    }
}

uint8 RowBorder_Get(uint8 u8Row)
{
    for (uint8 i = LINE_MAX; i >= FARLINE-1; i--)
    {
        if (Img[i][u8Row] == IMG_BLACK)
        {
            if (i == LINE_MAX)
                return (LINE_MAX);
            else
                return (i + 1);
        }
    }

    return (FARLINE);
}

void TrackType_Decide(void)
{
    uint8 u8AFCount = 0;
    uint8 u8LMCount = 0;
    uint8 u8RMCount = 0;
    uint8 u8AMCount = 0;
    uint8 u8WhiteLine = 0;
    static bool s_b_XieCrossFlag = false; 
    static bool s_b_XieCrossDir = false;  

    BreakPoint_Find(); 

    for (uint8 i = LINE_MAX; i > g_u8Farline; i--)
    {
        switch (e_BorderState[i])
        {
        case AllFind:
        {
            u8AFCount++;
            break;
        }

        case LeftMiss:
        {
            u8LMCount++;
            break;
        }

        case RightMiss:
        {
            u8RMCount++;
            break;
        }

        case AllMiss:
        {
            u8AMCount++;
            break;
        }
        }

        if (ActualWidth[i] > 70)
        {
            u8WhiteLine++;
        }
    }

    if (!g_b_RoundAboutFlag && !g_b_CrossFlag && !s_b_XieCrossFlag)
    {
        for (uint8 i = LINE_MAX - 6; i > 30 && i > g_u8Farline; i--)
        {
            if (ABS(ActualWidth[i] - Width[i]) <= 4 && ABS(ActualWidth[i - 2] - Width[i - 2]) <= 4 &&
                ABS(ActualWidth[i - 4] - (uint8)(Width[i - 4] / 2.0 + 40)) <= 7 &&
                ABS(ActualWidth[i - 6] - (uint8)(Width[i - 6] / 2.0 + 40)) <= 7 &&
                ActualWidth[i - 4] > ActualWidth[i - 6] && ActualWidth[i] > ActualWidth[i - 2] && (u8LMCount > 10 || u8RMCount > 10))
            {
                g_b_RoundAboutFlag = true;

                if (e_BorderState[i - 6] == LeftMiss && st_ForkPosition[3].b_FindFork)
                {
                    g_b_RoundDirection = false; 
                }
                else if (e_BorderState[i - 6] == RightMiss && st_ForkPosition[4].b_FindFork)
                {
                    g_b_RoundDirection = true; 
                }
                else
                {
                    g_b_RoundAboutFlag = false; 
                }
                break;
            }
        }
    }

    if (g_b_RoundAboutFlag)
    {
        Roundabout_Deal();
        return;
    }

    if (!g_b_CrossFlag && st_ForkPosition[3].b_FindFork && st_ForkPosition[4].b_FindFork)
    {
        for (uint8 i = LINE_MAX; i > g_u8Farline && i > 30; i--) 
        {
            if (ABS(ActualWidth[i] - Width[i]) <= 4 && ABS(ActualWidth[i - 3] - Width[i - 3]) <= 4 &&
                ActualWidth[i - 9] > 75 && ActualWidth[i] > ActualWidth[i - 3] && u8AMCount > 15)
            {
                s_b_XieCrossFlag = false;
                g_b_CrossFlag = true;
                break;
            }
        }
    }

    if (!g_b_CrossFlag && (u8AMCount > 20 || (u8WhiteLine > 20 && u8AMCount > 12)))
    {
        s_b_XieCrossFlag = false;
        g_b_CrossFlag = true;
    }

    if (g_b_CrossFlag)
    {
        Cross_Deal();
        return;
    }

    if (!s_b_XieCrossFlag && ((st_ForkPosition[1].b_FindFork && st_ForkPosition[3].b_FindFork) || (st_ForkPosition[2].b_FindFork && st_ForkPosition[4].b_FindFork)))
    {
        for (uint8 i = 50; i > 22; i--)
        {
            if (ActualWidth[i] < ActualWidth[i - 3] && ActualWidth[i] < ActualWidth[i + 3] &&
                ActualWidth[i + 3] <= ActualWidth[i - 3] && ActualWidth[i + 5] < ActualWidth[i - 5] &&
                ActualWidth[i - 5] > ActualWidth[i - 3] && ActualWidth[i + 5] > ActualWidth[i + 3] &&
                (u8LMCount > 25 || u8RMCount > 25) && (e_BorderState[i] == LeftMiss || e_BorderState[i] == RightMiss))
            {
                s_b_XieCrossFlag = true;

                if (e_BorderState[i] == LeftMiss)
                    s_b_XieCrossDir = false;
                else
                    s_b_XieCrossDir = true;
            }
        }
    }

    if (s_b_XieCrossFlag)
    {
        if (!s_b_XieCrossDir && u8LMCount > 10)
            XieCross_Deal(1); 
        else if (s_b_XieCrossDir && u8RMCount > 10)
            XieCross_Deal(2); 
        else
        {
            s_b_XieCrossFlag = false;
            g_b_CrossFlag = true;
            Cross_Deal();
        }

        return;
    }
}

void BreakPoint_Find(void)
{
    bool b_LeftForkDecide = false; 
    for (int8 i = LeftBorder[LINE_MAX]; i < g_u8RowMinX; i++)
    {
        if (!b_LeftForkDecide && RowBorder[i] < RowBorder[i + 1])
        {
            b_LeftForkDecide = true;
            i += 2;
        }

        if (b_LeftForkDecide)
        {
            if (RowBorder[i] <= RowBorder[i + 1])
            {
                b_LeftForkDecide = true;
            }
            else if (RowBorder[i] > RowBorder[i + 1])
            {
                st_ForkPosition[1].b_FindFork = true;
                st_ForkPosition[1].u8ForkType = 1;
                st_ForkPosition[1].u8Line = RowBorder[i];
                st_ForkPosition[1].u8Row = i;
                break;
            }
        }
    }

    for (uint8 i = LeftBorder[LINE_MAX]; i < g_u8RowMinX; i++)
    {
        if (st_ForkPosition[1].b_FindFork && RowBorder[i] < st_ForkPosition[1].u8Line)
            break;
        if (!(st_ForkPosition[3].b_FindFork) && RowBorder[i] - RowBorder[i + 1] > 8)
        {
            st_ForkPosition[3].b_FindFork = true;
            st_ForkPosition[3].u8ForkType = 2;
            st_ForkPosition[3].u8Line = RowBorder[i];
            st_ForkPosition[3].u8Row = i;
            break;
        }
    }

    bool b_RightForkDecide = false; 
    for (int8 i = RightBorder[LINE_MAX]; i > g_u8RowMinX; i--)
    {
        if (!b_RightForkDecide && RowBorder[i] < RowBorder[i - 1])
        {
            b_RightForkDecide = true;
            i -= 2;
        }

        if (b_RightForkDecide)
        {
            if (RowBorder[i] <= RowBorder[i - 1])
            {
                b_RightForkDecide = true;
            }
            else if (RowBorder[i] > RowBorder[i - 1])
            {
                st_ForkPosition[2].b_FindFork = true;
                st_ForkPosition[2].u8ForkType = 1;
                st_ForkPosition[2].u8Line = RowBorder[i];
                st_ForkPosition[2].u8Row = i;
                break;
            }
        }
    }

    for (uint8 i = RightBorder[LINE_MAX]; i > g_u8RowMinX; i--)
    {
        if (st_ForkPosition[2].b_FindFork && RowBorder[i] < st_ForkPosition[2].u8Line)
            break;
        if (!(st_ForkPosition[4].b_FindFork) && RowBorder[i] - RowBorder[i - 1] > 8)
        {
            st_ForkPosition[4].b_FindFork = true;
            st_ForkPosition[4].u8ForkType = 2;
            st_ForkPosition[4].u8Line = RowBorder[i];
            st_ForkPosition[4].u8Row = i;
            break;
        }
    }
}

uint8 Blackpoint_Get(uint8 u8FindBorder)
{
    uint8 u8BlackPoint = 0;

    for (uint8 i = 0; i <= INDEX_MAX; i++)
    {
        if (Img[u8FindBorder][i] == IMG_BLACK)
        {
            u8BlackPoint++;
        }
    }

    return u8BlackPoint;
}

void Cross_Deal(void)
{
    uint8 u8UpPointX, u8UpPointY, u8DownPointX, u8DownPointY;
    uint8 u8LimitLine = 30;             
    uint8 u8UpLineRes = 20;             
    bool b_LimitFind = false;           
    bool b_ULRFind = false;             
    bool b_UpFind = false;              
    bool b_DownFind = false;            
    static bool s_b_DownPassed = false; 

    for (uint8 i = LINE_MAX; i > g_u8Farline; i--)
    {
        if (e_BorderState[i] == AllMiss && !b_LimitFind)
        {
            u8LimitLine = i;
            b_LimitFind = true;
            i -= 4;
        }

        if (e_BorderState[i] == AllFind && !b_ULRFind && b_LimitFind)
        {
            b_ULRFind = true;
            if (i - 5 > g_u8Farline)
                u8UpLineRes = i - 5;
            else
                u8UpLineRes = g_u8Farline + 1;
        }

        if (b_LimitFind && b_ULRFind)
            break;
    }

    for (uint8 i = RightBorder[LINE_MAX]; i > g_u8RowMinX; i--)
    {
        if (!b_DownFind && RowBorder[i] > u8LimitLine && RowBorder[i] - RowBorder[i - 1] > 7)
        {
            u8DownPointX = i;
            u8DownPointY = RowBorder[i];
            b_DownFind = true;
            i -= 2;
        }

        if (!b_DownFind && RowBorder[i] < u8LimitLine)
        {
            u8DownPointX = st_SomeValueSave.m_u8DownRightPointCopy;
            u8DownPointY = LINE_MAX;
            b_DownFind = true;
            i -= 2;
        }

        if (b_DownFind && RowBorder[i] < u8LimitLine && RowBorder[i] > RowBorder[i - 1] &&
            RowBorder[i - 1] > RowBorder[i - 3])
        {
            u8UpPointX = i - 1;
            u8UpPointY = RowBorder[i - 1];
            b_UpFind = true;
        }

        if (b_DownFind && b_UpFind)
        {
            break;
        }
    }

    if (!b_UpFind)
    {
        u8UpPointX = RightBorder[u8UpLineRes];
        u8UpPointY = u8UpLineRes;
        b_UpFind = true;
    }

    Border_Mend(u8UpPointX, u8UpPointY, u8DownPointX, u8DownPointY, true);

    b_UpFind = false;
    b_DownFind = false;

    for (uint8 i = LeftBorder[LINE_MAX]; i < g_u8RowMinX; i++)
    {
        if (!b_DownFind && RowBorder[i] > u8LimitLine && RowBorder[i] - RowBorder[i + 1] > 7)
        {
            u8DownPointX = i;
            u8DownPointY = RowBorder[i];
            b_DownFind = true;
            i += 2;
        }

        if (!b_DownFind && RowBorder[i] < u8LimitLine)
        {
            u8DownPointX = st_SomeValueSave.m_u8DownLeftPointCopy;
            u8DownPointY = LINE_MAX;
            b_DownFind = true;
            i += 2;
        }

        if (b_DownFind && RowBorder[i] < u8LimitLine && RowBorder[i] > RowBorder[i + 1] &&
            RowBorder[i + 1] > RowBorder[i + 3])
        {
            u8UpPointX = i + 1;
            u8UpPointY = RowBorder[i + 1];
            b_UpFind = true;
        }

        if (b_DownFind && b_UpFind)
        {
            break;
        }
    }

    if (!b_UpFind)
    {
        u8UpPointX = LeftBorder[u8UpLineRes];
        u8UpPointY = u8UpLineRes;
        b_UpFind = true;
    }
    
    Border_Mend(u8UpPointX, u8UpPointY, u8DownPointX, u8DownPointY, false);

    if (u8LimitLine >= LINE_MAX - 1 && !s_b_DownPassed)
    {
        s_b_DownPassed = true;
    }

    if (s_b_DownPassed && (e_BorderState[55] == AllFind || e_BorderState[50] == AllFind))
    {
        s_b_DownPassed = false;
        g_b_CrossFlag = false;
    }
}

void Roundabout_Deal(void)
{
    static uint8 s_u8MendState = 1; 
    static bool s_b_Flag1 = false;
    static bool s_b_Flag2 = false;
    static bool s_b_Flag3 = false;
    static bool s_b_Flag4 = false;
    static bool s_b_SpeFlag = false;
    uint8 u8UpPointX, u8UpPointY, u8DownPointX, u8DownPointY;
    bool b_UpFind = false;           
    bool b_DownFind = false;         
    if (g_b_RoundDirection == false) 
    {
        switch (s_u8MendState) 
        {
        case 1:
        {
            uint8 u8LimitLine = 40;
            for (uint8 i = LINE_MAX; i > g_u8Farline; i--)
            {
                if (e_LeftBFF[i] == MISS)
                {
                    u8LimitLine = i;
                    if (u8LimitLine == LINE_MAX && !s_b_Flag1)
                        s_b_Flag1 = true;
                    if (s_b_Flag1 && u8LimitLine != LINE_MAX && LeftBorder[LINE_MAX] > 4)
                    {
                        s_u8MendState = 2;
                        goto MENDSTATE2; 
                    }
                    break;
                }
            }

            for (uint8 i = LeftBorder[LINE_MAX]; i < g_u8RowMinX; i++)
            {
                if (!b_DownFind && RowBorder[i] > u8LimitLine && RowBorder[i] - RowBorder[i + 1] > 8)
                {
                    b_DownFind = true;
                    u8DownPointX = i;
                    u8DownPointY = RowBorder[i];
                    i += 1;
                }
                if (!b_DownFind && RowBorder[i] <= u8LimitLine)
                {
                    b_DownFind = true;
                    u8DownPointX = st_SomeValueSave.m_u8DownLeftPointCopy;
                    u8DownPointY = LINE_MAX;
                }
                if (b_DownFind && RowBorder[i] < u8LimitLine && RowBorder[i] - RowBorder[i + 1] > 5)
                {
                    b_UpFind = true;
                    u8UpPointX = i;
                    u8UpPointY = RowBorder[i];
                    break;
                }
            }

            if (!b_UpFind)
            {
                b_UpFind = true;
                u8UpPointY = 30;
                u8UpPointX = (uint8)(40 - Width[u8UpPointY] / 2.0);
            }

            Border_Mend(u8UpPointX, u8UpPointY, u8DownPointX, u8DownPointY, false);

            break;
        }
        case 2:
        {
        MENDSTATE2:
            b_DownFind = true;
            u8DownPointX = INDEX_MAX;
            u8DownPointY = LINE_MAX;
            uint8 u8AMCount = 0; 

            if (!s_b_Flag2)
            {
                if (st_ForkPosition[3].b_FindFork) 
                {
                    if (st_ForkPosition[1].b_FindFork) 
                    {
                        b_UpFind = true;
                        u8UpPointX = st_ForkPosition[1].u8Row;
                        u8UpPointY = st_ForkPosition[1].u8Line;
                    }
                    else if (st_ForkPosition[2].b_FindFork) 
                    {
                        b_UpFind = true;
                        u8UpPointX = st_ForkPosition[2].u8Row;
                        u8UpPointY = st_ForkPosition[2].u8Line;
                    }
                    else
                    {
                        for (uint8 i = st_ForkPosition[3].u8Row + 1; i < g_u8RowMinX; i++)
                        {
                            if (RowBorder[i] > RowBorder[i + 1] && RowBorder[i + 1] > RowBorder[i + 4])
                            {
                                b_UpFind = true;
                                u8UpPointX = i;
                                u8UpPointY = RowBorder[i];
                                break;
                            }
                        }
                    }
                }
                if (!st_ForkPosition[3].b_FindFork) 
                {
                    s_b_Flag2 = true;
                }
            }
            if (s_b_Flag2)
            {
                b_UpFind = true;
                u8UpPointX = LeftBorder[LINE_MAX] + 16;
                u8UpPointY = RowBorder[u8UpPointX];
            }

            if (s_b_Flag2 && (e_RightBFF[LINE_MAX] == FIND || e_LeftBFF[LINE_MAX] == FIND) && !st_ForkPosition[1].b_FindFork && !st_ForkPosition[2].b_FindFork)
            {
                s_u8MendState = 3;
            }


            if (s_b_SpeFlag && (st_ForkPosition[4].b_FindFork || u8AMCount > 9))
            {
                s_u8MendState = 3;
            }

            Border_Mend(u8UpPointX, u8UpPointY, u8DownPointX, u8DownPointY, true);

            break;
        }
        case 3:
        {
            uint8 u8LMCount = 0; 
            if (st_ForkPosition[4].b_FindFork && !s_b_Flag3)
            {
                s_b_Flag3 = true;
            }
            if (s_b_Flag3)
            {
                for (uint8 i = LINE_MAX; i > g_u8Farline; i--)
                {
                    if (e_BorderState[i] == LeftMiss)
                    {
                        u8LMCount++;
                    }
                }
                b_UpFind = true;
                u8UpPointX = LeftBorder[LINE_MAX] + 5; 
                u8UpPointY = RowBorder[u8UpPointX];

                if (st_ForkPosition[4].b_FindFork)
                {
                    b_DownFind = true;
                    u8DownPointX = st_ForkPosition[4].u8Row;
                    u8DownPointY = st_ForkPosition[4].u8Line;
                }
                else 
                {
                    for (uint8 i = RightBorder[LINE_MAX]; i > g_u8RowMinX; i--)
                    {
                        if (RowBorder[i] - RowBorder[i - 1] > 5)
                        {
                            b_DownFind = true;
                            u8DownPointX = i;
                            u8DownPointY = RowBorder[u8DownPointX];
                            break;
                        }
                    }
                    if (!b_DownFind && (e_RightBFF[LINE_MAX] == MISS && e_RightBFF[48] == FIND))
                    {
                        s_u8MendState = 4;
                    }
                }

                if (u8LMCount > 15 && st_ForkPosition[1].b_FindFork)
                {
                    s_u8MendState = 4;
                }
            }

            Border_Mend(u8UpPointX, u8UpPointY, u8DownPointX, u8DownPointY, true);

            break;
        }
        case 4:
        {
            uint8 u8LimitLine = 30; 
            if (!s_b_Flag4)
            {
                if (st_ForkPosition[1].b_FindFork)
                {
                    s_b_Flag4 = true;
                }
            }
            if (s_b_Flag4)
            {
                b_DownFind = true;
                u8DownPointX = st_SomeValueSave.m_u8DownLeftPointCopy;
                u8DownPointY = LINE_MAX;

                for (uint8 i = LINE_MAX; i > g_u8Farline; i--)
                {
                    if (e_LeftBFF[i] == MISS && e_LeftBFF[i - 1] == FIND)
                    {
                        u8LimitLine = i - 1;
                        break;
                    }
                }

                if (st_ForkPosition[1].b_FindFork) 
                {
                    b_UpFind = true;
                    u8UpPointX = st_ForkPosition[1].u8Row;
                    u8UpPointY = st_ForkPosition[1].u8Line;
                }
                if ((u8LimitLine > 50 && e_LeftBFF[52] == FIND) || (e_LeftBFF[50] == FIND && ABS(ActualWidth[50] - Width[50]) <= 4))
                {
                    g_b_RoundAboutFlag = false; 
                    s_b_Flag1 = false;
                    s_b_Flag2 = false;
                    s_b_Flag3 = false;
                    s_b_Flag4 = false;
                    s_b_SpeFlag = false;
                    s_u8MendState = 1; 
                }
            }

            Border_Mend(u8UpPointX, u8UpPointY, u8DownPointX, u8DownPointY, false);

            break;
        }
        }
    }
    else 
    {
    }
}

void Obstacle_Deal(void)
{
}

void Slope_Deal(void)
{
}

void Border_Mend(int8 s8UpX, int8 s8UpY, int8 s8DownX, int8 s8DownY,
                 bool b_Direction)
{
    int8 s8DeltaX = s8UpX - s8DownX;
    int8 s8DeltaY = s8DownY - s8UpY;
    float f_Add = 0.0f;

    if (s8DeltaY == 0)
    {
        f_Add = 0.0f;
    }
    else
    {
        f_Add = (float)
                    s8DeltaX /
                s8DeltaY;
    }

    for (uint8 i = s8DownY; i >= s8UpY; i--)
    {
        if (!b_Direction) 
        {
            LeftBorder[i] = s8DownX + (int8)(f_Add * (s8DownY - i) + 0.5);
            if (i == LINE_MAX)
            {
                for (int8 j = LeftBorder[i]; j >= 0; j--)
                {
                    Img[i][j] = IMG_BLACK;
                }
            }
            else
            {
                Img[i][LeftBorder[i]] = IMG_BLACK;
                if (ABS(LeftBorder[i] - LeftBorder[i + 1]) >= 2)
                {
                    if (f_Add > 0)
                    {
                        for (uint8 k = LeftBorder[i + 1] + 1; k < LeftBorder[i]; k++)
                        {
                            Img[i][k] = IMG_BLACK;
                        }
                    }
                }
            }
            e_LeftBFF[i] = MEND;
        }
        else 
        {
            RightBorder[i] = s8DownX + (int8)(f_Add * (s8DownY - i) - 0.5);
            if (i == LINE_MAX)
            {
                for (int8 j = RightBorder[i]; j <= INDEX_MAX; j++)
                {
                    Img[i][j] = IMG_BLACK;
                }
            }
            else
            {
                Img[i][RightBorder[i]] = IMG_BLACK;
                if (ABS(RightBorder[i] - RightBorder[i + 1]) >= 2)
                {
                    if (f_Add < 0)
                    {
                        for (uint8 k = RightBorder[i + 1] - 1; k > RightBorder[i]; k--)
                        {
                            Img[i][k] = IMG_BLACK;
                        }
                    }
                }
            }
            e_RightBFF[i] = MEND;
        }
    }
}

void Border_Show(void)
{
    uint8 ImgBorder[CAMERA_SIZE] =
        {
            0};
    uint8 i; 

    for (i = LINE_MAX; i > g_u8Farline; i--)
    {
        ImgBorder[(LeftBorder[i] + i * CAMERA_W) / 8] += 0x01 << (7 - LeftBorder[i] % 8);
        ImgBorder[(RightBorder[i] + i * CAMERA_W) / 8] += 0x01 << (7 - RightBorder[i] % 8);
        ImgBorder[(MidBorder[i] + i * CAMERA_W) / 8] += 0x01 << (7 - MidBorder[i] % 8);
    }

    for (int16 j = g_u8Farline * CAMERA_W / 8; j < g_u8Farline * CAMERA_W / 8 + 10;
         j++)
    {
        ImgBorder[j] = 255;
    }

    vcan_sendimg(ImgBorder, CAMERA_SIZE); 
}

void Offset_Cal()
{
#if 0

    float f_DeltaX = (float)(g_u8RowMinX)-39.5;
    float f_DeltaY = (float)(g_u8Farline);
    float f_Y_Kp = 0.0f;
    float f_Offset1 = 0.0f;

    if (f_DeltaY > 50)
        f_Y_Kp = 0.40;
    else if (f_DeltaY > 40)
        f_Y_Kp = 0.32;
    else if (f_DeltaY > 30)
        f_Y_Kp = 0.24;
    else if (f_DeltaY > 20)
        f_Y_Kp = 0.16;
    else if (f_DeltaY > 10)
        f_Y_Kp = 0.08;
    else
        f_Y_Kp = 0.04;

    f_Offset1 = (7.25 * f_DeltaX * f_Y_Kp * (1 + f_Y_Kp) + 290.0 * f_Y_Kp * f_Y_Kp);
#endif

#if 0
#define OFFSETCALLINE 7             
#define CENTER 39.5                 
    uint8 u8CurDealLine = LINE_MAX; //当前处理行
    float f_UsefulRange = (float)(LINE_MAX - g_u8Farline) / (OFFSETCALLINE + 1);
    float WeightedCT[OFFSETCALLINE] = {0.03, 0.03, 0.03, 0.06, 0.60, 1.00, 0.10}; 
    float f_Sum_WeiCT = 0.0f;

    g_f_Offset = 0.0f;

    for (uint8 i = 0; i < OFFSETCALLINE; i++)
    {
        f_Sum_WeiCT += WeightedCT[i];
    }

    for (uint8 i = 1; i <= OFFSETCALLINE; i++)
    {
        u8CurDealLine = (uint8)(g_u8Farline + f_UsefulRange * i);
        if (e_BorderState[u8CurDealLine] == LeftMiss)
        {
            g_f_Offset += (RightBorder[u8CurDealLine] - Width[u8CurDealLine] / 2 - CENTER) * WeightedCT[OFFSETCALLINE - i];
        }
        else if (e_BorderState[u8CurDealLine] == RightMiss)
        {
            g_f_Offset += (LeftBorder[u8CurDealLine] + Width[u8CurDealLine] / 2 - CENTER) * WeightedCT[OFFSETCALLINE - i];
        }
        else
        {
            g_f_Offset += (MidBorder[u8CurDealLine] - CENTER) * WeightedCT[OFFSETCALLINE - i];
        }
    }

    g_f_Offset /= f_Sum_WeiCT;

    if (g_f_Offset > 40)
    {
        g_f_Offset = 40;
    }
    if (g_f_Offset < -40)
    {
        g_f_Offset = -40;
    }

    if (ABS(g_f_Offset) <= 1)
    {
        g_f_Offset = 0;
    }

    if (g_u8Farline > 35)
    {
        g_f_Offset *= 1.35;
    }

    else if (g_u8Farline > 24)
    {
        g_f_Offset *= 1.10;
    }

    else if (g_u8Farline < 15)
    {
        g_f_Offset *= 0.6;
    }

    //速度给定
    g_u32Speed = 40;
    //P值给定
    g_f_Kp = 21.0f;
    //D值给定
    g_f_Td = 0.8f;
#endif

#if 1
#define CENTER 39.5
    uint8 u8Kp_Cal = (RowBorder[39] - FARLINE)/10;

    //偏差计算
    if(g_u8Farline > 50)
    {
        g_f_Offset = MidBorder[g_u8Farline] - 39.5;
        if(g_f_Offset > 0)
        g_f_Offset += Width[LINE_MAX]/2.0;
        else if(g_f_Offset < 0)
        g_f_Offset -= Width[LINE_MAX]/2.0;
    }
    else if(g_u8Farline > 25)
    {
        g_f_Offset = MidBorder[g_u8Farline + 5] * 0.5 + MidBorder[g_u8Farline + 10] * 0.5 - 79;
        if(g_u8Farline > 35)
        {
            if(g_f_Offset > 0)
            g_f_Offset += Width[40]/3.0;
            else if(g_f_Offset < 0)
            g_f_Offset -= Width[40]/3.0;
        }
    }
    else
    {
        g_f_Offset = MidBorder[g_u8Farline + 10] * 0.4 + MidBorder[g_u8Farline + 20] * 0.6 - 79;
    }

    if (g_f_Offset > 40)
    {
        g_f_Offset = 40;
    }
    if (g_f_Offset < -40)
    {
        g_f_Offset = -40;
    }

    if (ABS(g_f_Offset) <= 1)
    {
        g_f_Offset = 0;
    }

    g_f_Kp = 6 + 2.5 * u8Kp_Cal + 0.6 * u8Kp_Cal * u8Kp_Cal;

    g_f_Td = 0.0f;

    LineStateDecide();

    if(currentLineState == straightline)
    {
        g_u32Speed = 100;
    }
    else
    {
        g_u32Speed = 50;
    }
#endif
}

void LineStateDecide(void) {
    #define N 8
    #define M 4
    #define POINT_TC TURNDECIDE
    #define POINT_TS STRAIGHTDECIDE
     static uint8 s_u8CurveCount = 0;
     static uint8 s_u8StraightCount = 0;
     uint8 u8TurnPoint = (RowBorder[CENTER_POINT]+RowBorder[CENTER_POINT+1])/2;

     switch(lastLineState) {
          case straightline: {
               if(u8TurnPoint>=POINT_TC&&s_u8CurveCount<N) {
                    currentLineState=straightline;
                    s_u8CurveCount++;
                    s_u8StraightCount=0;
               } else if(u8TurnPoint>=POINT_TC&&N==s_u8CurveCount) {
                    currentLineState=curveline;
                    s_u8CurveCount=N;
                    s_u8StraightCount=0;
               } else {
                    currentLineState=straightline;
                    s_u8CurveCount=0;
                    s_u8StraightCount=M;
               }
               break;
          }
          case curveline: {
               if(u8TurnPoint<POINT_TS&&s_u8StraightCount<M) {
                    currentLineState=curveline;
                    s_u8StraightCount++;
                    s_u8CurveCount=0;
               } else if(u8TurnPoint<POINT_TS&&s_u8StraightCount==M) {
                    currentLineState=straightline;
                    s_u8StraightCount=M;
                    s_u8CurveCount=0;
               } else {
                    currentLineState=curveline;
                    s_u8StraightCount=0;
                    s_u8CurveCount=N;
               }
               break;
          }
     }

     switch(lastCarState) {
          case turnleft: {
               break;
          }
          case gostraight: {
               break;
          }
          case turnright: {
               break;
          }
     }

     lastLineState=currentLineState;
     lastCarState=currentCarState;
}

bool Stopcar_Test(void)
{
    uint8 i, j;
    uint8 u8BlackPoint = 0;
    uint8 u8BlackLine = 0;

    for (j = 55; j > 45; j--, u8BlackPoint = 0)
    {
        for (i = 20; i < 60; i++)
        {
            if (Img[j][i] == IMG_BLACK)
                u8BlackPoint++;
            else
                continue;
        }

        if (u8BlackPoint > 33)
            u8BlackLine++;
    }

    if (u8BlackLine > 7)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void Send_Image(void)
{
    for (uint8 i = LINE_MAX; i > g_u8Farline; i--)
    {
        Img[i][LeftBorder[i]] = IMG_BLACK;
        Img[i][RightBorder[i]] = IMG_BLACK;
        Img[i][MidBorder[i]] = IMG_BLACK;
    }

    for (uint8 i = LeftBorder[g_u8Farline]; i <= RightBorder[g_u8Farline]; i++)
    {
        Img[g_u8Farline][i] = IMG_BLACK;
    }

    Img_Reduce((uint8 *)Img, ImgBuffer,
               CAMERA_H * CAMERA_W);     
    vcan_sendimg(ImgBuffer, CAMERA_SIZE); 
}

void Img_Reduce(uint8 *p_OriginalArray, uint8 *p_GetArray, uint32 u32Length)
{
    while (u32Length > 0)
    {
        *p_GetArray = 0;
        *p_GetArray += ((*p_OriginalArray++) & 0x01) << 7;
        *p_GetArray += ((*p_OriginalArray++) & 0x01) << 6;
        *p_GetArray += ((*p_OriginalArray++) & 0x01) << 5;
        *p_GetArray += ((*p_OriginalArray++) & 0x01) << 4;
        *p_GetArray += ((*p_OriginalArray++) & 0x01) << 3;
        *p_GetArray += ((*p_OriginalArray++) & 0x01) << 2;
        *p_GetArray += ((*p_OriginalArray++) & 0x01) << 1;
        *p_GetArray += ((*p_OriginalArray++) & 0x01) << 0;
        p_GetArray++;
        u32Length -= 8;
    }
}
