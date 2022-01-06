#ifndef _OLED_H
#define _0LED_H

extern void OLED_Init();
extern void OLED_CLS();

extern void OLED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
extern void OLED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
extern void OLED_P14x16Str(uint8 x,uint8 y,uint8 ch[]);
extern void OLED_P32x64Str(uint8 x,uint8 y,uint8 ch[]);

extern void OLED_P8x16_Clr_One(uint8 x,uint8 y);

extern void OLED_PutPixel(uint8 x,uint8 y);
extern void OLED_Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 gif);

extern void OLED_ShowData(uint8 x,uint8 y,int ad);
extern void OLED_ShowData_Float(uint8 x,uint8 y,float ad);

extern void OLED_Drawimg(); //Ð´Ó¥ÑÛÉãÏñÍ·º¯Êý

#endif


