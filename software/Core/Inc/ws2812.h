#ifndef __WS2812_H
#define __WS2812_H

#include "main.h"

#define pix_num 1
#define array_num ((pix_num) * 24)

#define HIGH 139
#define LOW 69

/*��ɫ�ṹ��*/
typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t light;
} Color;

typedef struct 
{
    uint8_t cnt;        //��������
    Color* Cols;        //������ɫ����ָ��
    uint8_t Col_num;    //������ɫ���鳤�ȣ�����������
    uint8_t* sendbuf;   //ת����ķ�����������
    uint16_t bufCnt;    //�������鳤��

}WS2812_Pixel_dev;

void WS2812_Init(void);
void WS2812_Clear(void);
void WS2812_Load(void);
void WS2812_Set_Pixel_Color(uint8_t num,Color *col);
void WS2812_transfer(Color* col);

#endif
