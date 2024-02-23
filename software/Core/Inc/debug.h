#ifndef __DEBUG_H
#define __DEBUG_H
#include "stdio.h"
#include "main.h"
#include "usart.h"
#define debug 

#define DATA_MUINT8  (uint8_t)1
#define DATA_MUINT16 (uint8_t)2
#define DATA_MFLOAT (uint8_t)3

#ifdef debug 

#define usart huart1    //ʹ�õĴ��ں�



typedef struct
{
    uint8_t head;
    uint8_t addr;
    uint8_t id;
    uint8_t length;
    uint8_t *databuf;
    uint8_t sc;
    uint8_t ac;
    uint8_t m_sendbuf[50];
} debug_info_dev;
/*
    ������λ���㱨����
    @param dataLength ���ݳ��ȣ���������8λ
    @param sendBuffer ���������ַ 
    @param code ֡��
*/
//void ano_report_Data(int dataLength,char* sendBuffer,char code);
void ano_report(debug_info_dev *dev);
#endif

#endif
