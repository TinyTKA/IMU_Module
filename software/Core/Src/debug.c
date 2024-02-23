#include "debug.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xff);
    return ch;
}

#ifdef debug


void ano_report(debug_info_dev *dev)
{
    dev->sc=0;
    dev->ac=0;
    dev->m_sendbuf[0] = dev->head;
    dev->m_sendbuf[1] = dev->addr;
    dev->m_sendbuf[2] = dev->id;
    dev->m_sendbuf[3] = dev->length;
    // 从数据缓冲区搬运到发送缓冲区

     for (uint8_t j = 0; j < dev->length; j++)
    {
        dev->m_sendbuf[4 + j] = dev->databuf[j];
    }
    for (uint8_t j = 0; j < dev->length + 4; j++)
    {
        dev->sc += dev->m_sendbuf[j];
        dev->ac += dev->sc;
    }
    dev->m_sendbuf[dev->length + 4] = dev->sc;
    dev->m_sendbuf[dev->length + 5] = dev->ac;
    HAL_UART_Transmit(&huart1,dev->m_sendbuf,dev->length+6,0xff);
}

#endif
