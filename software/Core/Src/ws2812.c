#include "ws2812.h"
#include "tim.h"

Color cols[pix_num];
uint32_t sendbuff[array_num+1];
uint32_t frameGap[24 * 10];

uint32_t num;
void WS2812_Init(void)
{
	
}
void WS2812_Clear(void)
{
}
void WS2812_Load(void)
{
    sendbuff[array_num]=0;
    //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, sendbuff, array_num+1);
}
void WS2812_Set_Pixel_Color(uint8_t num, Color *col)
{
    for (uint8_t i = 0; i < 24; i++)
    {
        if (i <= 7)
        {
            sendbuff[i] = ((col->r >> i) & 0x01) == 0x01 ? HIGH : LOW;
        }
        else if (i > 7 && i <= 15)
        {
            sendbuff[i] = ((col->g >> (i % 8)) & 0x01) == 0x01 ? HIGH : LOW;
        }
        else
        {
            sendbuff[i] = ((col->b >> (i % 16)) & 0x01) == 0x01 ? HIGH : LOW;
        }
    }
}
void WS2812_transfer(Color *col)
{

}
