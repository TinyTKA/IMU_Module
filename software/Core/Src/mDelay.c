#include "mDelay.h"

void delay_us(uint32_t us)
{

    __HAL_TIM_CLEAR_FLAG(&htim16,TIM_FLAG_UPDATE);
    while (us > 0)
    {
        __HAL_TIM_SET_COUNTER(&htim16,0);
        HAL_TIM_Base_Start(&htim16);
    
        while (!__HAL_TIM_GET_FLAG(&htim16,TIM_FLAG_UPDATE))
            ;
        us--;
         __HAL_TIM_CLEAR_FLAG(&htim16,TIM_FLAG_UPDATE);
    }
}