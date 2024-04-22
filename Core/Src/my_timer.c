#include "my_timer.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == LED_Int_Pin)
    {
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    }
}