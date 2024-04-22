#include "my_timer.h"

osThreadId_t* pLedToggleHandle = NULL;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == LED_Int_Pin)
    {
        osThreadFlagsSet(*pLedToggleHandle, LED_INT_EVENT);
    }
}