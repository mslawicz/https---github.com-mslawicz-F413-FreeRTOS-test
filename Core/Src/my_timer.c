#include "my_timer.h"

void LedTrigger(osTimerId_t timerHandle)
{
    for(;;)
    {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        osTimerStart(timerHandle, 50);
        osDelay(3333);
    }
}