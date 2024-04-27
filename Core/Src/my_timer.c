#include "my_timer.h"
#include <stdlib.h>

osThreadId_t* pInjectTask = NULL;

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    osThreadFlagsSet(*pInjectTask, INJECT_EVENT);
}