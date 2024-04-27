#include "my_timer.h"
#include <stdlib.h>

osThreadId_t* pInjectTask = NULL;

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_SET);
    osThreadFlagsSet(*pInjectTask, INJECT_EVENT);
    HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET);
}