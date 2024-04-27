#include "my_timer.h"
#include <stdlib.h>

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
    htim->Instance->CCR1 = htim->Instance->CCR1 == 100 ? 600 : 100;
}