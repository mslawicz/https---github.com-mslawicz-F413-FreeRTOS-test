#include "my_timer.h"
#include <stdlib.h>

osThreadId_t* pInjectTask = NULL;
TIM_HandleTypeDef* pEncoderTIM = NULL;

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_SET);
    osThreadFlagsSet(*pInjectTask, INJECT_EVENT);
    HAL_GPIO_WritePin(TEST1_GPIO_Port, TEST1_Pin, GPIO_PIN_RESET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == Marker_Int_Pin)
    {
        /* Marker pulse interrupt */
        HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_SET);
        pEncoderTIM->Instance->CNT = 0;     /* reset counter on MARKER pulse */
        HAL_GPIO_WritePin(TEST3_GPIO_Port, TEST3_Pin, GPIO_PIN_RESET);
    }
}