#include "my_timer.h"
#include <stdlib.h>

osThreadId_t* pLedToggleHandle = NULL;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == LED_Int_Pin)
    {
        osThreadFlagsSet(*pLedToggleHandle, LED_INT_EVENT);
    }
}

uint32_t lastCnt;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint16_t dif = abs(htim->Instance->CNT - lastCnt);
    if(dif < 10)
    {
        uint8_t k;
        for(k=0; k < dif; k++)
        {
            HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(TEST_GPIO_Port, TEST_Pin, GPIO_PIN_RESET);
        }
    }
    lastCnt = htim->Instance->CNT;
}