#ifndef __MY_TIMER_H
#define __MY_TIMER_H

#include "main.h"
#include "cmsis_os2.h"

#define INJECT_EVENT    1

extern osThreadId_t* pInjectTask;
extern TIM_HandleTypeDef* pEncoderTIM;
extern osTimerId_t* pMarkerLedTimerHandle;

#endif /* __MY_TIMER_H */