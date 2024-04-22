#ifndef __MY_TIMER_H
#define __MY_TIMER_H

#include "main.h"
#include "cmsis_os2.h"

#define LED_INT_EVENT   1

extern osThreadId_t* pLedToggleHandle;

#endif /* __MY_TIMER_H */