#ifndef __LOGGER_H
#define __LOGGER_H

#include "stdio.h"
#include "usbd_cdc_if.h"
#include "cmsis_os2.h"

extern osMutexId_t* pLoggerMutexHandle;

enum LogLevel_t
{
    LVL_DEBUG,
    LVL_INFO,
    LVL_WARNING,
    LVL_ERROR,
    LVL_NONE,
    LVL_ALWAYS
};

void logMessage(enum LogLevel_t level, const char* msg, ...);

#endif /* __LOGGER_H */