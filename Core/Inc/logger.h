#ifndef __LOGGER_H
#define __LOGGER_H

#include "stdio.h"
#include "usbd_cdc_if.h"

enum LogLevel_t
{
    LVL_NONE,
    LVL_ERROR,
    LVL_WARNING,
    LVL_INFO,
    LVL_DEBUG,
    LVL_ALL
};

void logMessage(enum LogLevel_t level, const char* msg, ...);

#endif /* __LOGGER_H */