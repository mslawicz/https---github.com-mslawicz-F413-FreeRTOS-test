#ifndef __LOGGER_H
#define __LOGGER_H

#include "stdio.h"
#include "usbd_cdc_if.h"

#define PRINT_BUF_LENGTH    80

extern char printBuf[PRINT_BUF_LENGTH];

#define LOG(...) {\
    snprintf(printBuf, PRINT_BUF_LENGTH, __VA_ARGS__);\
    CDC_Transmit_FS((uint8_t*)printBuf, strlen(printBuf));\
}

#endif /* __LOGGER_H */