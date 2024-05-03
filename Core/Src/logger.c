#include "logger.h"
#include "stdarg.h"

#define PRINT_BUF_LENGTH    80

char printBuf[PRINT_BUF_LENGTH];
enum LogLevel_t currentLogLevel = LVL_INFO;

void _close(void)
{
}

void _lseek(void)
{
}

void _read(void)
{
}

void _write(void)
{
}

void logMessage(enum LogLevel_t level, const char* msg, ...)
{
    HAL_GPIO_WritePin(TEST4_GPIO_Port, TEST4_Pin, GPIO_PIN_SET);
    va_list args;
    va_start(args, msg);
    vsprintf(printBuf, msg, args);
    va_end(args);
    CDC_Transmit_FS((uint8_t*)printBuf, strlen(printBuf));
    HAL_GPIO_WritePin(TEST4_GPIO_Port, TEST4_Pin, GPIO_PIN_RESET);
}