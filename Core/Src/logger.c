#include "logger.h"
#include "stdarg.h"

#define PRINT_BUF_LENGTH    80

char printBuf[PRINT_BUF_LENGTH];
enum LogLevel_t currentLogLevel = LVL_ERROR;

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
    if(level >= currentLogLevel)
    {
        const char* pText = NULL;
        switch((int)level)
        {
            case LVL_DEBUG:
            pText = "debug";
            break;

            case LVL_INFO:
            pText = "info";
            break;            

            case LVL_WARNING:
            pText = "warning";
            break;

            case LVL_ERROR:
            pText = "error";
            break;

            default:
            pText = "*";
            break;                        
        }
        
        strcpy(printBuf, pText);
        size_t len = strlen(pText);
        printBuf[len++] = ':';

        pText = "\n\r";

        va_list args;
        va_start(args, msg);
        len = len + vsnprintf(printBuf + len, /*PRINT_BUF_LENGTH*/20 - len - strlen(pText), msg, args);
        va_end(args);

        strcpy(printBuf + len, pText);
        len += strlen(pText);
        CDC_Transmit_FS((uint8_t*)printBuf, len);
        HAL_GPIO_WritePin(TEST4_GPIO_Port, TEST4_Pin, GPIO_PIN_RESET);
    }
}