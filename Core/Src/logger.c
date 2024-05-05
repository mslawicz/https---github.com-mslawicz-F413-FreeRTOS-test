#include "logger.h"
#include "stdarg.h"

#define PRINT_BUF_LENGTH    80

char printBuf[PRINT_BUF_LENGTH];
enum LogLevel_t currentLogLevel = LVL_ERROR;
osMutexId_t* pLoggerMutexHandle = NULL;

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
        /* calculate timestamp components */
        uint32_t ticks = HAL_GetTick();
        uint8_t ms = ticks % 1000;
        ticks /= 1000;
        uint8_t sec = ticks % 60;
        ticks /= 60;
        uint8_t min = ticks % 60;
        ticks /= 60;
        uint8_t hour = ticks % 24;
        ticks /= 24;                
        /* aquire logger mutex */
        osMutexAcquire(*pLoggerMutexHandle, osWaitForever);
        /* place timestamp in the buffer */
        sprintf(printBuf, "[%lu:%d:%02d:%02d:%03d]", ticks, hour, min, sec, ms);
        size_t len = strlen(printBuf);
        /* place level severity text */
        strcpy(printBuf + len, pText);
        len += strlen(pText);
        printBuf[len++] = ':';
        /* terminating string */
        pText = "\n\r";
        /* process variadic arguments */
        va_list args;
        va_start(args, msg);
        vsnprintf(printBuf + len, PRINT_BUF_LENGTH - len - strlen(pText), msg, args);
        va_end(args);
        /* get the actual current string length */
        len = strlen(printBuf);
        /* add the terminating string */
        strcpy(printBuf + len, pText);
        len += strlen(pText);
        /* transmit the string over USB link */
        CDC_Transmit_FS((uint8_t*)printBuf, len);
        /* release logger mutex */
        osMutexRelease(*pLoggerMutexHandle);
        HAL_GPIO_WritePin(TEST4_GPIO_Port, TEST4_Pin, GPIO_PIN_RESET);
    }
}