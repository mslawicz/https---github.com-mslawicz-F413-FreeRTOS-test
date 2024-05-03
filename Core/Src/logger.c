#include "logger.h"

char printBuf[PRINT_BUF_LENGTH];

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

void logMessage(void)
{
    HAL_GPIO_WritePin(TEST4_GPIO_Port, TEST4_Pin, GPIO_PIN_SET);
    CDC_Transmit_FS((uint8_t*)printBuf, strlen(printBuf));
    HAL_GPIO_WritePin(TEST4_GPIO_Port, TEST4_Pin, GPIO_PIN_RESET);
}