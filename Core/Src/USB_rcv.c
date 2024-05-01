#include "USB_rcv.h"

void USB_rcvTask(void)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
  }    
}